#pragma once

#include <string>
#include <thread>
#include <chrono>
#include <functional>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <rclcpp/serialization.hpp>

extern bool g_running;

class Ros2BagReader {
public:
    Ros2BagReader(const std::string& bag_path, const std::string& imu_topic, const std::string& lid_topic,
                  std::function<void(sensor_msgs::msg::Imu::ConstSharedPtr)> imu_callback,
                  std::function<void(sensor_msgs::msg::PointCloud2::ConstSharedPtr)> lid_callback)
        : bag_path_(bag_path), imu_topic_(imu_topic), lid_topic_(lid_topic),
          imu_callback_(imu_callback), lid_callback_(lid_callback),
          is_running_(false), should_stop_(false) {}

    ~Ros2BagReader() {
        stop();
    }

    void start() {
        if (is_running_) return;
        is_running_ = true;
        should_stop_ = false;
        worker_thread_ = std::thread(&Ros2BagReader::play, this);
    }

    void stop() {
        should_stop_ = true;
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
        is_running_ = false;
    }

    void play() {
        // reader.open() throws std::runtime_error when the path is not a valid
        // rosbag2 directory. This runs on the worker thread, so an uncaught
        // exception here would call std::terminate and abort the process;
        // catch it and shut the run down cleanly instead.
        try {
            play_impl();
        } catch (const std::exception & e) {
            RCLCPP_ERROR(rclcpp::get_logger("Ros2BagReader"),
                "Failed to open or read rosbag2 at '%s': %s", bag_path_.c_str(), e.what());
            RCLCPP_ERROR(rclcpp::get_logger("Ros2BagReader"),
                "Check that 'General.bag_path' points to a valid rosbag2 directory. Shutting down.");
            // Every worker loop polls on `g_running && rclcpp::ok()`, so clearing
            // both lets all threads unwind instead of hanging on missing data.
            g_running = false;
            rclcpp::shutdown();
        }
        is_running_ = false;
    }

    void set_finish_callback(std::function<void()> cb) {
        finish_callback_ = cb;
    }

private:
    // Livox publishes its own point format instead of sensor_msgs/PointCloud2.
    // Convert it to a PointCloud2 so the rest of the pipeline (which only knows
    // about PointCloud2) can consume it unchanged.
    //
    // Layout matches the other lidar handlers: `intensity` carries the
    // reflectivity, and `time` carries the per-point offset in seconds relative
    // to the frame start. The frame start itself goes into header.stamp, which
    // is what the point handler returns as t0 -- sync_packages() relies on
    // curvature being relative to it (pcl_end_time = pcl_beg_time + curvature).
    static sensor_msgs::msg::PointCloud2::ConstSharedPtr customToPointCloud2(
        const livox_ros_driver2::msg::CustomMsg & msg)
    {
        auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        cloud->header.frame_id = msg.header.frame_id;
        cloud->header.stamp = msg.header.stamp;
        cloud->height = 1;
        cloud->width = msg.point_num;
        cloud->is_bigendian = false;
        cloud->is_dense = false;
        cloud->point_step = 20;
        cloud->row_step = cloud->point_step * cloud->width;

        auto make_field = [](const std::string & name, uint32_t offset) {
            sensor_msgs::msg::PointField f;
            f.name = name;
            f.offset = offset;
            f.datatype = sensor_msgs::msg::PointField::FLOAT32;
            f.count = 1;
            return f;
        };
        cloud->fields = {
            make_field("x", 0),
            make_field("y", 4),
            make_field("z", 8),
            make_field("intensity", 12),
            make_field("time", 16),
        };

        cloud->data.resize(static_cast<size_t>(cloud->width) * cloud->point_step);
        uint8_t * out = cloud->data.data();
        for (uint32_t i = 0; i < msg.point_num; i++) {
            const auto & p = msg.points[i];
            float * dst = reinterpret_cast<float *>(out + static_cast<size_t>(i) * cloud->point_step);
            dst[0] = p.x;
            dst[1] = p.y;
            dst[2] = p.z;
            dst[3] = static_cast<float>(p.reflectivity);
            dst[4] = static_cast<float>(p.offset_time) * 1e-9f;  // ns -> s, relative to timebase
        }
        return cloud;
    }

    void play_impl() {
        rosbag2_cpp::Reader reader;
        reader.open(bag_path_);

        // The lidar topic is either a plain PointCloud2 or a Livox CustomMsg
        // depending on the recording. Look up the recorded type and pick the
        // matching deserializer -- feeding one into the other silently produces
        // garbage points rather than an error.
        bool lidar_is_livox = false;
        for (const auto & topic : reader.get_all_topics_and_types()) {
            if (topic.name == lid_topic_) {
                lidar_is_livox = (topic.type == "livox_ros_driver2/msg/CustomMsg");
                break;
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("Ros2BagReader"), "Lidar topic '%s' uses %s",
            lid_topic_.c_str(), lidar_is_livox ? "livox_ros_driver2/msg/CustomMsg" : "sensor_msgs/msg/PointCloud2");

        rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pcl_serialization;
        rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> livox_serialization;

        bool first_message = true;
        rclcpp::Time first_msg_time;
        auto play_start_time = std::chrono::steady_clock::now();

        while (reader.has_next() && rclcpp::ok() && !should_stop_) {
            auto bag_message = reader.read_next();
            rclcpp::Time msg_time(bag_message->time_stamp);

            if (first_message) {
                first_msg_time = msg_time;
                play_start_time = std::chrono::steady_clock::now();
                first_message = false;
            } else {
                auto time_to_wait = (msg_time - first_msg_time).nanoseconds() - 
                                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::steady_clock::now() - play_start_time).count();
                if (time_to_wait > 0) {
                    std::this_thread::sleep_for(std::chrono::nanoseconds(time_to_wait));
                }
            }

            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);

            if (bag_message->topic_name == imu_topic_) {
                auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
                imu_serialization.deserialize_message(&extracted_serialized_msg, imu_msg.get());
                imu_callback_(imu_msg);
            } else if (bag_message->topic_name == lid_topic_) {
                if (lidar_is_livox) {
                    auto livox_msg = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
                    livox_serialization.deserialize_message(&extracted_serialized_msg, livox_msg.get());
                    lid_callback_(customToPointCloud2(*livox_msg));
                } else {
                    auto pcl_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
                    pcl_serialization.deserialize_message(&extracted_serialized_msg, pcl_msg.get());
                    lid_callback_(pcl_msg);
                }
            }
        }
        
        if (!should_stop_) 
        {
            RCLCPP_INFO(rclcpp::get_logger("Ros2BagReader"), "Bag playback finished. Triggering node finish flag...");
            if (finish_callback_) {
                finish_callback_();
            }
        }
    }

    std::string bag_path_;
    std::string imu_topic_;
    std::string lid_topic_;
    std::function<void(sensor_msgs::msg::Imu::ConstSharedPtr)> imu_callback_;
    std::function<void(sensor_msgs::msg::PointCloud2::ConstSharedPtr)> lid_callback_;
    std::function<void()> finish_callback_;
    std::atomic<bool> is_running_;
    std::atomic<bool> should_stop_;
    std::thread worker_thread_;
};
