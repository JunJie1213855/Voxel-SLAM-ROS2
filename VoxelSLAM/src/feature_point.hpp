#ifndef FEATURE_POINT_HPP
#define FEATURE_POINT_HPP

#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

typedef pcl::PointXYZINormal PointType;
using namespace std;

// Numbering matches the upstream ROS 1 version: LIVOX is 0, which is what the
// shipped Livox configs (mid360/avia/avia_fly) specify.
enum LID_TYPE{LIVOX, VELODYNE, OUSTER, HESAI, ROBOSENSE, TARTANAIR};

namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      // float intensity;
      float time;
      std::uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    // (float, intensity, intensity)
    (float, time, time)
    (std::uint16_t, ring, ring)
)

namespace ouster_ros 
{
  struct EIGEN_ALIGN16 Point 
  {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t  ring;
    // uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  // use std::uint32_t to avoid conflicting with pcl::uint32_t
  (std::uint32_t, t, t)
  // (std::uint16_t, reflectivity, reflectivity)
  // (std::uint8_t, ring, ring)
  // (std::uint16_t, ambient, ambient)
  // (std::uint32_t, range, range)
)

namespace xt32_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      double timestamp;
      uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(xt32_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, timestamp, timestamp)
    (std::uint16_t, ring, ring)
)


namespace rslidar_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      std::uint16_t ring;
      double timestamp;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(rslidar_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (double, timestamp, timestamp)
)

// Layout of the PointCloud2 produced by Ros2BagReader::customToPointCloud2()
// from a Livox CustomMsg: xyz + reflectivity + per-point offset time in seconds.
namespace livox_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      float time;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace livox_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
)

class Features
{
public:
  int lidar_type, point_filter_num;
  double blind = 1;
  double omega_l = 3610;

  double process(const sensor_msgs::msg::PointCloud2::ConstPtr &msg, pcl::PointCloud<PointType> &pl_full)
  {
    double t0 = rclcpp::Time(msg->header.stamp).seconds();
    switch(lidar_type)
    {
    case VELODYNE:
      velodyne_handler(msg, pl_full);
      break;

    case OUSTER:
      ouster_handler(msg, pl_full);
      break;

    case HESAI:
      hesai_handler(msg, pl_full);
      break;
    
    case ROBOSENSE:
      t0 = robosense_handler(msg, pl_full);
      break;
    
    case TARTANAIR:
      tartanair_handler(msg, pl_full);
      break;

    case LIVOX:
      livox_handler(msg, pl_full);
      break;

    default:
      printf("Lidar Type Error\n");
      exit(0);
    }

    return t0;
  }

  // Consumes the PointCloud2 that Ros2BagReader builds out of a Livox CustomMsg.
  // `time` holds the per-point offset in seconds relative to header.stamp, so
  // curvature keeps the relative-to-frame-start semantics that sync_packages()
  // and motion_blur() expect (see sync_packages: pcl_end_time = beg + curvature).
  void livox_handler(const sensor_msgs::msg::PointCloud2::ConstPtr &msg, pcl::PointCloud<PointType> &pl_full)
  {
    pcl::PointCloud<livox_ros::Point> pl_orig;
    // Qualified: Features has its own rslidar_ros overload of fromROSMsg which
    // would otherwise shadow the PCL one via unqualified lookup.
    pcl::fromROSMsg(*msg, pl_orig);

    int plsize = pl_orig.points.size();
    pl_full.reserve(plsize);
    for(int i=0; i<plsize; i++)
    {
      PointType ap;
      ap.x = pl_orig.points[i].x;
      ap.y = pl_orig.points[i].y;
      ap.z = pl_orig.points[i].z;
      ap.intensity = pl_orig.points[i].intensity;
      ap.curvature = pl_orig.points[i].time;

      if(i % point_filter_num == 0)
      {
        if(ap.x*ap.x + ap.y*ap.y + ap.z*ap.z > blind)
        {
          pl_full.points.push_back(ap);
        }
      }
    }
  }

  void velodyne_handler(const sensor_msgs::msg::PointCloud2::ConstPtr &msg, pcl::PointCloud<PointType> &pl_full)
  {
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);

    int plsize = pl_orig.size();
    if(plsize == 0) return;
    if(pl_orig.back().time > 0.01 && pl_orig.back().time < 0.12)
    {
      // for(velodyne_ros::Point &iter : pl_orig.points)
      for(int i=0; i<plsize; i++)
      {
        velodyne_ros::Point &iter = pl_orig[i];
        PointType ap;
        ap.x = iter.x; ap.y = iter.y; ap.z = iter.z;
        
        // ap.intensity = iter.intensity;
        // ap.curvature = iter.time * 1e-3; // ms
        // ap.curvature = iter.time * 1e-6;
        ap.curvature = iter.time;

        if(i % point_filter_num == 0)
        {
          if(ap.x*ap.x + ap.y*ap.y + ap.z*ap.z > blind)
          {
            pl_full.push_back(ap);
          }
        }
      }

    }
    else
    {
      // lidar clockwise rotate
      bool first_point = true;
      double yaw_first = 0;
      double yaw_last = 0;
      double yaw_bias = 0;
      int cool = 0;
      float max_ang = 0;
      for(int i=0; i<plsize; i++)
      {
        cool--;
        velodyne_ros::Point &iter = pl_orig[i];
        PointType ap;
        ap.x = iter.x; ap.y = iter.y; ap.z = iter.z;

        if(fabs(ap.x) < 0.1)
          continue;
        
        double yaw_angle = atan2(ap.y, ap.x) * 57.2957 - yaw_bias;
        if(first_point)
        {
          yaw_first = yaw_angle;
          yaw_last  = yaw_angle;
          first_point = false;
        }

        if(ap.x*ap.x + ap.y*ap.y + ap.z*ap.z < blind)
          continue;

        if(yaw_angle - yaw_last > 180 && cool <= 0)
        {
          yaw_bias += 360; yaw_angle-= 360; cool = 1000;
        }

        if(fabs(yaw_angle - yaw_last) > 180)
        {
          yaw_angle += 360;
        }

        ap.curvature = (yaw_first - yaw_angle) / omega_l;
        yaw_last = yaw_angle;

        if(ap.curvature > max_ang)
          max_ang = ap.curvature;

        if(ap.curvature >= 0 && ap.curvature < 0.1)
          if(i % point_filter_num == 0)
            pl_full.push_back(ap);
      }

      // printf("maxang: %f\n", max_ang);
    }

  }

  void ouster_handler(const sensor_msgs::msg::PointCloud2::ConstPtr &msg, pcl::PointCloud<PointType> &pl_full)
  {
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);

    int plsize = pl_orig.points.size();
    pl_full.reserve(plsize);
    for(int i=0; i<plsize; i++)
    {
      PointType ap;
      ap.x = pl_orig.points[i].x;
      ap.y = pl_orig.points[i].y;
      ap.z = pl_orig.points[i].z;
      ap.intensity = pl_orig[i].intensity;
      // ap.curvature = pl_orig[i].t / float(1e6); // ms
      ap.curvature = pl_orig[i].t / float(1e9); // s

      if(i % point_filter_num == 0)
      {
        if(ap.x*ap.x + ap.y*ap.y + ap.z*ap.z > blind)
        {
          pl_full.points.push_back(ap);
        }
      }

    }

  }

  void hesai_handler(const sensor_msgs::msg::PointCloud2::ConstPtr &msg, pcl::PointCloud<PointType> &pl_full)
  { 
    pcl::PointCloud<xt32_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);

    int plsize = pl_orig.points.size();
    pl_full.reserve(plsize);
    double time_head = pl_orig.points[0].timestamp;
    for(int i=0; i<plsize; i++)
    {
      PointType added_pt;

      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = (pl_orig.points[i].timestamp - time_head);

      if (i % point_filter_num == 0)
      {
        if (added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > blind)
        {
          pl_full.points.push_back(added_pt);
        }
      }


    }

  }

  bool fromROSMsg(const sensor_msgs::msg::PointCloud2& msg, pcl::PointCloud<rslidar_ros::Point>& cloud)
  {
      cloud.clear();
      
      if (msg.data.empty() || msg.height * msg.width == 0) {
          return false;
      }
      
      // 查找字段偏移量
      uint32_t x_offset = 0, y_offset = 0, z_offset = 0;
      uint32_t intensity_offset = 0, ring_offset = 0, timestamp_offset = 0;
      
      bool has_x = false, has_y = false, has_z = false;
      bool has_intensity = false, has_ring = false, has_timestamp = false;

      int intensity_data_type = 0;
      
      for (const auto& field : msg.fields) {
          if (field.name == "x") {
              x_offset = field.offset;
              has_x = true;
          } else if (field.name == "y") {
              y_offset = field.offset;
              has_y = true;
          } else if (field.name == "z") {
              z_offset = field.offset;
              has_z = true;
          } else if (field.name == "intensity") {
              intensity_offset = field.offset;
              intensity_data_type = field.datatype;
              has_intensity = true;
          } else if (field.name == "channel") { // 对应结构体中的 ring
              ring_offset = field.offset;
              has_ring = true;
          } else if (field.name == "time_stamp") { // 对应结构体中的 timestamp
              timestamp_offset = field.offset;
              has_timestamp = true;
          }
      }
      
      if (!has_x || !has_y || !has_z) {
          return false; // 缺少核心坐标字段
      }
      
      // 准备点云容量
      cloud.width = msg.width;
      cloud.height = msg.height;
      cloud.is_dense = (msg.is_dense == 1);
      cloud.resize(cloud.width * cloud.height);
      
      // 逐点转换
      for (size_t i = 0; i < cloud.size(); ++i) {
          rslidar_ros::Point& point = cloud.points[i];
          const uint8_t* data_ptr = &msg.data[i * msg.point_step];    
              
          // 1. 拷贝 XYZ (FLOAT32)
          std::memcpy(&point.x, data_ptr + x_offset, sizeof(float));
          std::memcpy(&point.y, data_ptr + y_offset, sizeof(float));
          std::memcpy(&point.z, data_ptr + z_offset, sizeof(float));
          
          // 2. 拷贝 Intensity
          if (has_intensity) {
              if (intensity_data_type == sensor_msgs::msg::PointField::UINT8) {
                  point.intensity = static_cast<float>(data_ptr[intensity_offset]);
              } else if (intensity_data_type == sensor_msgs::msg::PointField::FLOAT32) {
                  std::memcpy(&point.intensity, data_ptr + intensity_offset, sizeof(float));
              }
          } else {
              point.intensity = 0.0f;
          }

          // 3. 拷贝 Ring (对应消息中的 channel, UINT16)
          if (has_ring) {
              std::memcpy(&point.ring, data_ptr + ring_offset, sizeof(uint16_t));
          } else {
              point.ring = 0;
          }

          // 4. 拷贝 Timestamp (对应消息中的 time_stamp, FLOAT64/double)
          if (has_timestamp) 
          {
            int32_t timestamp_value;
            std::memcpy(&timestamp_value, data_ptr + timestamp_offset, sizeof(int32_t));
            point.timestamp = static_cast<float>(timestamp_value) *1.0e-9; // 转换为秒
          } else {
              point.timestamp = 0.0f;
          }
      }
      
      return true;
  }

  double robosense_handler(const sensor_msgs::msg::PointCloud2::ConstPtr &msg, pcl::PointCloud<PointType> &pl_full)
  {
    pcl::PointCloud<rslidar_ros::Point> pl_orig;
    fromROSMsg(*msg, pl_orig);

    int plsize = pl_orig.points.size();
    pl_full.reserve(plsize);
    // double t0 = pl_orig[0].timestamp;
    double t0 = rclcpp::Time(msg->header.stamp).seconds();
    double time_max = 0.0;
    double time_min = 1e9;
    for(int i=0; i<plsize; i++)
    {
      PointType ap;
      ap.x = pl_orig.points[i].x;
      ap.y = pl_orig.points[i].y;
      ap.z = pl_orig.points[i].z;
      ap.intensity = pl_orig.points[i].intensity;
      // ap.curvature = (pl_orig[i].timestamp - t0) * float(1e3); //
      ap.curvature = pl_orig[i].timestamp;

      if(ap.curvature > time_max)
        time_max = ap.curvature;
      if(ap.curvature < time_min)
        time_min = ap.curvature;


      if(i % point_filter_num == 0)
      {
        if(ap.x*ap.x + ap.y*ap.y + ap.z*ap.z > blind)
        {
          pl_full.points.push_back(ap);
        }
      }

    }

    return t0;
  }

  void tartanair_handler(const sensor_msgs::msg::PointCloud2::ConstPtr &msg, pcl::PointCloud<PointType> &pl_full)
  {
    pcl::PointCloud<pcl::PointXYZ> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    pl_full.reserve(pl_orig.size());

    PointType pp; pp.curvature = 0;
    for(pcl::PointXYZ &ap: pl_orig.points)
    {
      pp.x = ap.x;
      pp.y = ap.y;
      pp.z = ap.z; 
      pl_full.push_back(pp);
    }

    return;
  }

};

#endif
