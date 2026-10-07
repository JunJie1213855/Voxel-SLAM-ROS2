import os
import sys
import open3d as o3d
import numpy as np

def main():
    # 配置参数
    base_folder = "/home/pix/code/mapping_ws/src/Voxel-SLAM/figure/indoor_fr"
    target_timestamp = 1776666854.0  # 替换为你需要的起始时间戳
    N = 30                 # 需要拼接的 pcd 数量
    downsample_voxel_size = 0.1
    
    pose_file = os.path.join(base_folder, "alidarState.txt")
    if not os.path.exists(pose_file):
        print(f"错误: 找不到文件 {pose_file}")
        return

    pose_data = [] # 存储 tuples: (timestamp, transformation_matrix, origin_index)
    
    # 读取位姿 alidarState.txt
    with open(pose_file, 'r') as f:
        for idx, line in enumerate(f):
            data = line.strip().split()
            if len(data) < 8:
                continue
            
            timestamp = float(data[0])
            tx, ty, tz = float(data[1]), float(data[2]), float(data[3])
            qx, qy, qz, qw = float(data[4]), float(data[5]), float(data[6]), float(data[7])
            
            R = np.array([
                [1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
                [2*qx*qy + 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw],
                [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy]
            ])
            
            T = np.eye(4)
            T[:3, :3] = R
            T[0, 3] = tx
            T[1, 3] = ty
            T[2, 3] = tz
            
            pose_data.append((timestamp, T, idx))

    print(f"共读取到 {len(pose_data)} 个位姿。")

    # 寻找最接近 target_timestamp 的起始帧
    timestamps = np.array([p[0] for p in pose_data])
    start_idx_in_list = np.argmin(np.abs(timestamps - target_timestamp))
    start_time, T_0, start_pcd_idx = pose_data[start_idx_in_list]
    
    print(f"目标时间戳: {target_timestamp}, 匹配到的起始帧时间戳: {start_time}, 第一帧对应的 pcd 序号: {start_pcd_idx}")
    
    # 计算起始帧对应的逆变换（作为0点）
    T_0_inv = np.linalg.inv(T_0)
    
    part_pcd = o3d.t.geometry.PointCloud()
    
    # 确定要遍历处理的帧范围
    end_idx_in_list = min(start_idx_in_list + N, len(pose_data))
    frames_to_process = pose_data[start_idx_in_list:end_idx_in_list]
    print(f"计划拼接从 {start_pcd_idx} 开始的 {len(frames_to_process)} 帧。")

    for i, (t, T_curr, original_pcd_idx) in enumerate(frames_to_process):
        pcd_path = os.path.join(base_folder, f"{original_pcd_idx}.pcd")
        
        if not os.path.exists(pcd_path):
            print(f"警告: 找不到 pcd 文件 {pcd_path}")
            continue
            
        pcd = o3d.t.io.read_point_cloud(pcd_path)
        if pcd.is_empty():
            continue
            
        pcd = pcd.voxel_down_sample(voxel_size=downsample_voxel_size)
        
        # 计算当前帧相对于起始帧(0点)的相对位姿
        T_rel = T_0_inv @ T_curr
        T_tensor = o3d.core.Tensor(T_rel, o3d.core.Dtype.Float64)
        
        pcd = pcd.transform(T_tensor)
        
        if part_pcd.is_empty():
            part_pcd = pcd
        else:
            part_pcd = part_pcd.append(pcd)
            
        if (i + 1) % 10 == 0 or (i + 1) == len(frames_to_process):
            print(f"已处理 {i + 1}/{len(frames_to_process)} 帧 ... (目前局部点云点数: {part_pcd.point.positions.shape[0]})")

    print(f"拼接完成！局部点云包含 {part_pcd.point.positions.shape[0]} 个点。")
    
    output_path = os.path.join(base_folder, f"part_map_{start_pcd_idx}_to_{original_pcd_idx}.pcd")
    o3d.t.io.write_point_cloud(output_path, part_pcd)
    print(f"局部地图已保存至: {output_path}")
    
    # ==== 可视化 ====
    print("启动 Open3D 可视化窗口 (按 Q 或关闭窗口退出)...")
    legacy_pcd = part_pcd.to_legacy()
    if "intensity" in part_pcd.point:
        try:
            import matplotlib.pyplot as plt
            intensities = part_pcd.point["intensity"].numpy().flatten()
            p2, p98 = np.percentile(intensities, (2, 98))
            intensities_clipped = np.clip(intensities, p2, p98)
            
            if p98 > p2:
                normalized_int = (intensities_clipped - p2) / (p98 - p2)
            else:
                normalized_int = np.zeros_like(intensities_clipped)
                
            cmap = plt.get_cmap("jet")(normalized_int)[:, :3]
            legacy_pcd.colors = o3d.utility.Vector3dVector(cmap)
        except ImportError:
            print("未安装 matplotlib，使用默认点云颜色。")

    o3d.visualization.draw_geometries([legacy_pcd], window_name="Part Map Stitching")

if __name__ == "__main__":
    main()