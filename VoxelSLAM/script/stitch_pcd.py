import os
import sys
import open3d as o3d
import numpy as np

def main():
    # base_folder = "/home/pix/code/mapping_ws/src/Voxel-SLAM/figure/test"
    base_folder = "/home/pix/code/mapping_ws/src/Voxel-SLAM/figure/indoor_mapping"
    pose_file = os.path.join(base_folder, "alidarState.txt")
    
    if not os.path.exists(pose_file):
        print(f"错误: 找不到文件 {pose_file}")
        return

    poses = []
    # 读取位姿 alidarState.txt
    # 格式: t px py pz qx qy qz qw vx vy vz bgx bgy bgz bax bay baz gx gy gz v6...
    with open(pose_file, 'r') as f:
        for line in f:
            data = line.strip().split()
            if len(data) < 8:
                continue
            
            # 提取平移
            tx, ty, tz = float(data[1]), float(data[2]), float(data[3])
            # 提取四元数 (x, y, z, w)
            qx, qy, qz, qw = float(data[4]), float(data[5]), float(data[6]), float(data[7])
            
            # 手动从四元数计算旋转矩阵，防止不同库顺序规范不同
            x, y, z, w = qx, qy, qz, qw
            R = np.array([
                [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
                [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
                [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
            ])
            
            # 创建4x4变换矩阵
            T = np.eye(4)
            T[:3, :3] = R
            T[0, 3] = tx
            T[1, 3] = ty
            T[2, 3] = tz
            poses.append(T)

    print(f"共读取到 {len(poses)} 个位姿。")

    global_pcd = o3d.t.geometry.PointCloud()
    downsample_voxel_size = 0.2  # 可根据点云稠密程度自定义降采样体素大小
    
    # 遍历位姿与对应的pcd合并
    for i, T in enumerate(poses):
        pcd_path = os.path.join(base_folder, f"{i}.pcd")
        
        if not os.path.exists(pcd_path):
            continue
            
        pcd = o3d.t.io.read_point_cloud(pcd_path)
        if pcd.is_empty():
            continue
            
        # 降采样
        pcd = pcd.voxel_down_sample(voxel_size=downsample_voxel_size)
        
        # 坐标变换 (用得到的位姿阵将当前帧投影到全局坐标系)
        T_tensor = o3d.core.Tensor(T, o3d.core.Dtype.Float64)
        pcd = pcd.transform(T_tensor)
        
        # 拼接到全局
        if global_pcd.is_empty():
            global_pcd = pcd
        else:
            global_pcd = global_pcd.append(pcd)
        
        if (i + 1) % 10 == 0:
            print(f"已处理 {i + 1}/{len(poses)} 帧 ... (目前点云总点数: {global_pcd.point.positions.shape[0]})")

    print(f"拼接完成！全局点云包含 {global_pcd.point.positions.shape[0]} 个点。")
    
    # 再次做一次全局降采样，减少内存负担
    # print("正在进行全局降采样...")
    # global_pcd = global_pcd.voxel_down_sample(voxel_size=downsample_voxel_size)
    # print(f"全局降采样后包含 {global_pcd.point.positions.shape[0]} 个点。")
    
    output_path = os.path.join(base_folder, "global_map.pcd")
    o3d.t.io.write_point_cloud(output_path, global_pcd)
    print(f"全局地图已保存至: {output_path} (保留了 intensity 等属性)")
    
    # 可视化展示 (转为传统点云用于可视化或直接使用 draw)
    print("正在启动 Open3D 可视化窗口 (按 Q 或关闭窗口退出)...")
    
    legacy_pcd = global_pcd.to_legacy()
    # 为了将 intensity 可视化，我们将其映射为伪色彩 (Jet)
    if "intensity" in global_pcd.point:
        try:
            import matplotlib.pyplot as plt
            intensities = global_pcd.point["intensity"].numpy().flatten()
            
            # 使用百分位数进行截断，避免异常极值导致颜色映射失真
            p2, p98 = np.percentile(intensities, (2, 98))
            intensities_clipped = np.clip(intensities, p2, p98)
            
            if p98 > p2:
                normalized_int = (intensities_clipped - p2) / (p98 - p2)
            else:
                normalized_int = np.zeros_like(intensities_clipped)
            
            # 使用 jet 伪彩色映射
            cmap = plt.get_cmap("jet")(normalized_int)[:, :3]
            legacy_pcd.colors = o3d.utility.Vector3dVector(cmap)
            print("已将 intensity 映射为 Jet 伪色彩。")
        except ImportError:
            print("未安装 matplotlib，使用默认点云颜色。")

    o3d.visualization.draw_geometries([legacy_pcd], window_name="Global Map Stitching")

if __name__ == "__main__":
    main()
