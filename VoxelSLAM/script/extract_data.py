import os
import shutil

def process_folder(in_path, out_folder_idx, out_path, step):
    pose_file = os.path.join(in_path, 'pose.json')
    if not os.path.exists(pose_file):
        print(f"Error: {pose_file} not found.")
        return

    with open(pose_file, 'r') as f:
        lines = f.readlines()
    
    out_pose_lines = []
    
    pcd_out_dir = os.path.join(out_path, str(out_folder_idx))
    os.makedirs(pcd_out_dir, exist_ok=True)
    
    new_idx = 0
    for i in range(0, len(lines), step):
        pcd_src = os.path.join(in_path, f"{i}.pcd")
        pcd_dst = os.path.join(pcd_out_dir, f"{new_idx}.pcd")
        
        if os.path.exists(pcd_src):
            shutil.copy(pcd_src, pcd_dst)
            out_pose_lines.append(lines[i])
            new_idx += 1
        else:
            print(f"Warning: {pcd_src} not found. Skipping to next possible valid pose.")
            # Depending on how the PCDs are generated, some frames might be skipped or missing.
            
    out_pose_file = os.path.join(out_path, 'original_pose', f"{out_folder_idx}.json")
    with open(out_pose_file, 'w') as f:
        f.writelines(out_pose_lines)
    
    print(f"Extracted {new_idx} frames from {in_path}")

def extract_data(ref_path, trans_path, out_path, step=10):
    print(f"Extracting data with step {step}...")
    os.makedirs(out_path, exist_ok=True)
    os.makedirs(os.path.join(out_path, 'original_pose'), exist_ok=True)
    
    process_folder(ref_path, 0, out_path, step)
    process_folder(trans_path, 1, out_path, step)
    
    print(f"Extraction complete! Data saved to {os.path.abspath(out_path)}\/")

if __name__ == "__main__":
    # 配置绝对路径 (需要根据你实际存放的数据情况修改)
    ref_path = "/home/pix/code/mapping_ws/src/Voxel-SLAM/figure/test_ft_circle"
    trans_path = "/home/pix/code/mapping_ws/src/Voxel-SLAM/figure/test_rt_circle"
    
    # 输出的指定文件夹
    out_path = "/home/pix/code/calibration_ws/src/mlcc/scene_suzhou"
    
    # 设置抽帧步长 (例如10)
    step = 10
    
    extract_data(ref_path, trans_path, out_path, step)