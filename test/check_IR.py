import os
import shutil
import datetime

def extract_last_images_from_all_ir_folders(root_folder, target_folder):
    # 检查目标文件夹是否存在，不存在则创建
    if not os.path.exists(target_folder):
        os.makedirs(target_folder)
    
    # 遍历 root_folder 中的所有子文件夹
    for subdir, _, _ in os.walk(root_folder):
        # 查找名为 IR 的文件夹
        if os.path.basename(subdir) == "IR":
            # 获取 IR 文件夹中的所有文件
            files = [os.path.join(subdir, f) for f in os.listdir(subdir) if os.path.isfile(os.path.join(subdir, f))]
            
            # 确保文件夹中有文件
            if files:
                # 找到最后修改的文件
                latest_file = max(files, key=os.path.getmtime)
                
                # 构造目标文件路径，保留文件名并区分来源文件夹
                relative_path = os.path.relpath(subdir, root_folder)  # 获取相对路径
                target_subfolder = os.path.join(target_folder, relative_path)  # 构造目标子文件夹路径
                if not os.path.exists(target_subfolder):
                    os.makedirs(target_subfolder)
                
                # 复制文件到目标文件夹
                shutil.copy(latest_file, target_subfolder)
                print(f"最新的图片 {os.path.basename(latest_file)} 已复制到 {target_subfolder}。")

def copy_latest_images_by_id(root_folder, target_folder, exts=(".png", ".jpg", ".jpeg", ".bmp", ".tiff")):
    """
    目录结构假设：
    root_folder/
      ├─ ID_xxx/
      │    ├─ IR/
      │    ├─ RGB/
      │    └─ depth/
      ├─ ID_yyy/
      │    ├─ IR/
      │    └─ RGB/
      └─ ...

    只遍历每个 ID 目录下的【直接子文件夹】（IR/RGB/depth 等），
    在每个子文件夹中选择“最新修改时间”的图片文件复制到
    target_folder/ID/子文件夹/ 下。
    """
    if not os.path.exists(target_folder):
        os.makedirs(target_folder)

    # 仅遍历 root_folder 的第一层（ID 层）
    for id_name in os.listdir(root_folder):
        id_path = os.path.join(root_folder, id_name)
        if not os.path.isdir(id_path):
            continue  # 跳过非目录
        # 遍历 ID 下的直接子文件夹
        for sub_name in os.listdir(id_path):
            sub_path = os.path.join(id_path, sub_name)
            if not os.path.isdir(sub_path):
                continue

            # 收集该子文件夹中的图片文件
            files = []
            try:
                for f in os.listdir(sub_path):
                    fp = os.path.join(sub_path, f)
                    if os.path.isfile(fp) and f.lower().endswith(exts):
                        files.append(fp)
            except PermissionError:
                print(f"[跳过] 无权限读取: {sub_path}")
                continue

            if not files:
                # 没有图片就跳过
                # print(f"[空] {id_name}/{sub_name} 无图片")
                continue

            # 找到修改时间最新的文件
            latest_file = max(files, key=os.path.getmtime)

            # 目标路径：target/ID/子文件夹/
            dst_dir = os.path.join(target_folder, id_name, sub_name)
            os.makedirs(dst_dir, exist_ok=True)
            try:
                shutil.copy2(latest_file, dst_dir)  # 保留mtime等元数据
                print(f"[复制] {id_name}/{sub_name} → {os.path.basename(latest_file)}")
            except Exception as e:
                print(f"[失败] 复制 {latest_file} 到 {dst_dir}：{e}")

# 示例用法
root_folder = "/home/pi/Desktop/ROBOTSOFTWARE/Data/Data_Estrus_2025_08"  # 替换为你的根文件夹路径
time = datetime.datetime.now()
target_folder = f"Latest_IR_Images/{str(time.date())}_{str(time.hour)}_check"  # 替换为你的目标文件夹路径

copy_latest_images_by_id(root_folder, target_folder)
