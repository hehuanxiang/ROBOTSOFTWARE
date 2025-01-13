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

# 示例用法
root_folder = "/home/pi/Desktop/ROBOTSOFTWARE/Data/Data_Estrus_2024_12"  # 替换为你的根文件夹路径
time = datetime.datetime.now()
target_folder = f"Latest_IR_Images/{str(time.date())}_{str(time.hour)}_check"  # 替换为你的目标文件夹路径

extract_last_images_from_all_ir_folders(root_folder, target_folder)
