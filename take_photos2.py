import cv2
import os
import sys
import argparse
import shutil  # 用于检查磁盘空间（兼容更多系统）
import os.path as osp  # 用于获取文件大小

def take_photos(save_dir="/home/bcsh/photos", date_prefix="20260316"):
    """
    打开摄像头，按 's' 键拍摄照片并保存为 640x480 尺寸，名称格式为 日期_序号.jpg（如20260316_00001.jpg）。
    按 'q' 键退出。
    保存路径默认改为鲁班猫本地目录：/home/bcsh/photos
    （已屏蔽串口依赖，仅保留纯拍照功能）
    """
    # 确保保存目录存在（不存在则自动创建）
    try:
        os.makedirs(save_dir, exist_ok=True)
        print(f"\n=== 初始化完成 ===")
        print(f"📁 照片保存目录: {save_dir}")
        print(f"📅 当前日期前缀: {date_prefix}")
    except Exception as e:
        print(f"❌ 错误：无法创建保存目录 {save_dir} - {e}")
        return
    
    # 检查本地目录可用空间（至少1GB）
    try:
        # 使用 shutil 兼容更多Linux系统（替代statvfs）
        disk_usage = shutil.disk_usage(save_dir)
        free_space_gb = disk_usage.free / (1024**3)
        if free_space_gb < 1:
            print(f"⚠️  警告：{save_dir} 可用空间不足1GB ({free_space_gb:.2f}GB)，可能影响保存！")
        else:
            print(f"✅ 本地目录可用空间：{free_space_gb:.2f}GB（满足要求）")
    except Exception as e:
        print(f"⚠️  检查本地空间时出错（不影响保存）: {e}")
    
    try:
        # 方案一核心修改：直接用OpenCV打开摄像头，绕过SDK的串口依赖
        # 0 是默认摄像头设备号，如果识别不到可尝试改为 1、2 等
        cap = cv2.VideoCapture(0)
        # 设置摄像头分辨率（可选，保证输出基础分辨率）
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        if not cap.isOpened():
            print(f"❌ 错误：无法打开摄像头。请检查摄像头硬件连接或设备号！")
            return
        
        # 查找当前日期前缀下已存在的照片，从最大序号继续（更换日期则从00001开始）
        counter = 1
        if os.listdir(save_dir):
            try:
                # 筛选出当前日期前缀的jpg文件，提取序号
                existing_files = [
                    f for f in os.listdir(save_dir) 
                    if f.endswith('.jpg') and f.startswith(f"{date_prefix}_")
                ]
                if existing_files:
                    # 提取序号部分（如20260316_00005.jpg → 00005 → 5）
                    nums = [int(f.split('_')[1].split('.')[0]) for f in existing_files]
                    max_num = max(nums)
                    counter = max_num + 1
                print(f"📝 检测到{date_prefix}日期下已有{max_num}张照片，本次将从 {date_prefix}_{counter:05d}.jpg 开始保存")
            except:
                counter = 1  # 解析失败则从1开始
                print(f"📝 未检测到有效照片序号，本次将从 {date_prefix}_00001.jpg 开始保存")
        
        print("\n=== 摄像头已就绪 ===")
        print("⌨️  按 's' 键拍摄照片 | 按 'q' 键退出程序")
        print("-" * 50)
        
        # 记录本次拍摄的数量
        shoot_count = 0
        
        while True:
            ret, frame = cap.read()
            if not ret:
                print(f"❌ 错误：无法读取摄像头帧，可能摄像头已断开！")
                break
            
            # 显示实时画面
            cv2.imshow("Camera", frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                # 调整尺寸到 640x480（确保统一尺寸）
                resized_frame = cv2.resize(frame, (640, 480))
                # 生成带日期前缀的文件名（格式：日期_00001.jpg）
                filename = f"{date_prefix}_{counter:05d}.jpg"
                filepath = os.path.join(save_dir, filename)
                # 保存照片（添加写入权限检查）
                if cv2.imwrite(filepath, resized_frame):
                    # 获取文件大小（转换为KB）
                    file_size = osp.getsize(filepath) / 1024
                    shoot_count += 1
                    print(f"\n✅ 拍摄成功！")
                    print(f"📸 文件名：{filename}")
                    print(f"📂 保存路径：{filepath}")
                    print(f"📏 文件大小：{file_size:.2f} KB")
                    print(f"📊 本次已拍摄：{shoot_count} 张 | 累计序号：{counter:05d}")
                    print("-" * 50)
                    counter += 1
                else:
                    print(f"\n❌ 拍摄失败！")
                    print(f"🚫 无法保存照片：{filepath}")
                    print(f"💡 可能原因：权限不足 / 磁盘空间已满")
                    print("-" * 50)
            elif key == ord('q'):
                print("\n=== 退出程序 ===")
                print(f"📊 本次共拍摄成功 {shoot_count} 张照片")
                print(f"💾 所有照片已保存至：{save_dir}")
                print("👋 程序已安全退出！")
                break
        
        # 释放摄像头资源
        cap.release()
        cv2.destroyAllWindows()
    except Exception as e:
        print(f"\n❌ 程序运行出错：{e}")
        # 确保资源释放
        try:
            cap.release()
            cv2.destroyAllWindows()
        except:
            pass

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="鲁班猫摄像头拍照工具（本地保存版，无串口依赖）")
    # 默认路径改为鲁班猫本地目录，可通过参数自定义
    parser.add_argument("--save_dir", type=str, default="/home/bcsh/photos", 
                        help="照片保存目录（绝对路径，默认：/home/bcsh/photos）")
    # 新增日期前缀参数，可手动输入
    parser.add_argument("--date_prefix", type=str, default="20260316", 
                        help="照片命名的日期前缀（如20260316，默认：20260316）")
    args = parser.parse_args()
    
    take_photos(save_dir=args.save_dir, date_prefix=args.date_prefix)