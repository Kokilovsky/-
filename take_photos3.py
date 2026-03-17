import cv2
import os
import sys
import argparse
import shutil
import os.path as osp

def adjust_exposure_and_capture(save_dir="/home/bcsh/photos", date_prefix="20260316"):
    """
    交互式调节摄像头曝光度，支持实时预览曝光效果，调节完成后可拍摄保存照片
    操作说明：
    - 按 '+'/']' 键：增加曝光度
    - 按 '-'/']' 键：降低曝光度
    - 按 's' 键：拍摄并保存照片
    - 按 'q' 键：退出程序
    """
    # 初始化保存目录
    try:
        os.makedirs(save_dir, exist_ok=True)
        print(f"\n=== 初始化完成 ===")
        print(f"📁 照片保存目录: {save_dir}")
        print(f"📅 当前日期前缀: {date_prefix}")
    except Exception as e:
        print(f"❌ 错误：无法创建保存目录 {save_dir} - {e}")
        return

    # 检查磁盘空间
    try:
        disk_usage = shutil.disk_usage(save_dir)
        free_space_gb = disk_usage.free / (1024**3)
        if free_space_gb < 1:
            print(f"⚠️  警告：可用空间不足1GB ({free_space_gb:.2f}GB)！")
        else:
            print(f"✅ 可用空间：{free_space_gb:.2f}GB（满足要求）")
    except Exception as e:
        print(f"⚠️  空间检查失败：{e}")

    # 打开摄像头
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print(f"❌ 错误：无法打开摄像头！")
        return

    # 获取摄像头默认曝光度（不同摄像头取值范围不同：通常-10~10 或 0~100）
    default_exposure = cap.get(cv2.CAP_PROP_EXPOSURE)
    current_exposure = default_exposure
    print(f"\n=== 曝光度调节模式 ===")
    print(f"🎚️ 当前曝光度：{current_exposure:.1f}（默认值：{default_exposure:.1f}）")
    print("⌨️ 操作说明：")
    print("   + / ] 键 → 增加曝光度 | - / [ 键 → 降低曝光度")
    print("   s 键 → 拍摄照片 | q 键 → 退出程序")
    print("-" * 60)

    # 初始化照片计数器
    counter = 1
    if os.listdir(save_dir):
        try:
            existing_files = [f for f in os.listdir(save_dir) if f.endswith('.jpg') and f.startswith(f"{date_prefix}_")]
            if existing_files:
                nums = [int(f.split('_')[1].split('.')[0]) for f in existing_files]
                counter = max(nums) + 1
        except:
            counter = 1
    shoot_count = 0

    # 曝光度调节步长（可根据实际效果调整）
    exposure_step = 0.5

    while True:
        ret, frame = cap.read()
        if not ret:
            print(f"❌ 错误：无法读取摄像头画面！")
            break

        # 显示当前曝光度信息在画面上
        cv2.putText(
            frame, 
            f"Exposure: {current_exposure:.1f} (± to adjust)", 
            (10, 30), 
            cv2.FONT_HERSHEY_SIMPLEX, 
            0.7, 
            (0, 255, 0), 
            2
        )
        cv2.putText(
            frame, 
            f"Shoot: {shoot_count} | Next: {date_prefix}_{counter:05d}.jpg", 
            (10, 60), 
            cv2.FONT_HERSHEY_SIMPLEX, 
            0.7, 
            (255, 0, 0), 
            2
        )

        # 显示实时画面
        cv2.imshow("Camera (Adjust Exposure + Shoot)", frame)

        # 按键检测
        key = cv2.waitKey(1) & 0xFF

        # 调节曝光度
        if key in [ord('+'), ord(']'), ord('=')]:  # 增加曝光度（适配不同键盘）
            current_exposure += exposure_step
            cap.set(cv2.CAP_PROP_EXPOSURE, current_exposure)
            print(f"🎚️ 曝光度已增加 → {current_exposure:.1f}")
        
        elif key in [ord('-'), ord('[')]:  # 降低曝光度
            current_exposure -= exposure_step
            cap.set(cv2.CAP_PROP_EXPOSURE, current_exposure)
            print(f"🎚️ 曝光度已降低 → {current_exposure:.1f}")
        
        # 拍摄保存照片
        elif key == ord('s'):
            resized_frame = cv2.resize(frame, (640, 480))
            filename = f"{date_prefix}_{counter:05d}.jpg"
            filepath = os.path.join(save_dir, filename)
            
            if cv2.imwrite(filepath, resized_frame):
                file_size = osp.getsize(filepath) / 1024
                shoot_count += 1
                print(f"\n✅ 拍摄成功！")
                print(f"📸 文件名：{filename} | 曝光度：{current_exposure:.1f}")
                print(f"📂 路径：{filepath} | 大小：{file_size:.2f} KB")
                print(f"📊 本次拍摄：{shoot_count} 张")
                print("-" * 60)
                counter += 1
            else:
                print(f"\n❌ 保存失败：{filepath}")
                print("-" * 60)
        
        # 退出程序
        elif key == ord('q'):
            print(f"\n=== 退出程序 ===")
            print(f"📊 本次共拍摄：{shoot_count} 张照片")
            print(f"💾 保存路径：{save_dir}")
            print(f"🎚️ 最后曝光度：{current_exposure:.1f}")
            print("👋 程序已退出！")
            break

    # 释放资源
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="鲁班猫摄像头曝光度调节+拍照工具")
    parser.add_argument("--save_dir", type=str, default="/home/bcsh/photos",
                        help="照片保存目录（默认：/home/bcsh/photos）")
    parser.add_argument("--date_prefix", type=str, default="20260316",
                        help="照片日期前缀（如20260316）")
    args = parser.parse_args()
    
    adjust_exposure_and_capture(save_dir=args.save_dir, date_prefix=args.date_prefix)