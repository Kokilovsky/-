import cv2
import os
import sys
import argparse

# 添加SDK路径（假设脚本在app目录下）
sys.path.append(os.path.join(os.path.dirname(__file__), 'sdk'))

from sdk.api import UpAPI

def take_photos(save_dir="/media/bcsh/6268-9AE5"):
    """
    打开摄像头，按 's' 键拍摄照片并保存为 640x480 尺寸，名称从 00001.jpg 开始顺序递增。
    按 'q' 键退出。
    """
    # 检查U盘是否挂载
    if not os.path.exists(save_dir):
        print(f"错误：保存目录 {save_dir} 不存在。请确保U盘已插入并挂载。")
        return
    
    # 检查可用空间（至少1GB）
    try:
        stat = os.statvfs(save_dir)
        free_space_gb = stat.f_bavail * stat.f_frsize / (1024**3)
        if free_space_gb < 1:
            print(f"警告：{save_dir} 可用空间不足1GB ({free_space_gb:.2f}GB)。")
            return
    except Exception as e:
        print(f"检查空间时出错: {e}")
        return
    
    try:
        api = UpAPI()
        cap = api.get_camera()
        if not cap or not cap.isOpened():
            print("错误：无法打开摄像头。请检查鲁班猫摄像头连接和SDK。")
            return
        
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        
        counter = 1
        print(f"摄像头已打开。照片将保存到: {save_dir}")
        print("按 's' 键拍摄照片，按 'q' 键退出。")
        
        while True:
            ret, frame = cap.read()
            if not ret:
                print("错误：无法读取摄像头帧。")
                break
            
            # 显示实时画面
            cv2.imshow("Camera", frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                # 调整尺寸到 640x480
                resized_frame = cv2.resize(frame, (640, 480))
                # 生成文件名
                filename = f"{counter:05d}.jpg"
                filepath = os.path.join(save_dir, filename)
                if cv2.imwrite(filepath, resized_frame):
                    print(f"照片已保存: {filepath}")
                    counter += 1
                else:
                    print(f"错误：保存照片失败: {filepath}")
            elif key == ord('q'):
                break
        
        cap.release()
        cv2.destroyAllWindows()
    except Exception as e:
        print(f"发生错误: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="鲁班猫摄像头拍照工具")
    parser.add_argument("--save_dir", type=str, default="/media/bcsh/6268-9AE5", help="照片保存目录（绝对路径）")
    args = parser.parse_args()
    
    take_photos(save_dir=args.save_dir)
