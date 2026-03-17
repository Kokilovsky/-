import cv2
import os

# 尝试不同的摄像头索引
for i in range(5):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"成功打开摄像头索引 {i}")
        break
else:
    # 尝试使用设备路径
    print("尝试使用设备路径...")
    possible_paths = ["/dev/video0", "/dev/video1", "/dev/video2"]
    for path in possible_paths:
        if os.path.exists(path):
            cap = cv2.VideoCapture(path)
            if cap.isOpened():
                print(f"成功打开摄像头路径 {path}")
                break
    else:
        print("无法打开摄像头")
        print("检查摄像头是否正确连接")
        print("检查当前用户是否有访问摄像头的权限")
        print("运行 'ls -la /dev/video*' 查看可用的摄像头设备")
        exit()

# 获取初始曝光度值
initial_exposure = cap.get(cv2.CAP_PROP_EXPOSURE)
print(f"初始曝光度: {initial_exposure}")

# 创建窗口
cv2.namedWindow("摄像头曝光度测试", cv2.WINDOW_NORMAL)

print("操作说明:")
print("- 上箭头: 增加曝光度")
print("- 下箭头: 减少曝光度")
print("- r键: 恢复初始曝光度")
print("- q键: 退出程序")

while True:
    # 读取帧
    ret, frame = cap.read()
    if not ret:
        print("无法获取帧")
        break
    
    # 获取当前曝光度值
    current_exposure = cap.get(cv2.CAP_PROP_EXPOSURE)
    
    # 在图像上显示当前曝光度
    cv2.putText(frame, f"当前曝光度: {current_exposure:.2f}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    # 显示操作说明
    cv2.putText(frame, "上箭头: 增加曝光", (10, 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)
    cv2.putText(frame, "下箭头: 减少曝光", (10, 80), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)
    cv2.putText(frame, "r键: 恢复初始值", (10, 100), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)
    cv2.putText(frame, "q键: 退出", (10, 120), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)
    
    # 显示图像
    cv2.imshow("摄像头曝光度测试", frame)
    
    # 等待按键
    key = cv2.waitKey(1) & 0xFF
    
    # 按键处理
    if key == ord('q'):
        break
    elif key == ord('r'):
        # 恢复初始曝光度
        cap.set(cv2.CAP_PROP_EXPOSURE, initial_exposure)
        print(f"已恢复初始曝光度: {initial_exposure}")
    elif key == 82:  # 上箭头
        # 增加曝光度
        current_exposure += 1.0
        cap.set(cv2.CAP_PROP_EXPOSURE, current_exposure)
        print(f"增加曝光度至: {current_exposure}")
    elif key == 84:  # 下箭头
        # 减少曝光度
        current_exposure -= 1.0
        cap.set(cv2.CAP_PROP_EXPOSURE, current_exposure)
        print(f"减少曝光度至: {current_exposure}")

# 释放资源
cap.release()
cv2.destroyAllWindows()
print("程序已退出")