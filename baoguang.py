#!/usr/bin/env python3
"""
摄像头曝光度调节程序
支持通过 OpenCV 调节 USB 摄像头的曝光度参数
"""

import cv2
import sys

class CameraExposureControl:
    """摄像头曝光度控制类"""
    
    def __init__(self, camera_index=0):
        """
        初始化摄像头
        
        Args:
            camera_index: 摄像头索引，默认为0
        """
        self.camera_index = camera_index
        self.cap = None
        
    def open_camera(self):
        """打开摄像头"""
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            if not self.cap.isOpened():
                print(f"错误: 无法打开摄像头 {self.camera_index}")
                return False
            
            print(f"✓ 成功打开摄像头 {self.camera_index}")
            return True
        except Exception as e:
            print(f"错误: 打开摄像头失败 - {e}")
            return False
    
    def print_camera_properties(self):
        """打印当前摄像头参数"""
        if not self.cap or not self.cap.isOpened():
            print("错误: 摄像头未打开")
            return
        
        print("\n" + "="*50)
        print("当前摄像头参数:")
        print("="*50)
        
        properties = {
            "亮度 (BRIGHTNESS)": cv2.CAP_PROP_BRIGHTNESS,
            "对比度 (CONTRAST)": cv2.CAP_PROP_CONTRAST,
            "饱和度 (SATURATION)": cv2.CAP_PROP_SATURATION,
            "曝光度 (EXPOSURE)": cv2.CAP_PROP_EXPOSURE,
            "自动曝光 (AUTO_EXPOSURE)": cv2.CAP_PROP_AUTO_EXPOSURE,
            "增益 (GAIN)": cv2.CAP_PROP_GAIN,
            "白平衡温度 (WHITE_BALANCE_BLUE_U)": cv2.CAP_PROP_WHITE_BALANCE_BLUE_U,
            "白平衡温度 (WHITE_BALANCE_RED_V)": cv2.CAP_PROP_WHITE_BALANCE_RED_V,
        }
        
        for name, prop_id in properties.items():
            value = self.cap.get(prop_id)
            print(f"{name:30s}: {value:.2f}")
        
        print("="*50 + "\n")
    
    def set_exposure(self, exposure_value, auto_exposure=False):
        """
        设置曝光度
        
        Args:
            exposure_value: 曝光度值 (范围通常为 -12.0 到 -1.0，某些摄像头可能不同)
            auto_exposure: 是否自动曝光 (0=手动, 1=自动)
        """
        if not self.cap or not self.cap.isOpened():
            print("错误: 摄像头未打开")
            return False
        
        # 设置自动曝光模式
        # 注意: 不同摄像头的自动曝光参数可能不同
        # 0.75 (手动) 或 0.25 (自动) 是常见的值
        # 有些摄像头使用: 1=手动, 3=自动
        try:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25 if auto_exposure else 0.75)
            self.cap.set(cv2.CAP_PROP_EXPOSURE, exposure_value)
            print(f"✓ 曝光度设置为: {exposure_value:.2f} (自动曝光: {'开启' if auto_exposure else '关闭'})")
            return True
        except Exception as e:
            print(f"错误: 设置曝光度失败 - {e}")
            return False
    
    def set_brightness(self, brightness_value):
        """
        设置亮度
        
        Args:
            brightness_value: 亮度值 (范围通常为 0-100 或 -64-64)
        """
        if not self.cap or not self.cap.isOpened():
            print("错误: 摄像头未打开")
            return False
        
        try:
            self.cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness_value)
            print(f"✓ 亮度设置为: {brightness_value:.2f}")
            return True
        except Exception as e:
            print(f"错误: 设置亮度失败 - {e}")
            return False
    
    def set_contrast(self, contrast_value):
        """
        设置对比度
        
        Args:
            contrast_value: 对比度值 (范围通常为 0-100 或 0-1)
        """
        if not self.cap or not self.cap.isOpened():
            print("错误: 摄像头未打开")
            return False
        
        try:
            self.cap.set(cv2.CAP_PROP_CONTRAST, contrast_value)
            print(f"✓ 对比度设置为: {contrast_value:.2f}")
            return True
        except Exception as e:
            print(f"错误: 设置对比度失败 - {e}")
            return False
    
    def preview(self, window_name="摄像头预览"):
        """
        开启预览窗口并允许实时调节参数
        
        按键说明:
        - q: 退出
        - +: 增加曝光度
        - -: 减少曝光度
        - b: 增加亮度
        - n: 减少亮度
        - c: 增加对比度
        - d: 减少对比度
        - a: 切换自动曝光
        - p: 打印当前参数
        """
        if not self.cap or not self.cap.isOpened():
            print("错误: 摄像头未打开")
            return
        
        print("\n预览模式:")
        print("按键说明:")
        print("  q - 退出")
        print("  + - 增加曝光度")
        print("  - - 减少曝光度")
        print("  b - 增加亮度")
        print("  n - 减少亮度")
        print("  c - 增加对比度")
        print("  d - 减少对比度")
        print("  a - 切换自动曝光")
        print("  p - 打印当前参数")
        print("\n")
        
        auto_exp = False
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("错误: 无法读取摄像头画面")
                break
            
            # 在画面上显示当前参数
            cv2.putText(frame, f"Exposure: {self.cap.get(cv2.CAP_PROP_EXPOSURE):.2f}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Brightness: {self.cap.get(cv2.CAP_PROP_BRIGHTNESS):.2f}", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Contrast: {self.cap.get(cv2.CAP_PROP_CONTRAST):.2f}", 
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Auto Exp: {'ON' if auto_exp else 'OFF'}", 
                       (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.imshow(window_name, frame)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('+') or key == ord('='):
                # 增加曝光度
                current = self.cap.get(cv2.CAP_PROP_EXPOSURE)
                self.set_exposure(current + 0.5, auto_exp)
            elif key == ord('-'):
                # 减少曝光度
                current = self.cap.get(cv2.CAP_PROP_EXPOSURE)
                self.set_exposure(current - 0.5, auto_exp)
            elif key == ord('b'):
                # 增加亮度
                current = self.cap.get(cv2.CAP_PROP_BRIGHTNESS)
                self.set_brightness(current + 5)
            elif key == ord('n'):
                # 减少亮度
                current = self.cap.get(cv2.CAP_PROP_BRIGHTNESS)
                self.set_brightness(current - 5)
            elif key == ord('c'):
                # 增加对比度
                current = self.cap.get(cv2.CAP_PROP_CONTRAST)
                self.set_contrast(current + 5)
            elif key == ord('d'):
                # 减少对比度
                current = self.cap.get(cv2.CAP_PROP_CONTRAST)
                self.set_contrast(current - 5)
            elif key == ord('a'):
                # 切换自动曝光
                auto_exp = not auto_exp
                current = self.cap.get(cv2.CAP_PROP_EXPOSURE)
                self.set_exposure(current, auto_exp)
            elif key == ord('p'):
                # 打印当前参数
                self.print_camera_properties()
        
        cv2.destroyAllWindows()
    
    def close_camera(self):
        """关闭摄像头"""
        if self.cap:
            self.cap.release()
            print("✓ 摄像头已关闭")


def interactive_mode():
    """交互式命令行模式"""
    print("\n" + "="*50)
    print("摄像头曝光度调节程序 - 交互模式")
    print("="*50)
    
    # 选择摄像头
    print("\n请输入摄像头索引 (默认为0):")
    camera_input = input("> ").strip()
    camera_index = int(camera_input) if camera_input else 0
    
    camera = CameraExposureControl(camera_index)
    
    if not camera.open_camera():
        return
    
    camera.print_camera_properties()
    
    while True:
        print("\n请选择操作:")
        print("1. 设置曝光度")
        print("2. 设置亮度")
        print("3. 设置对比度")
        print("4. 切换自动曝光")
        print("5. 打印当前参数")
        print("6. 开启预览模式")
        print("7. 退出")
        
        choice = input("\n请输入选项 (1-7): ").strip()
        
        if choice == '1':
            value = input("请输入曝光度值 (推荐范围: -12.0 到 -1.0): ").strip()
            try:
                exposure = float(value)
                camera.set_exposure(exposure)
            except ValueError:
                print("错误: 请输入有效的数字")
        
        elif choice == '2':
            value = input("请输入亮度值 (推荐范围: 0-100): ").strip()
            try:
                brightness = float(value)
                camera.set_brightness(brightness)
            except ValueError:
                print("错误: 请输入有效的数字")
        
        elif choice == '3':
            value = input("请输入对比度值 (推荐范围: 0-100): ").strip()
            try:
                contrast = float(value)
                camera.set_contrast(contrast)
            except ValueError:
                print("错误: 请输入有效的数字")
        
        elif choice == '4':
            auto = input("启用自动曝光? (y/n): ").strip().lower()
            current = camera.cap.get(cv2.CAP_PROP_EXPOSURE)
            camera.set_exposure(current, auto == 'y')
        
        elif choice == '5':
            camera.print_camera_properties()
        
        elif choice == '6':
            camera.preview()
            break
        
        elif choice == '7':
            break
        
        else:
            print("错误: 无效选项")
    
    camera.close_camera()
    print("\n程序结束")


def main():
    """主函数"""
    if len(sys.argv) > 1 and sys.argv[1] == '--preview':
        # 直接进入预览模式
        camera_index = int(sys.argv[2]) if len(sys.argv) > 2 else 0
        camera = CameraExposureControl(camera_index)
        if camera.open_camera():
            camera.print_camera_properties()
            camera.preview()
            camera.close_camera()
    else:
        # 交互式模式
        interactive_mode()


if __name__ == "__main__":
    main()
