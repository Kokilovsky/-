from sdk.api import UpAPI
import time

if __name__ == '__main__':
    # ========== 手动配置（查完硬件层后填这里） ==========
    # 先填你查到的理论量程（比如4095/1023），暂时填0，后面改
    THEORETICAL_MAX = 4095  # 假设是12位ADC，先填4095，实测后验证
    # ==================================================
    
    api = UpAPI(debug=True, grayscale_threshold=3060)
    # 记录运行中出现的最大实际值（接近理论量程）
    actual_max_record = 0  

    while True:
        try:
            grayscale_hardware = api._UpAPI__sensor.get_grayscale()
            raw_analog_data = grayscale_hardware.get_grayscale_data()
            
            # 1. 更新实际最大记录值
            if raw_analog_data:
                current_max = max(raw_analog_data)
                if current_max > actual_max_record:
                    actual_max_record = current_max
            
            # 2. 打印所有关键信息
            print("="*50)
            print(f"灰度模拟量原始值：{raw_analog_data}")
            print(f"当前最大值：{current_max if raw_analog_data else '无'}")
            print(f"运行至今实际最大输出值：{actual_max_record}")
            print(f"传感器理论量程上限：{THEORETICAL_MAX}（12位ADC）")
            print(f"阈值：{api._UpAPI__grayscale_threshold}")
            print(f"判定结果（True=黑/False=白）：{api.get_grayscale_data()}")
            print("="*50)

        except Exception as e:
            print(f"读取数据失败：{e}")

        time.sleep(0.5)