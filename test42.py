from sdk.api import UpAPI
from sdk.logic_layer.pid import PIDController
import time

class CustomLineFollower:
    def __init__(self, threshold=700):
        self.threshold = threshold  # 黑线检测阈值
    
    def process_frame(self, grayscale_data):
        """质心法计算偏移量"""
        if grayscale_data is None or len(grayscale_data) == 0:
            return None
        
        # 计算所有传感器的总和
        total = sum(grayscale_data)
        
        # 如果总和太小,说明没有检测到黑线
        if total < self.threshold:
            return None
        
        # 计算加权中心(质心)
        weighted_sum = sum(i * val for i, val in enumerate(grayscale_data))
        center_of_mass = weighted_sum / total
        
        # 计算相对于传感器中心的偏移
        sensor_center = len(grayscale_data) / 2
        offset = center_of_mass - sensor_center
        
        return offset


if __name__ == '__main__':
    # 参数设置
    move_speed = 16
    k_p = 20
    k_i = 0.01
    k_d = 100

    api = UpAPI()
    pid = PIDController(k_p=k_p, k_i=k_i, k_d=k_d, max_output=100)
    
    # 使用自定义巡线逻辑
    custom_follower = CustomLineFollower(threshold=100)

    try:
        while True:
            # 获取传感器数据
            grayscale_data = api.get_grayscale_data()
            
            # 使用自定义逻辑计算偏移量
            offset = custom_follower.process_frame(grayscale_data)
            
            if offset is not None:
                # PID 计算
                turn_rate = pid.compute(offset)
                print(f"Offset: {offset:>7.2f} | TurnRate: {turn_rate:>6d}")
                
                # 发送运动指令
                api.move_rotation(speed=move_speed, turn_rate=turn_rate)
            else:
                print("未检测到黑线,直行")
                api.move_forward(speed=move_speed)
            
            time.sleep(0.1)

    except KeyboardInterrupt:
        api.stop()
