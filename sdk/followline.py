import time
from sdk.api import UpAPI
from sdk.logic_layer.pid import PIDController

class FollowLine:
    def __init__(self, move_speed=60, k_p=10.5, k_i=0.02, k_d=40):
        self.move_speed = move_speed
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.pid = PIDController(k_p=k_p, k_i=k_i, k_d=k_d)
        self.api = UpAPI()
        self.last_turn_rate = 0
        self.start_time = None
        self.elapsed_time = 0
        self.declining_count = 0
        self.declining_limit = 1
        self.declining_rate = 0.05
        self.stable_count = 0
        self.stable_limit = 5
        self.stable_declining_count = 0.5
        self.straight_coefficient = 1.2 #直线状况下特别纠偏系数
        self.turing_coefficient = 1.8 #曲线状况下特别纠偏系数
        #####
        self.k_d_change_time = 0.2 #开始时k_d变化时间
        self.long_term_k_d = 90.5 #长期k_d值
        self.end_time = 5.7 #结束时间
        #####
        self.time_checkpoint1 = 1 #第一个检查点时间
        self.time_checkpoint2 = 3.7 #第二个检查点时间

    def follow_line(self):

     #initial

    # 传感器 逻辑处理器 执行器

    
    #last_error_change = 0.0  # 初始化上一次的误差变化率

        while True:
            # 检查是否需要停止
            if self.elapsed_time >= self.end_time:
                self.api.stop()
                print("机器已停止")
                break  # 退出循环

            if self.elapsed_time >= self.k_d_change_time:
                k_d = self.long_term_k_d
                self.pid.k_d = k_d  # 更新PID控制器的k_d值
        
        # 获取数据
            grayscale_data = self.api.get_grayscale_data()

        # 计算偏移量
            offset,length = self.api.follow_line()

        # 临时测试：强制offset为None来测试else分支
        # offset = None  # 取消注释此行来测试else分支

            print("=============")
            print(f"grayscale_data: {grayscale_data}")
            print(f"offset: {offset}")

        # 发送移动指令
            if offset == 0 and length == 0:
                stable_count = 0
                if self.elapsed_time >= self.time_checkpoint2 or self.elapsed_time <= self.time_checkpoint1:
                    turn_rate = self.last_turn_rate * self.straight_coefficient
                else:
                    turn_rate = self.last_turn_rate * self.turing_coefficient
                print(f"turn_rate: {self.last_turn_rate} (using last)")
                print(f"elapsed_time: {self.elapsed_time}")
            # 记录第一个前进指令的开始时间
                if self.start_time is None:
                    self.start_time = time.time()
                if self.declining_count <= self.declining_limit:
                    self.declining_count += self.declining_rate
                self.api.move_rotation(speed=self.move_speed, turn_rate=turn_rate)
                turn_rate = turn_rate - self.last_turn_rate * self.declining_count
                self.elapsed_time = time.time() - self.start_time  # 使用上一次的旋转速率

            else:
                turn_rate = self.pid.compute(offset)
                #last_error_change = abs(self.pid.get_error_change())
                print(f"turn_rate: {turn_rate}")
                # 记录第一个前进指令的开始时间
                if self.start_time is None:
                    self.start_time = time.time()
                self.api.move_rotation(speed=self.move_speed, turn_rate=turn_rate)
                self.last_turn_rate = turn_rate  # 更新上一次的旋转速率
                self.elapsed_time = time.time() - self.start_time
                print(f"elapsed_time: {self.elapsed_time}")
                self.stable_count += 1
                if self.stable_count >= self.stable_limit:
                    self.declining_count = self.stable_declining_count
                    self.stable_count = 0
                else:
                    self.declining_count = 0
        
