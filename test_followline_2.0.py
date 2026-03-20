import time
from sdk.api import UpAPI
from sdk.logic_layer.pid import PIDController


if __name__ == '__main__':
    # 参数设置
    move_speed = 60

    k_p = 10.5
    k_i = 0.02
    k_d = 40 #initial

    # 传感器 逻辑处理器 执行器
    api = UpAPI()
    pid = PIDController(k_p=k_p, k_i=k_i, k_d=k_d)

    last_turn_rate = 0  # 初始化上一次的旋转速率
    start_time = None  # 初始化开始时间
    elapsed_time = 0  # 初始化经过时间
    declining_count = 0 #初始化特别纠偏机制参数下降量
    #####
    declining_limit = 1 #初始化特别纠偏机制参数下降量极限
    declining_rate = 0.05 #初始化特别纠偏机制参数下降量变化速度
    #####
    stable_count = 0 #初始化稳定计数
    #####
    straight_coefficient = 1.2 #直线状况下特别纠偏系数
    turing_coefficient = 1.8 #曲线状况下特别纠偏系数
    #####
    k_d_change_time = 0.2 #开始时k_d变化时间
    long_term_k_d = 90.5 #长期k_d值
    end_time = 5.7 #结束时间
    #####
    time_checkpoint1 = 1 #第一个检查点时间
    time_checkpoint2 = 3.7 #第二个检查点时间
    #####
    stable_limit = 5 #稳定计数极限
    stable_declining_count = 0.5 #稳定计数下特别纠偏机制参数下降量

    #last_error_change = 0.0  # 初始化上一次的误差变化率

    while True:
        # 检查是否需要停止
        if elapsed_time >= end_time:
            api.stop()
            print("机器已停止")
            break  # 退出循环

        if elapsed_time >= k_d_change_time:
            k_d = long_term_k_d
            pid.k_d = k_d  # 更新PID控制器的k_d值
        
        # 获取数据
        grayscale_data = api.get_grayscale_data()

        # 计算偏移量
        offset,length = api.follow_line()

        # 临时测试：强制offset为None来测试else分支
        # offset = None  # 取消注释此行来测试else分支

        print("=============")
        print(f"grayscale_data: {grayscale_data}")
        print(f"offset: {offset}")

        # 发送移动指令
        if offset == 0 and length == 0:
            stable_count = 0
            if elapsed_time >= time_checkpoint2 or elapsed_time <= time_checkpoint1:
               turn_rate = last_turn_rate * straight_coefficient
            else:
                turn_rate = last_turn_rate * turing_coefficient
            print(f"turn_rate: {last_turn_rate} (using last)")
            print(f"elapsed_time: {elapsed_time}")
            # 记录第一个前进指令的开始时间
            if start_time is None:
                start_time = time.time()
            if declining_count <= declining_limit:
                declining_count += declining_rate
            api.move_rotation(speed=move_speed, turn_rate=turn_rate) 
            turn_rate = turn_rate - last_turn_rate * declining_count
            elapsed_time = time.time() - start_time # 使用上一次的旋转速率

        else:
            turn_rate = pid.compute(offset)
            #last_error_change = abs(pid.get_error_change())
            print(f"turn_rate: {turn_rate}")
            # 记录第一个前进指令的开始时间
            if start_time is None:
                start_time = time.time()
            api.move_rotation(speed=move_speed, turn_rate=turn_rate)
            last_turn_rate = turn_rate  # 更新上一次的旋转速率
            elapsed_time = time.time() - start_time
            print(f"elapsed_time: {elapsed_time}")
            stable_count += 1
            if stable_count >= stable_limit:
                declining_count = stable_declining_count
                stable_count = 0
            else:
                declining_count = 0
        
