# 机器人视觉竞技 A

from sdk.data_layer.arm import arm_action_factory as arm_action
from sdk.api import UpAPI
from sdk.model import YoloModel
from sdk.logic_layer.cross_planner import CrossLocator
from sdk.logic_layer.pid import PIDController
from sdk.logic_layer.time_meter import TimeMeter
from sdk.followline import FollowLine
from enum import Enum, auto
import numpy as np
import time
import math
import threading


class MainState(Enum):
    """
    一级状态机：目标状态
    """
    IDLE = auto()
    APRIL_TAG = auto()
    GESTURE = auto()
    YOLO = auto()
    FACE = auto()
    HOME = auto()
    FINISH = auto()

class Controller:
    def __init__(self):
        # -------- 参数/时间配置 --------
        # 速度参数
        self.grayscale_threshold = 700
        self.speed_move_main = 60          # 主行进目标速度
        self.speed_hit_position = 25
        self.speed_spin = 300
        self.speed_locate_move = 6
        self.speed_locate_spin = 30
        self.speed_all_false = 12
        # 加速（旧模式保留）
        self.ramp_time_main = 150
        self.ramp_min_start = 5
        self.ramp_mode = 'ease_in_out'
        self.ramp_exp = 2.0
        # 分段速度曲线配置
        self.accel_time_ms = 100
        self.decel_time_ms = 100

        # 各阶段用时（ms）
        self.time_init = 1000
        self.time_left_turn = 568
        self.time_right_turn = 583
        self.time_back_turn_1 = 1190
        self.time_back_turn_2 = 1190
        self.time_april = 500
        self.time_gesture = 500
        self.time_hit = 500
        self.time_yolo = 700
        self.time_face = 700
        self.time_pre_yolo = 500
        self.time_pre_face = 400
        self.time_move2april = 1100
        self.time_move2gesture = 1570
        self.time_move2yolo = 1000
        self.time_move2face = 1020
        self.time_move2home = 2000

        # -------- 识别/目标参数 --------
        self.target_yolo = "tank"
        self.target_face = "bad_person"
        self.target_id = 1
        self.target_number_right = 3
        self.target_number_both = 5

        self.gesture_type = 0
        self.yolo_location = 0
        self.face_location = 0
        self.gesture_type_judge = np.array([0, 0])
        self.yolo_location_judge = np.array([0, 0])
        self.face_location_judge = np.array([0, 0])
        # 判定阈值
        self.gesture_type_max = 5
        self.yolo_location_max = 3
        self.face_location_max = 5
        self.locator_end_correct = 0

        self.yolo_model = YoloModel.VEHICLE

        # 手臂动作
        self.left_arm_actions = {
            "clamp": arm_action.left_arm_clamp(),
            "up": arm_action.left_arm_raise()
        }
        self.right_arm_actions = {
            "clamp": arm_action.right_arm_clamp(),
            "up": arm_action.right_arm_raise(),
            "pre_hit": arm_action.right_arm_prepare_beat(),
            "hit": arm_action.right_arm_beat()
        }

        # 状态
        self.state_main = MainState.IDLE
        self.start = 1
        self.stage = 0

        # 子系统
        self.api = UpAPI(yolo_model=self.yolo_model, grayscale_threshold=self.grayscale_threshold)
        self.locator = CrossLocator()

        # 计时器
        self.spanner_init = TimeMeter(self.time_init)
        self.spanner_left_turn = TimeMeter(self.time_left_turn)
        self.spanner_right_turn = TimeMeter(self.time_right_turn)
        self.spanner_back_turn_1 = TimeMeter(self.time_back_turn_1)
        self.spanner_back_turn_2 = TimeMeter(self.time_back_turn_2)
        self.spanner_move2april = TimeMeter(self.time_move2april)
        self.spanner_move2gesture = TimeMeter(self.time_move2gesture)
        self.spanner_move2yolo = TimeMeter(self.time_move2yolo)
        self.spanner_move2face = TimeMeter(self.time_move2face)
        self.spanner_move2home = TimeMeter(self.time_move2home)

        # 其它
        self.count_stable = 0
        self.count_continuous_stable = 15
        self.recording_out = self.api.recording_prepare()

        # 校正特殊判断
        self.locator_mode_error = 0

        # 尾段校正相关参数
        self.count_cross_pass = 0
        self.cross_state = 0
        self.locator_end_correct = 0
        self.locator_end_size = 0

        # 并行识别线程
        self._stop_event = threading.Event()
        self._vision_thread = threading.Thread(target=self._vision_loop, daemon=True)
        # 加速起步时间记录
        self._ramp_start_times = {}
        self._phase_start_times = {}

    def __phase_speed(self, key, target_speed, total_ms):
        """分段速度曲线:
        0~accel_time_ms 线性(可换函数)加速到 target_speed;
        中段保持 target_speed;
        最后 decel_time_ms 线性减速到 0.
        若 total_ms < accel+decel 则按比例压缩：加速到峰值后立刻进入减速。
        """
        now = self.__get_nowtime()
        if key not in self._phase_start_times:
            self._phase_start_times[key] = now
        elapsed = now - self._phase_start_times[key]
        if total_ms <= 0:
            return 0
        accel = min(self.accel_time_ms, total_ms/2)
        decel = min(self.decel_time_ms, total_ms/2)
        cruise_end = total_ms - decel
        if elapsed >= total_ms:
            return 0
        if elapsed < accel:  # 加速段 0 -> target_speed
            ratio = elapsed / accel
            return int(target_speed * ratio)
        if elapsed >= cruise_end:  # 减速段
            remain = total_ms - elapsed
            ratio = remain / decel  # 0~1
            return int(max(0, target_speed * ratio))
        return int(target_speed)

    # 保留旧 __ramp_speed 名称可兼容，但内部直接调用分段（若未来还需原曲线，可另存）
    def __ramp_speed(self, key, target_speed, ramp_time_ms):
        return self.__phase_speed(key, target_speed, ramp_time_ms)

    def run(self):
        # 启动视觉线程（边运动边识别）
        if not self._vision_thread.is_alive():
            self._vision_thread.start()

        while True:
            if self.state_main == MainState.IDLE:
                if not self.spanner_init.in_progress:
                    self.spanner_init.start()

                if self.spanner_init.complete():
                    print("初始化完成")
                    self.state_main = MainState.APRIL_TAG
                    break
                    
                else:
                    print("初始化中...")
                    self.__clamp_arms()

        while self.start:
            name = input("输入1开始运行")
            if name == '1':
                self.start_time = self.__get_nowtime()
                self.start = 0
                break

        while True:
            # 传感器数据
            grayscale_data = self.api.get_grayscale_data()
            # print(grayscale_data)  # 打印灰度值

            # 主循环
            if self.state_main == MainState.APRIL_TAG:
                if self.stage == 0:
                    # 前往AprilTag
                    if not self.spanner_move2april.in_progress:
                        self.spanner_move2april.start()

                    if self.spanner_move2april.complete():
                        print("已到达AprilTag")
                        self.api.stop()
                        self.stage += 1
                    else:
                        speed = self.__phase_speed("april_move", self.speed_move_main, self.time_move2april)
                        self.api.move_translation(angle=360-50, speed=speed)

                elif self.stage == 1:
                    # 举手
                    print("执行举手动作")
                    self.__do_arm_action(1)
                    time.sleep(self.time_april/1000)

                    self.__clamp_arms()
                    self.stage += 1

                elif self.stage == 2:
                    # 转身
                    if not self.spanner_back_turn_1.in_progress:
                        self.spanner_back_turn_1.start()
                    
                    if self.spanner_back_turn_1.complete():
                        print("转身完毕")
                        self.api.stop()
                        time.sleep(0.03)
                        self.stage = 0
                        self.state_main = MainState.GESTURE
                        # time.sleep(0.05)
                    else:
                        self.api.spin_left(self.speed_spin)

            elif self.state_main == MainState.GESTURE:
                if self.stage == 0:
                    # 前往手势区域，同时后台线程已在识别
                    if not self.spanner_move2gesture.in_progress:
                        self.spanner_move2gesture.start()
                        self._phase_start_times['gesture_move'] = self.__get_nowtime()
                        self.start_time_gesture = self.__get_nowtime()
                    if self.spanner_move2gesture.complete():
                        print("到达手势区域，准备执行动作")
                        self.api.stop()
                        self.stage = 1
                    else:
                        speed = self.__phase_speed("gesture_move", self.speed_move_main, self.time_move2gesture)
                        self.api.move_forward(speed)
                elif self.stage == 1:
                    # 等待（少量）或直接执行
                    if self.gesture_type == 0:
                        # 给识别一点补偿时间（最多 1 秒）
                        if (self.__get_nowtime() - self.start_time_gesture) > 1000:
                            print("未识别到手势，采用默认右手")
                            self.gesture_type = self.target_number_right
                        else:
                            time.sleep(0.03)
                            continue
                    print(f"执行手势动作 {self.gesture_type}")
                    self.__do_arm_action(self.gesture_type)
                    time.sleep(self.time_gesture/1000)
                    self.__clamp_arms()
                    self.stage = 2
                elif self.stage == 2:
                    if not self.spanner_back_turn_2.in_progress:
                        self.spanner_back_turn_2.start()
                    if self.spanner_back_turn_2.complete():
                        print("手势段转身完成")
                        self.api.stop()
                        time.sleep(0.1)
                        self.stage = 0
                        self.state_main = MainState.YOLO
                    else:
                        self.api.spin_left(self.speed_spin)

            elif self.state_main == MainState.YOLO:
                if self.stage == 0:
                    if not self.spanner_move2yolo.in_progress:
                        self.spanner_move2yolo.start()
                        self._phase_start_times['yolo_move'] = self.__get_nowtime()
                        self.start_time_yolo = self.__get_nowtime()
                    if self.spanner_move2yolo.complete():
                        self.api.stop()
                        self.__crossroad_location(1)
                        print("到达YOLO区域，准备击打")
                        self.api.stop()
                        self.stage = 1
                    else:
                        speed = self.__phase_speed("yolo_move", self.speed_move_main, self.time_move2yolo)
                        self.api.move_translation(angle=-55, speed=speed)
                elif self.stage == 1:
                    if self.yolo_location == 0:
                        if (self.__get_nowtime() - self.start_time_yolo) > 1200:
                            print("YOLO未识别到，放弃击打")
                            self.stage = 3  # 跳到转身
                        else:
                            time.sleep(0.03)
                            continue
                    angle = 60
                    if self.yolo_location == 2:
                        angle = -10
                        self.time_pre_yolo -= 50
                    self.__pre_hit()
                    self.api.move_translation(angle=angle,speed=self.speed_hit_position,run_time=self.time_pre_yolo)
                    time.sleep(self.time_pre_yolo/1000)
                    self.__hit()
                    time.sleep(self.time_hit/1000)
                    self.__do_arm_action(3)
                    if self.yolo_location == 1:
                        angle -= 10
                    self.api.move_translation(angle=180+angle,speed=self.speed_hit_position,run_time=self.time_pre_yolo)
                    time.sleep(self.time_pre_yolo/1000)
                    self.__clamp_arms()
                    print("击打tank完成")
                    self.stage = 2
                elif self.stage == 2:
                    if not self.spanner_right_turn.in_progress:
                        self.spanner_right_turn.start()
                    if self.spanner_right_turn.complete():
                        print("YOLO段转身完成")
                        self.api.stop()
                        self.stage = 0
                        self.state_main = MainState.FACE
                        # self.api.move_backward(speed=20,run_time=300)
                        # time.sleep(0.3)
                        # self.api.stop()
                        # self.__crossroad_location(1)
                    else:
                        self.api.spin_right(self.speed_spin)

            elif self.state_main == MainState.FACE:
                if self.stage == 0:
                    if not self.spanner_move2face.in_progress:
                        self.spanner_move2face.start()
                        self._phase_start_times['face_move'] = self.__get_nowtime()
                        self.start_time_face = self.__get_nowtime()
                    if self.spanner_move2face.complete():
                        print("到达人脸区域，准备击打")
                        self.api.stop()
                        self.stage = 1
                    else:
                        speed = self.__phase_speed("face_move", self.speed_move_main, self.time_move2face)
                        self.api.move_forward(speed=speed)
                elif self.stage == 1:
                    if self.face_location == 0:
                        if (self.__get_nowtime() - self.start_time_face) > 1200:
                            print("人脸未识别到，跳过击打")
                            self.stage = 2
                        else:
                            time.sleep(0.03)
                            continue
                    angle = 55
                    if self.face_location == 2:
                        angle = -5
                    self.__pre_hit()
                    self.api.move_translation(angle=angle,speed=self.speed_hit_position,run_time=self.time_pre_yolo)
                    time.sleep(self.time_pre_yolo/1000)
                    self.__hit()
                    time.sleep(self.time_hit/1000)
                    self.__do_arm_action(3)
                    self.api.move_translation(angle=180+angle,speed=self.speed_hit_position,run_time=self.time_pre_yolo)
                    time.sleep(self.time_pre_yolo/1000)
                    self.__clamp_arms()
                    print("击打人脸完成")
                    self.stage = 2
                elif self.stage == 2:
                    self.stage = 0
                    self.api.stop()
                    self.state_main = MainState.HOME

            elif self.state_main == MainState.HOME:
                if self.stage == 0:
                    if not self.spanner_move2home.in_progress:
                        self.spanner_move2home.start()
                        self._phase_start_times['home_move'] = self.__get_nowtime()

                    if self.spanner_move2home.complete():
                        print("已到达终点区域")
                        self.api.stop()
                        self.stage += 1
                    else:
                        speed = self.__phase_speed("home_move", self.speed_move_main, self.time_move2home)
                        self.api.turn_right()
                        self.api.move_forward(speed=speed)

                        if self.locator.leave_cross(grayscale_data):
                            print("出发")
                        elif self.locator.detect_black(grayscale_data):
                            self.api.stop()
                                
                elif self.stage == 1:
                    self.stage = 0
                    self.state_main = MainState.FINISH
                    self.follow_line = FollowLine()
                    self.follow_line.follow_line()

            elif self.state_main == MainState.FINISH:
                if self.locator_end_correct:
                    move_time = 130
                    if self.locator_end_size == 2:
                        move_time = 200
                    if self.locator_end_correct == 1:
                        print("执行向右微调")
                        self.api.move_right(speed=30,run_time=move_time)
                        time.sleep(move_time / 1000)
                    else:
                        print("执行向左微调")
                        self.api.move_left(speed=30,run_time=move_time)
                        time.sleep(move_time / 1000)

                print("任务完成")
                self.api.stop()
                print(f"一共耗时{(self.__get_nowtime()-self.start_time)/1000}s")
                self._stop_event.set()
                break

    def __do_arm_action(self, number):
        print(f"做手势动作{number}")
        if number == self.target_id:
            # 举左手
            left_action = self.left_arm_actions["up"]
            right_action = self.right_arm_actions["clamp"]
            self.api.execute_arm_action(left_action, right_action)

        elif number == self.target_number_right:
            # 举右手
            left_action = self.left_arm_actions["clamp"]
            right_action = self.right_arm_actions["up"]
            self.api.execute_arm_action(left_action, right_action)

        elif number == self.target_number_both:
            # 举双手
            left_action = self.left_arm_actions["up"]
            right_action = self.right_arm_actions["up"]
            self.api.execute_arm_action(left_action, right_action)

    def __clamp_arms(self):
        left_action = self.left_arm_actions["clamp"]
        right_action = self.right_arm_actions["clamp"]
        self.api.execute_arm_action(left_action, right_action)

    def __pre_hit(self):
        left_action = self.left_arm_actions["clamp"]
        right_action = self.right_arm_actions["pre_hit"]
        self.api.execute_arm_action(left_action, right_action)

    def __hit(self):
        left_action = self.left_arm_actions["clamp"]
        right_action = self.right_arm_actions["hit"]
        self.api.execute_arm_action(left_action, right_action)

    def __reset_yolo(self):
        self.api.close_yolo_window()
        self.api.reset_yolo_pool()
        # 录像清理
        self.api.reset_recording()

    def __get_nowtime(self):
        return int(time.time()*1000)
    
    def __gesture_judge(self):
        while not self.gesture_type:
            find_gesture, number = self.api.detect_gesture()
            gesture_time = (self.__get_nowtime()-self.start_time_gesture)/1000
            print(f"寻找手势结果为{find_gesture},已耗时{gesture_time}s")
            if number == 3:
                self.gesture_type_judge[0] += 1
            elif number == 5:
                self.gesture_type_judge[1] += 1
            if self.gesture_type_judge[0] > self.gesture_type_max:
                self.gesture_type = 3
            elif self.gesture_type_judge[1] > self.gesture_type_max:
                self.gesture_type = 5

    def __yolo_judge(self):
        while not self.yolo_location:
            find_target, offset, offset_y, width = self.api.sustain_detect_yolo(self.recording_out)
            yolo_time = (self.__get_nowtime()-self.start_time_yolo)/1000
            print(f"tank靠左计数为{self.yolo_location_judge[0]},tank靠右计数为{self.yolo_location_judge[1]},已耗时{yolo_time}")
            if find_target and offset is not None and offset_y is not None:
                if offset < offset_y:
                    self.yolo_location_judge[0] += 1
                elif offset > offset_y:
                    self.yolo_location_judge[1] += 1
                if self.yolo_location_judge[0] > self.yolo_location_max:
                    self.yolo_location = 1
                elif self.yolo_location_judge[1] >self.yolo_location_max:
                    self.yolo_location = 2


    def __face_judge(self):
        while not self.face_location:
            find_target, offset, offset_y, width = self.api.sustain_detect_face(self.recording_out)
            face_time = (self.__get_nowtime()-self.start_time_face)/1000
            print(f"t_0靠左计数为{self.face_location_judge[0]},t_0靠右计数为{self.face_location_judge[1]},已耗时{face_time}")
            if find_target and offset is not None and offset_y is not None:
                if offset < offset_y:
                    self.face_location_judge[0] += 1
                elif offset > offset_y:
                    self.face_location_judge[1] += 1
                if self.face_location_judge[0] > self.face_location_max:
                    self.face_location = 1
                elif self.face_location_judge[1] >self.face_location_max:
                    self.face_location = 2

    def __crossroad_location(self, direction):
        # direction=1为前进校正，direction=2为后退校正
        print("################################")
        grayscale_data = self.api.get_grayscale_data()
        print(grayscale_data)# 打印灰度值
        grayscale_data = self.api.get_grayscale_data()
        print(grayscale_data)# 打印灰度值
        # if self.locator.reach_target(grayscale_data):
        #     print("第一帧五个true，再后退一小段")
        #     self.api.move_backward(speed=20,run_time=300)
        #     time.sleep(0.3)
        #     self.api.stop()
        while True:
            grayscale_data = self.api.get_grayscale_data()
            print(grayscale_data)# 打印灰度值
            if self.locator.reach_target(grayscale_data):
                print("十字定位成功")
                break
            
            if direction == 1:
                print("前进校正----------------------------------------")
                if self.locator.translate_to_center(grayscale_data):
                    print("中心是黑色")
                    if self.locator_mode_error:
                        print("走特殊校正")
                        if self.locator.translate_left(grayscale_data):
                            print("135度移动")
                            self.api.move_translation(angle=135,speed=self.speed_locate_move)
                        elif self.locator.translate_right(grayscale_data):
                            print("-135度移动")
                            self.api.move_translation(angle=-135,speed=self.speed_locate_move)
                        else:
                            print("继续前进")
                            self.api.move_forward(self.speed_locate_move)
                    else:
                        print("走正常校正")
                        if self.locator.seeking_left(grayscale_data):
                            print("左转校正")
                            self.api.spin_left(self.speed_locate_spin)
                        elif self.locator.seeking_right(grayscale_data):
                            print("右转校正")
                            self.api.spin_right(self.speed_locate_spin)
                        else:
                            print("继续前进")
                            self.api.move_forward(self.speed_locate_move)

                else:
                    # 如果出现最左边三个灯或者最右边三个灯识别的情况，就出发error模式
                    self.locator_mode_error = 0
                    if self.locator.seeking_error(grayscale_data):
                        self.locator_mode_error = 1

                    if self.locator.move_straight(grayscale_data):
                        print("All False")
                        self.api.move_forward(self.speed_all_false)
                    else:
                        if self.locator.move_left(grayscale_data):
                            print("中心未检测到黑色，左平移")
                            self.api.move_left(speed=self.speed_locate_move)
                        elif self.locator.move_right(grayscale_data):
                            print("中心未检测到黑色，右平移")
                            self.api.move_right(speed=self.speed_locate_move)
                        else:
                            print("无法判断了")
                            pass
                    
            elif direction == 2:
                print("后退校正----------------------------------------")
                if self.locator.translate_to_center(grayscale_data):
                    print("中心是黑色")
                    if self.locator_mode_error:
                        print("走特殊校正")
                        if self.locator.translate_left(grayscale_data):
                            print("45度移动")
                            self.api.move_translation(angle=45,speed=self.speed_locate_move)
                        elif self.locator.translate_right(grayscale_data):
                            print("-45度移动")
                            self.api.move_translation(angle=-45,speed=self.speed_locate_move)
                        else:
                            print("继续后退")
                            self.api.move_backward(self.speed_locate_move)
                    else:
                        print("走正常校正")
                        if self.locator.seeking_left(grayscale_data):
                            print("右转校正")
                            self.api.spin_right(self.speed_locate_spin)
                        elif self.locator.seeking_right(grayscale_data):
                            print("左转校正")
                            self.api.spin_left(self.speed_locate_spin)
                        else:
                            print("继续后退")
                            self.api.move_backward(self.speed_locate_move)

                else:
                    # 如果出现最左边三个灯或者最右边三个灯识别的情况，就出发error模式
                    self.locator_mode_error = 0
                    if self.locator.seeking_error(grayscale_data):
                        self.locator_mode_error = 1

                    if self.locator.move_straight(grayscale_data):
                        print("All False")
                        self.api.move_backward(self.speed_all_false)
                    else:
                        if self.locator.move_left(grayscale_data):
                            print("中心未检测到黑色，左平移")
                            self.api.move_left(speed=self.speed_locate_move)
                        elif self.locator.move_right(grayscale_data):
                            print("中心未检测到黑色，右平移")
                            self.api.move_right(speed=self.speed_locate_move)
                        else:
                            print("无法判断了")
                            pass
    
            time.sleep(0.01)  # 避免过快循环
            
    # 新增：后台视觉循环（非阻塞）
    def _vision_loop(self):
        while not self._stop_event.is_set():
            try:
                # 手势阶段移动中
                if self.state_main == MainState.GESTURE and self.stage == 0 and self.gesture_type == 0:
                    find_gesture, number = self.api.detect_gesture()
                    if number == 3:
                        self.gesture_type_judge[0] += 1
                        if self.gesture_type_judge[0] > self.gesture_type_max:
                            self.gesture_type = 3
                    elif number == 5:
                        self.gesture_type_judge[1] += 1
                        if self.gesture_type_judge[1] > self.gesture_type_max:
                            self.gesture_type = 5
                    print(f"已识别成拳头{self.gesture_type_judge[0]}次，已识别成掌{self.gesture_type_judge[1]}次")

                # YOLO阶段移动中
                elif self.state_main == MainState.YOLO and self.stage == 0 and self.yolo_location == 0:
                    find_target, offset, offset_y, width = self.api.sustain_detect_yolo(self.recording_out)
                    if find_target and offset is not None and offset_y is not None:
                        if offset < offset_y:
                            self.yolo_location_judge[0] += 1
                            if self.yolo_location_judge[0] > self.yolo_location_max:
                                self.yolo_location = 1
                        elif offset > offset_y:
                            self.yolo_location_judge[1] += 1
                            if self.yolo_location_judge[1] > self.yolo_location_max:
                                self.yolo_location = 2
                        print(f"已识别成坦克靠左{self.yolo_location_judge[0]}次，已识别成坦克靠右{self.yolo_location_judge[1]}次")

                # 人脸阶段移动中
                elif self.state_main == MainState.FACE and self.stage == 0 and self.face_location == 0:
                    find_target, offset, offset_y, width = self.api.sustain_detect_face(self.recording_out)
                    if find_target and offset is not None and offset_y is not None:
                        if offset < offset_y:
                            self.face_location_judge[0] += 1
                            if self.face_location_judge[0] > self.face_location_max:
                                self.face_location = 1
                        elif offset > offset_y:
                            self.face_location_judge[1] += 1
                            if self.face_location_judge[1] > self.face_location_max:
                                self.face_location = 2
                        print(f"已识别成t_0靠左{self.face_location_judge[0]}次，已识别成t_0靠右{self.face_location_judge[1]}次")
                else:
                    time.sleep(0.01)
            except Exception:
                # 避免线程因临时错误退出
                time.sleep(0.02)
if __name__ == '__main__':
    controller = Controller()
    controller.run()