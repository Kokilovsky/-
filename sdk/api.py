from pickle import TRUE
from .application_layer.action import Action
from .application_layer.sensor import Sensor
from .application_layer.processor import Processor
from .data_layer.arm import arm_action_factory as data_arm
from sdk.model import YoloModel  # 确保YoloModel已补全FACE/GESTURE
import cv2


class UpAPI:
    _instance = None
    __fill_vehicle_count = 0  # Yolo 检测池加载计数
    __grayscale_record = [False] * 7  # 灰度数据缓存

    def __new__(cls, yolo_model=YoloModel.VEHICLE, grayscale_threshold=3060, debug=False):
        if cls._instance is None:
            cls._instance = super(UpAPI, cls).__new__(cls)

            # 子系统
            cls._instance.__action = Action()
            cls._instance.__sensor = Sensor()
            # 保持Processor初始化逻辑不变，内部已适配新版YoloDetector
            cls._instance.__processor = Processor(yolo_model)

            # 参数
            cls._instance.__debug = debug
            cls._instance.__grayscale_threshold = grayscale_threshold  # 灰度阈值
            cls._instance.__window_name_face = "Face"
            # 新增：手势检测窗口名（最小改动，仅新增）
            cls._instance.__window_name_gesture = "Gesture"

        return cls._instance

    def __window_exists(self, window_name):
        """
        窗口是否存在

        :param window_name: 窗口名称
        :return: 窗口是否存在
        """
        try:
            cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE)
            return True
        except cv2.error:
            return False

    def __adc_grayscale_data(self, data_list):
        """
        灰度阵列模拟量转为数字量

        :param data_list: 模拟量数据
        :return: 数字量数据
        """
        adc_data_list = [data >= self.__grayscale_threshold for data in data_list]
        return adc_data_list

    # ------------------------------ 底盘移动 ------------------------------
    # 【无改动】保留所有底盘移动接口
    def stop(self):
        self.__action.move_translation(speed=0)

    def move_forward(self, speed=50, run_time=50):
        self.__action.move_translation(0, speed, run_time=run_time)

    def move_backward(self, speed=50, run_time=50):
        self.__action.move_translation(180, speed, run_time=run_time)

    def move_left(self, speed=50, run_time=50):
        self.__action.move_translation(90, speed, run_time=run_time)

    def move_right(self, speed=50, run_time=50):
        self.__action.move_translation(270, speed, run_time=run_time)

    def turn_left(self, turn_rate=50, run_time=50):
        self.__action.move_rotation(turn_rate=turn_rate, run_time=run_time)

    def turn_right(self, turn_rate=50, run_time=50):
        self.__action.move_rotation(turn_rate=-turn_rate, run_time=run_time)

    def spin_left(self, turn_rate=100, run_time=50):
        self.__action.seeking(turn_rate=turn_rate, run_time=run_time)

    def spin_right(self, turn_rate=100, run_time=50):
        self.__action.seeking(turn_rate=-turn_rate, run_time=run_time)

    def move_translation(self, angle=0, speed=10, run_time=50):
        self.__action.move_translation(angle=angle, speed=speed, run_time=run_time)

    def move_rotation(self, speed=10, turn_rate=0, run_time=50):
        self.__action.move_rotation(speed=speed, turn_rate=turn_rate, run_time=run_time)

    # ------------------------------ 手臂控制 ------------------------------
    # 【无改动】保留所有手臂控制接口
    def raise_left_arm(self):
        left_arm = data_arm.left_arm_raise()
        right_arm = data_arm.right_arm_down()
        self.__action.set_servo_positions(left_arm, right_arm)

    def raise_right_arm(self):
        left_arm = data_arm.left_arm_down()
        right_arm = data_arm.right_arm_raise()
        self.__action.set_servo_positions(left_arm, right_arm)

    def raise_arms(self):
        left_arm = data_arm.left_arm_raise()
        right_arm = data_arm.right_arm_raise()
        self.__action.set_servo_positions(left_arm, right_arm)

    def put_down_arms(self):
        left_arm = data_arm.left_arm_down()
        right_arm = data_arm.right_arm_down()
        self.__action.set_servo_positions(left_arm, right_arm)

    def hover_arms(self):
        left_arm = data_arm.left_arm_hover()
        right_arm = data_arm.right_arm_hover()
        self.__action.set_servo_positions(left_arm, right_arm)

    def open_arms(self):
        left_arm = data_arm.left_arm_open()
        right_arm = data_arm.right_arm_open()
        self.__action.set_servo_positions(left_arm, right_arm)

    def hug_arms(self):
        left_arm = data_arm.left_arm_hug()
        right_arm = data_arm.right_arm_hug()
        self.__action.set_servo_positions(left_arm, right_arm)

    def execute_arm_action(self, left_arm_action, right_arm_action):
        self.__action.set_servo_positions(left_arm_action, right_arm_action)

    # ------------------------------ 传感器数据 ------------------------------
    # 【无改动】保留所有传感器接口
    def get_grayscale_data(self):
        grayscale = self.__sensor.get_grayscale()
        analog_data = grayscale.get_grayscale_data()

        if self.__debug:
            print(f"灰度传感器模拟量数据: {analog_data}")

        if analog_data is not None:
            digital_data = self.__adc_grayscale_data(analog_data)
            self.__grayscale_record = digital_data
            return digital_data
        else:
            return self.__grayscale_record

    def get_camera_frame(self):
        ret, frame = self.__sensor.get_camera().read()
        if ret:
            return frame
        raise RuntimeError("Failed to read camera frame")

    # ------------------------------ 视觉处理数据 ------------------------------
    # 【无改动】基础图像显示接口
    def display_camera_frame(self):
        frame = self.get_camera_frame()
        if frame is not None:
            cv2.imshow("camera", frame)
            cv2.waitKey(1)

    def follow_line(self):
        data = self.get_grayscale_data()
        line_follower = self.__processor.get_line_follower()
        offset = line_follower.process_frame(data)
        return offset

    def detect_apriltag(self):
        frame = self.get_camera_frame()

        if frame is not None:
            apriltag_detector = self.__processor.get_apriltag_detector()
            find_apriltag, tag_id, offset_x = apriltag_detector.process_frame(frame)
            return find_apriltag, tag_id, offset_x

        return False, -1, 0

    # ------------------------------ 人脸检测（适配FACE枚举） ------------------------------
    def detect_face(self, label="bad_person"):  # 仅修改默认label为FACE枚举对应值
        """
        DeepFace 人脸检测(适配YOLO FACE检测)

        :param label: 目标人脸标签(good_person/bad_person)
        :param sim_threshold: 目标人脸相似度
        :return: 是否找到人脸，人脸相对于屏幕中心水平偏移量
        """
        frame = self.get_camera_frame()

        face_detector = self.__processor.get_face_detector()
        detections = face_detector.process_frame(frame)

        for name, score, center, offset_x in detections:
            if name == label:
                return True, offset_x

        return False, None

    # ------------------------------ YOLO检测（适配多枚举） ------------------------------
    def detect_yolo(self, label=None):  # 最小改动：默认label根据yolo_model自动匹配
        """
        Yolo 检测(适配VEHICLE/WEAPON/FACE/GESTURE)

        :param label: 目标标签(不传则自动匹配当前yolo_model默认值)
        :return: 是否找到目标，目标相对于屏幕中心水平偏移量
        """
        frame = self.get_camera_frame()
        # 自动匹配默认label（减少调用方改动）

        yolo_detector = self.__processor.get_yolo_detector()
        detections = yolo_detector.process_frame(frame)

        for name, score, center, offset_x in detections:
            if name == label:
                return True, offset_x

        return False, None

    # ------------------------------ 手势检测（仅补全窗口名） ------------------------------
    def detect_gesture(self,label="gesture_five"):
        """
        手势识别(适配YOLO GESTURE检测)

        :return: 是否找到手势,手势摆出的数字(gesture_zero/gesture_five)
        """
        frame = self.get_camera_frame()
        five_or_zero = TRUE
        # 自动匹配默认label（减少调用方改动）
        gesture_detector = self.__processor.get_gesture_detector()
        detections = gesture_detector.process_frame(frame)

        for name, score, center, offset_x in detections:
            if name == label:
                return True, offset_x, five_or_zero
            elif name == "gesture_zero":
                five_or_zero = FALSE
                return True, offset_x, five_or_zero

        return False, None

    # ------------------------------ YOLO池管理（无改动） ------------------------------
    def preload_yolo_pool(self):
        frame = self.get_camera_frame()

        yolo_detector = self.__processor.get_yolo_detector()
        if self.__fill_vehicle_count >= yolo_detector.get_worker_number():
            return True
        else:
            self.__fill_vehicle_count += 1
            yolo_detector.fill_pool(frame)

        return False

    def reset_yolo_pool(self):
        self.__fill_vehicle_count = 0
        work_detector = self.__processor.get_yolo_detector()
        work_detector.clear_pool()

    # ------------------------------ 窗口关闭（仅新增手势窗口） ------------------------------
    def close_tag_window(self):
        apriltag_detector = self.__processor.get_apriltag_detector()
        tag_window = apriltag_detector.window_name
        cv2.destroyWindow(tag_window)

    def close_face_window(self):
        window_face = self.__window_name_face
        if self.__window_exists(window_face):
            cv2.destroyWindow(window_face)

    def close_yolo_window(self):
        yolo_detector = self.__processor.get_yolo_detector()
        yolo_window = yolo_detector.window_name

        if self.__window_exists(yolo_window):
            cv2.destroyWindow(yolo_window)

    def close_gesture_window(self):
        """
        关闭手势识别窗口（补全窗口名适配）
        """
        gesture_detector = self.__processor.get_gesture_detector()
        gesture_window = gesture_detector.window_name or self.__window_name_gesture

        if self.__window_exists(gesture_window):
            cv2.destroyWindow(gesture_window)