from .application_layer.action import Action
from .application_layer.sensor import Sensor
from .application_layer.processor import Processor
from .data_layer.arm import arm_action_factory as data_arm
from .model import YoloModel
import cv2
import time


class UpAPI:
    _instance = None
    __fill_vehicle_count = 0  # Yolo 检测池加载计数
    __grayscale_record = [False] * 7  # 灰度数据缓存

    def __new__(cls, yolo_model=YoloModel.VEHICLE, grayscale_threshold=1000, debug=False):
        if cls._instance is None:
            cls._instance = super(UpAPI, cls).__new__(cls)

            # 子系统
            cls._instance.__action = Action()
            cls._instance.__sensor = Sensor()
            cls._instance.__processor = Processor(yolo_model)

            # 参数
            cls._instance.__debug = debug
            cls._instance.__grayscale_threshold = grayscale_threshold  # 灰度阈值
            cls._instance.__window_name_face = "Face"

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

    def stop(self):
        """
        停止
        """
        self.__action.move_translation(speed=0)

    def move_forward(self, speed=50, run_time=50):
        """
        向前移动

        :param speed: 移动速度  0~100
        :param run_time: 执行时间
        """
        self.__action.move_translation(0, speed, run_time=run_time)

    def move_backward(self, speed=50, run_time=50):
        """
        向后移动

        :param speed: 移动速度  0~100
        :param run_time: 执行时间
        """
        self.__action.move_translation(180, speed, run_time=run_time)

    def move_left(self, speed=50, run_time=50):
        """
        向左移动

        :param speed: 移动速度  0~100
        :param run_time: 执行时间
        """
        self.__action.move_translation(90, speed, run_time=run_time)

    def move_right(self, speed=50, run_time=50):
        """
        向右移动

        :param speed: 移动速度  0~100
        :param run_time: 执行时间
        """
        self.__action.move_translation(270, speed, run_time=run_time)

    def turn_left(self, turn_rate=50, run_time=50):
        """
        前进左转

        :param turn_rate: 旋转速率  Mode为0、1时取值范围：-100~100，Mode为2时取值范围：-1000~1000
        :param run_time: 执行时间
        """
        self.__action.move_rotation(turn_rate=turn_rate, run_time=run_time)

    def turn_right(self, turn_rate=50, run_time=50):
        """
        前进右转

        :param turn_rate: 旋转速率  Mode为0、1时取值范围：-100~100，Mode为2时取值范围：-1000~1000
        :param run_time: 执行时间
        """
        self.__action.move_rotation(turn_rate=-turn_rate, run_time=run_time)

    def spin_left(self, turn_rate=100, run_time=50):
        """
        原地左旋

        :param turn_rate: 旋转速率  Mode为0、1时取值范围：-100~100，Mode为2时取值范围：-1000~1000
        :param run_time: 执行时间
        """
        self.__action.seeking(turn_rate=turn_rate, run_time=run_time)

    def spin_right(self, turn_rate=100, run_time=50):
        """
        原地右旋

        :param turn_rate: 旋转速率  Mode为0、1时取值范围：-100~100，Mode为2时取值范围：-1000~1000
        :param run_time: 执行时间
        """
        self.__action.seeking(turn_rate=-turn_rate, run_time=run_time)

    def move_translation(self, angle=0, speed=10, run_time=50):
        """
        水平移动

        :param angle: 平移角度  0~360  (正前方为0°，逆时针为正方向)
        :param speed: 移动速度  0~100
        :param run_time: 执行时间
        """
        self.__action.move_translation(angle=angle, speed=speed, run_time=run_time)

    def move_rotation(self, speed=10, turn_rate=0, run_time=45):
        """
        转向

        :param speed: 移动速度  0~100
        :param turn_rate: 旋转速率  Mode为0、1时取值范围：-100~100，Mode为2时取值范围：-1000~1000  (大于0左转，小于0右转)
        :param run_time: 执行时间
        """
        self.__action.move_rotation(speed=speed, turn_rate=turn_rate, run_time=run_time)

    # ------------------------------ 手臂控制 ------------------------------

    def raise_left_arm(self):
        """
        举起左手，放下右手
        """
        left_arm = data_arm.left_arm_raise()
        right_arm = data_arm.right_arm_down()
        self.__action.set_servo_positions(left_arm, right_arm)

    def raise_right_arm(self):
        """
        举起右手，放下左手
        """
        left_arm = data_arm.left_arm_down()
        right_arm = data_arm.right_arm_raise()
        self.__action.set_servo_positions(left_arm, right_arm)

    def raise_arms(self):
        """
        举起双手
        """
        left_arm = data_arm.left_arm_raise()
        right_arm = data_arm.right_arm_raise()
        self.__action.set_servo_positions(left_arm, right_arm)

    def put_down_arms(self):
        """
        放下双手
        """
        left_arm = data_arm.left_arm_down()
        right_arm = data_arm.right_arm_down()
        self.__action.set_servo_positions(left_arm, right_arm)

    def hover_arms(self):
        """
        悬空双手
        """
        left_arm = data_arm.left_arm_hover()
        right_arm = data_arm.right_arm_hover()
        self.__action.set_servo_positions(left_arm, right_arm)

    def open_arms(self):
        """
        展开双手
        """
        left_arm = data_arm.left_arm_open()
        right_arm = data_arm.right_arm_open()
        self.__action.set_servo_positions(left_arm, right_arm)

    def hug_arms(self):
        """
        双手抱拳
        """
        left_arm = data_arm.left_arm_hug()
        right_arm = data_arm.right_arm_hug()
        self.__action.set_servo_positions(left_arm, right_arm)

    def execute_arm_action(self, left_arm_action, right_arm_action):
        self.__action.set_servo_positions(left_arm_action, right_arm_action)

    # ------------------------------ 传感器数据 ------------------------------

    def get_grayscale_data(self):
        """
        获取灰度阵列数字量数据

        :return: 数字量数据
        """
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
        """
        从帧图像队列获取相机数据

        :return: 相机图像数据
        """
        ret, frame = self.__sensor.get_camera().read()
        if ret:
            return frame
        time.sleep(0.03)
        raise RuntimeError("Failed to read camera frame")

    # ------------------------------ 视觉处理数据 ------------------------------

    def display_camera_frame(self):
        """
        显示图像
        """
        frame = self.get_camera_frame()
        if frame is not None:
            cv2.imshow("camera", frame)
            cv2.waitKey(1)

    def follow_line(self):
        """
        灰度巡线

        :return: 黑线偏移值
        """
        data = self.get_grayscale_data()
        line_follower = self.__processor.get_line_follower()
        offset,length = line_follower.process_frame(data)
        return offset,length

    def detect_apriltag(self):
        """
        检测 Apriltag

        :return: 是否找到 Apriltag，Apriltag 的 ID，水平方向偏移量，宽度像素值，高度像素值
        """
        frame = self.get_camera_frame()

        if frame is not None:
            apriltag_detector = self.__processor.get_apriltag_detector()
            find_apriltag, tag_id, offset_x = apriltag_detector.process_frame(frame)
            return find_apriltag, tag_id, offset_x

        return False, -1, 0

    def detect_face(self, label="bad_person", sim_threshold=0.4):
        """
        DeepFace 人脸检测

        :param label: 目标人脸标签
        :param sim_threshold: 目标人脸相似度
        :return: 是否找到人脸，人脸相对于屏幕中心水平偏移量
        """
        frame = self.get_camera_frame()

        face_detector = self.__processor.get_face_detector()
        detections = face_detector.detect_faces_in_image(frame, sim_threshold=sim_threshold)

        image = face_detector.draw_bounding_boxes(frame, detections)
        cv2.imshow(self.__window_name_face, image)
        cv2.waitKey(1)

        for name, score, center, offset_x in detections:
            if name == label:
                return True, offset_x

        return False, None
    
    def detect_gesture(self):
        """
        手势识别
        
        frame = self.get_camera_frame()
     
        gesture_detector = self.__processor.get_gesture_detector()
        number = gesture_detector.process(frame)

        if number:
            return True, number
        return False, None
        """
        A = int(time.time() * 1000)
        frame = self.get_camera_frame()
        print(f"frame_time = {int(time.time() * 1000)-A}")

        yolo_detector = self.__processor.get_yolo_detector()
        detections = yolo_detector.process_frame(frame)  # 这一句有问题
    
        # 初始化变量为 None
        k = None
    
        # 遍历所有检测结果
        for name, score, center, offset_x in detections:
            print(f"score={score},name={name}")
            if score > 0.85:
                if name == "gesture_zero":
                    k = 3  # 举左手
                elif name == "gesture_five":
                    k = 5  # 举双手
    
        # 只有在找到至少一个目标时才返回 True
        if k is not None :
            return True, k
        else:
            return False, None

    # 这一段是我的思路
    def detect_yolo(self):
        frame = self.get_camera_frame()
    
        yolo_detector = self.__processor.get_yolo_detector()
        detections = yolo_detector.process_frame(frame)
    
        # 初始化变量为 None
        m = None
        n = None
    
        # 遍历所有检测结果
        for name, score, center, offset_x in detections:
            if name == "tank":
                m = offset_x  # 记录坦克偏移量
            elif name == "civilian_car":
                n = offset_x  # 记录救护车偏移量
    
        # 只有在找到至少一个目标时才返回 True
        if m is not None and n is not None:
            return True, m, n
        else:
            return False, None, None
        
    #  在手势十字提前看yolo的方案
    def pre_detect_yolo(self):
        frame = self.get_camera_frame()
    
        yolo_detector = self.__processor.get_yolo_detector()
        detections = yolo_detector.process_frame(frame)
    
        # 初始化变量为 None
        m = None
        n = None
    
        # 遍历所有检测结果
        for name, score, center, offset_x in detections:
            if name == "tank":
                m = offset_x  # 记录坦克偏移量
            elif name == "civilian_car":
                n = offset_x  # 记录救护车偏移量
        if n == None or m == None:
            if n != None:
                return False, "只读到civilian_car"
            elif m != None:
                return False, "只读到tank"
            else:
                return False, "一个没读到"
        else:
            if n > 280 and m < 280:
                return True, "tank"
            elif m > 280 and n < 280:
                return True, "civilian_car"
            elif m > 280 and n > 280:
                print("两个对象都在识别区外")
                if m < n:
                    return True, "tank"
                else:
                    return True, "civilian_car"
            elif m < 280 and n < 280:
                print("两个对象都在识别区内")
                if m < n:
                    return True, "tank"
                else:
                    return True, "civilian_car"

    #  边斜着走边识别yolo
    def sustain_detect_yolo(self,out):
        frame = self.get_camera_frame()
        # 输出录像
        #out.write(frame)
    
        yolo_detector = self.__processor.get_yolo_detector()
        detections = yolo_detector.process_frame(frame)
    
        # 初始化变量为 None
        m = None
        n = None
    
        # 遍历所有检测结果
        for name, score, center, offset_x in detections:
            if name == "tank":
                m = offset_x  # 记录坦克偏移量
            elif name == "civilian_car":
                n = offset_x  # 记录救护车偏移量
    
        # 只有在找到至少一个目标时才返回 True
        if m != None and n != None:
            return True, m, n
        else:
            return False, None, None

    def sustain_detect_face(self,out):
        frame = self.get_camera_frame()

        # 输出录像
        #out.write(frame)
    
        yolo_detector = self.__processor.get_yolo_detector()
        detections = yolo_detector.process_frame(frame)
    
        # 初始化变量为 None
        m = None
        n = None
    
        # 遍历所有检测结果
        for name, score, center, offset_x in detections:
            if name == "bad_person":
                m = offset_x  # 记录坏人偏移量
            elif name == "good_person":
                n = offset_x  # 记录好人偏移量
    
        # 只有在找到坏人时才返回 True
        if m != None and n != None:
            return True, m, n
        else:
            return False, None, None

    #  最开始yolo识别人脸
    def pre_detect_face(self):
        frame = self.get_camera_frame()
    
        yolo_detector = self.__processor.get_yolo_detector()
        detections = yolo_detector.process_frame(frame)
    
        # 初始化变量为 None
        m = None
        n = None
    
        # 遍历所有检测结果
        for name, score, center, offset_x in detections:
            if name == "bad_person":
                m = offset_x  # 记录坏人偏移量
            elif name == "good_person":
                n = offset_x  # 记录好人偏移量

        print(f"m={m},n={n}")
        if m == None or n == None:
            return False, "两个人脸中至少一个未识别"
        elif m < n:
            if n - m < 10:
                return False, "二者太接近，重新识别"
            else:
                return True, "坏人在左边"
        else:
            if m - n < 10:
                return False, "二者太接近，重新识别"
            else:
                return True, "坏人在右边"
            
    def preload_yolo_pool(self):
        """
        Yolo 预加载

        :return: 是否预加载完成
        """
        frame = self.get_camera_frame()

        yolo_detector = self.__processor.get_yolo_detector()
        if self.__fill_vehicle_count >= yolo_detector.get_worker_number():
            return True
        else:
            self.__fill_vehicle_count += 1
            yolo_detector.fill_pool(frame)

        return False

    def reset_yolo_pool(self):
        """
        重置 Yolo 线程池
        """
        self.__fill_vehicle_count = 0
        tank_detector = self.__processor.get_yolo_detector()
        tank_detector.clear_pool()

    def close_tag_window(self):
        """
        关闭 April Tag 检测调试窗口
        """
        apriltag_detector = self.__processor.get_apriltag_detector()
        tag_window = apriltag_detector.window_name
        cv2.destroyWindow(tag_window)

    def close_face_window(self):
        """
        关闭人脸检测调试窗口
        """
        window_face = self.__window_name_face
        if self.__window_exists(window_face):
            cv2.destroyWindow(window_face)

    def close_yolo_window(self):
        """
        关闭 Yolo 调试窗口
        """
        yolo_detector = self.__processor.get_yolo_detector()
        yolo_window = yolo_detector.window_name

        if self.__window_exists(yolo_window):
            cv2.destroyWindow(yolo_window)

    def close_gesture_window(self):
        """
        关闭手势识别窗口
        """
        gesture_detector = self.__processor.get_gesture_detector()
        gesture_window = gesture_detector.window_name

        if self.__window_exists(gesture_window):
            cv2.destroyWindow(gesture_window)

    def detect_yolo1(self):
        frame = self.get_camera_frame()
    
        yolo_detector = self.__processor.get_yolo_detector()
        detections = yolo_detector.process_frame(frame)
    
        # 初始化变量为 None
        m = None
        n = None
    
        # 遍历所有检测结果
        for name, score, center, offset_x in detections:
            if name == "tank":
                m = offset_x
            elif name == "civilian_car":
                n = offset_x  # 记录民用汽车偏移量
    
        # 只有在找到至少一个目标时才返回 True
        if m is not None and n is not None:
            return True, m, n
        else:
            return False, None, None

    def recording_prepare(self):
        cap = self.__sensor.get_camera()
        frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(cap.get(cv2.CAP_PROP_FPS))

        # 如果无法获取帧率，设置默认值
        if fps <= 0:
            fps = 30  # 常见摄像头的默认帧率

        # 定义视频编码器（FourCC）
        # 常用编码器：'XVID'（通用）、'MJPG'、'H264'、'MP4V'
        fourcc = cv2.VideoWriter_fourcc(*'XVID')  # 也可以使用 'mp4v' 保存为 MP4 格式

        # 创建 VideoWriter 对象
        # 参数：输出文件名、编码器、帧率、帧大小
        out = cv2.VideoWriter('output.avi', fourcc, fps, (frame_width, frame_height))

        return out

    def reset_recording(self):
        self.__sensor.get_camera().release()
        self.recording_prepare().release()

    # 关闭
    def clean_up(self):
        self.__processor.clean_up()

    # 以下函数为多线程方案专属    simple.py --------------------------------------------------------------------------------------------------------------
    def simple_get_camera(self):
        cap = self.__sensor.get_camera()
        return cap

    def simple_detect_gesture(self, frame):
        yolo_detector = self.__processor.get_yolo_detector()
        detections = yolo_detector.process_frame(frame)  # 这一句有问题
        # 初始化变量为 None
        k = None
        print("进入遍历")
        # 遍历所有检测结果
        for name, score, center, offset_x in detections:
            print(f"score={score},name={name}")
            if score > 0.85:
                if name == "gesture_zero":
                    k = 3  # 举左手
                elif name == "gesture_five":
                    k = 5  # 举右手
        print(f"k={k}")
    
        # 只有在找到至少一个目标时才返回 True
        if k is not None :
            return True, k
        else:
            return False, None

    def simple_detect_yolo(self, frame):
        yolo_detector = self.__processor.get_yolo_detector()
        detections = yolo_detector.process_frame(frame)
    
        # 初始化变量为 None
        m = None
        n = None
    
        # 遍历所有检测结果
        for name, score, center, offset_x in detections:
            if name == "tank":
                m = offset_x  # 记录坦克偏移量
            elif name == "civilian_car":
                n = offset_x  # 记录民用汽车偏移量
    
        # 只有在找到至少一个目标时才返回 True
        if m != None and n != None:
            return True, m, n
        else:
            return False, None, None

    def simple_detect_face(self, frame):
        yolo_detector = self.__processor.get_yolo_detector()
        detections = yolo_detector.process_frame(frame)
    
        # 初始化变量为 None
        m = None
        n = None
    
        # 遍历所有检测结果
        for name, score, center, offset_x in detections:
            if name == "bad_person":
                m = offset_x  # 记录坏人偏移量
            elif name == "good_person":
                n = offset_x  # 记录好人偏移量
    
        # 只有在找到坏人时才返回 True
        if m != None and n != None:
            return True, m, n
        else:
            return False, None, None

    #  测试摄像头，跑掉第一次超长time
    def camera_test(self, frame):
        print("打开摄像头画面")
        yolo_detector = self.__processor.get_yolo_detector()
        detections = yolo_detector.process_frame(frame)
        return True