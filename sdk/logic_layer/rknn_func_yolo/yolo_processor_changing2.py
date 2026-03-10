import cv2
import numpy as np
from ...model import YoloModel

class YoloProcessor:
    def __init__(self, yolo_model):
        if yolo_model == YoloModel.VEHICLE:
            self.classes = ("person",)
        elif yolo_model == YoloModel.WEAPON:
            self.classes = ("hammer", "sword", "knife")
        elif yolo_model == YoloModel.FACE:
            self.classes = ("good_person", "bad_person")
        elif yolo_model == YoloModel.GESTURE:
            self.classes = ("gesture_zero", "gesture_five")
        else:
            raise RuntimeError(f"Unknown YoloModel type: {yolo_model}")

        self.obj_threshold = 0.25
        self.nms_threshold = 0.7
        self.image_size = 640

    def __yolov8_post_process(self, outputs):
        if len(outputs) < 1:
            return None, None, None
    
        output = outputs[0]  # 添加这行
    
        # 根据输出维度判断是单类别还是多类别
        if output.shape[-1] == 5:  # 单类别 [N, 5]: x, y, w, h, conf
            boxes = output[..., :4]
            confs = output[..., 4:5]
            class_ids = np.zeros(len(boxes), dtype=int)
        elif output.shape[-1] > 5:  # 多类别 [N, 4+C]: x, y, w, h, conf, class_scores
            boxes = output[..., :4]
            confs = output[..., 4:5]
            class_scores = output[..., 5:]
            class_ids = np.argmax(class_scores, axis=-1)
        else:
            return None, None, None
    
        # 置信度过滤
        conf = confs[:, 0]
        keep = conf > self.obj_threshold
        boxes = boxes[keep]
        conf = conf[keep]
        class_ids = class_ids[keep]  # 只过滤class_ids，不需要class_scores
    
        if len(boxes) == 0:
            return None, None, None
    
        # xywh -> xyxy
        x, y, w, h = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        x1 = x - w / 2
        y1 = y - h / 2
        x2 = x + w / 2
        y2 = y + h / 2
        boxes_xyxy = np.stack([x1, y1, x2, y2], axis=1)
        boxes_xyxy = np.clip(boxes_xyxy, 0, self.image_size)
    
        # NMS
        keep_idx = cv2.dnn.NMSBoxes(
            boxes_xyxy.tolist(),
            conf.tolist(),
            self.obj_threshold,
            self.nms_threshold
        )
    
        if len(keep_idx) == 0:
            return None, None, None
    
        keep_idx = np.array(keep_idx).flatten()
        return boxes_xyxy[keep_idx], class_ids[keep_idx], conf[keep_idx]

    def __get_info(self, boxes, scores, classes, frame_shape=None):
        detections = []
        draw_info_list = []
        mid_screen_x = 320

        if frame_shape is not None:
            scale_x = frame_shape[1] / self.image_size
            scale_y = frame_shape[0] / self.image_size
        else:
            scale_x = 1.0
            scale_y = 480 / self.image_size

        for box, score, cl in zip(boxes, scores, classes):
            x1, y1, x2, y2 = box

            x1 = int(x1 * scale_x)
            y1 = int(y1 * scale_y)
            x2 = int(x2 * scale_x)
            y2 = int(y2 * scale_y)

            if frame_shape is not None:
                x1 = max(0, min(x1, frame_shape[1]))
                y1 = max(0, min(y1, frame_shape[0]))
                x2 = max(0, min(x2, frame_shape[1]))
                y2 = max(0, min(y2, frame_shape[0]))

            center = ((x1 + x2) // 2, (y1 + y2) // 2)
            offset_x = center[0] - mid_screen_x
            boundary = (x1, y1, x2, y2)

            name = self.classes[cl]
            detections.append((name, float(score), center, offset_x))
            draw_info_list.append((name, float(score), boundary, center))

        return detections, draw_info_list

    def __draw(self, image, draw_info_list):
        image = cv2.resize(image, (640, 480))
        for name, score, boundary, center in draw_info_list:
            top, left, right, bottom = boundary
            cv2.rectangle(image, (top, left), (int(right), int(bottom)), (255, 0, 0), 2)
            cv2.putText(image, f'{name} {score:.2f}',
                        (top, left - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.circle(image, center, 4, (0, 255, 0), 4)
        return image

    def process(self, rknn_lite, frame):
        original_h, original_w = frame.shape[:2]
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image_rgb = cv2.resize(image_rgb, (self.image_size, self.image_size))
    
        outputs = rknn_lite.inference(inputs=[image_rgb])
    
        # 根据yolov8n的实际输出格式调整
        # 可能需要reshape或transpose操作
        # 示例代码（需要根据实际情况修改）：
        # input_data = outputs[0].reshape([-1, self.image_size, self.image_size, 3])
    
        boxes, classes, scores = self.__yolov8_post_process(outputs)
    
        image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
        detections = []
    
        if boxes is not None:
            detections, draw_info_list = self.__get_info(
                boxes, scores, classes, frame_shape=(original_h, original_w))
            image = self.__draw(image, draw_info_list)
    
        return image, detections