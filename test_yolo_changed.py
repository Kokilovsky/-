from sdk.api import UpAPI
from sdk.model import YoloModel
import time
import cv2


def get_time_ms():
    return int(time.time() * 1000)


def test_yolo_detection(model_type):
    # 关键：你是 YOLOv8-pose 1类 person，不是 tank
    label_map = {
        YoloModel.VEHICLE: "person",
        YoloModel.WEAPON: "hammer",
        YoloModel.FACE: "good_person",
        YoloModel.GESTURE: "gesture_five"
    }
    target_label = label_map.get(model_type, "")
    if not target_label:
        print(f"不支持的模型类型: {model_type}")
        return

    try:
        # 关键：这里必须用 VEHICLE 通道加载你的 best.rknn（YOLOv8-pose）
        api = UpAPI(yolo_model=model_type, debug=True)
        print(f"\n===== 开始 YOLOv8-pose 人体检测 =====")
        print(f"识别目标标签: {target_label}")
    except Exception as e:
        print(f"API初始化失败: {e}")
        return

    try:
        while True:
            if api.preload_yolo_pool():
                find_target, offset_x = api.detect_yolo(label=target_label)

                current_time = time.strftime("%H:%M:%S", time.localtime())
                if find_target:
                    print(f"[{current_time}] 识别到目标: {target_label} | 偏移量X: {offset_x}")
                else:
                    print(f"[{current_time}] 未识别到目标: {target_label}")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print(f"\n用户中断识别，清理资源...")
    except Exception as e:
        print(f"识别过程出错: {e}")
    finally:
        api.reset_yolo_pool()
        api.close_yolo_window()
        if model_type == YoloModel.FACE:
            api.close_face_window()
        elif model_type == YoloModel.GESTURE:
            api.close_gesture_window()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    # 必须用 VEHICLE，因为你的 best.rknn 是加载在 VEHICLE 里的
    TEST_MODEL = YoloModel.VEHICLE
    test_yolo_detection(TEST_MODEL)