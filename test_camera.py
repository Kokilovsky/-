from sdk.api import UpAPI
import cv2


if __name__ == '__main__':
    api = UpAPI()

    while True:

        frame = api.get_camera_frame()
        cv2.imshow("camera", frame)
        cv2.waitKey(1)