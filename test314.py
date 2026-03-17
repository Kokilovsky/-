from sdk.api import UpAPI
from sdk.logic_layer.pid import PIDController

if __name__ == '__main__':
    move_speed = 60
    k_p = 17.5
    k_i = 0.015
    k_d = 95

    api = UpAPI()
    pid = PIDController(k_p=k_p, k_i=k_i, k_d=k_d, max_output=1000)

    try:
        while True:
            grayscale_data = api.get_grayscale_data()
            offset = api.follow_line()
            turn_rate = pid.compute(offset)

            # 详细输出
            print(f"Offset: {offset:>7.2f} | TurnRate: {turn_rate:>6d} | Speed: {move_speed}")

            if offset is not None:
                api.move_rotation(speed=move_speed, turn_rate=turn_rate)
            else:
                api.move_forward(speed=move_speed)

    except KeyboardInterrupt:
        api.stop()
