import time
import functools
from typing import List, Callable, Any

import threading
import can
import numpy as np

# somewhat arbitrarily set for now - the timing seems good anything higher leads to breaks between speed
CMD_DELTA_TIME = .25
BRAKE_CUR = 5


def blank_func(**kwargs):
    pass


class SpeedThread(threading.Thread):
    def __init__(self, target: Callable[[], None]):
        super().__init__(target=target, args=())
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def is_stopped(self):
        return self._stop.is_set()


class MotorListener(can.Listener):
    def __init__(self, func: Callable[[...], None]) -> None:
        # this should take the form of def func(**kwargs)
        self.func = func

    def on_message_received(self, msg: can.Message) -> None:
        # TODO: check for correctness
        # different stats about the motor
        b = bytes(msg.data)
        pos_int = np.int32(b[0] << 8 | b[1])
        spd_int = np.int32(b[2] << 8 | b[3])
        cur_int = np.int32(b[4] << 8 | b[5])
        motor_pos = float(pos_int * 0.1)  # motor position
        motor_spd = float(spd_int / 21)  # motor speed
        motor_cur = float(cur_int * 0.01)  # motor current
        motor_temp = np.int16(b[6])  # motor temperature
        motor_error = b[7]
        self.func(arb_id=msg.arbitration_id, pos=motor_pos, spd=motor_spd, cur=motor_cur, temp=motor_temp,
                  error=motor_error)

    def stop(self) -> None:
        pass


class Motor():
    MOTOR_MODES = {
        "brake": 2,
        "speed": 3,
        "set_origin": 5,
        "speed_pos": 6,
    }

    def __init__(self, id: int, bus: can.ThreadSafeBus, listener_func: Callable[[...], None] = blank_func) -> None:
        self.id = id
        # self.listener = MotorListener(listener_func)
        self.thread = None
        self.cur_speed = 0

        self.bus = bus
        # can.Notifier(self.bus, [self.listener])

        self.last_update = time.time()

    @staticmethod
    def clear_motor(func: Callable[[...], Any]):
        @functools.wraps(func)
        def wrapper(self, *args, **kwargs):
            if self.thread and not self.thread.is_stopped():
                self.thread.stop()
                time.sleep(CMD_DELTA_TIME)

                # TODO: consider that:
                # we might not need break at all, it more prevents movement than just slowing down the rotor
                # self._send_brake(BRAKE_CUR)

            return func(self, *args, **kwargs)

        return wrapper

    def start(self) -> None:
        self._send_command(self.id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFC])

    @clear_motor
    def stop(self) -> None:
        self._send_command(self.id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFD])
        # self.bus.shutdown()

    @staticmethod
    def _put_data_in_array(data: int, num_bytes: int) -> List[int]:
        return [(data & (0xff << (i * 8))) >> (i * 8) for i in range(num_bytes)][::-1]

    def _send_speed(self, speed: int) -> None:
        speed *= 21 * 10
        data = self._put_data_in_array(speed, 4)
        print("erpm: ", speed)
        self._send_command(self.MOTOR_MODES["speed"] << 8 | self.id, data)

    @clear_motor
    def set_origin(self):
        data = self._put_data_in_array(0, 1)
        self._send_command(self.MOTOR_MODES["set_origin"] << 8 | self.id, data)

    @clear_motor
    def set_spd_pos(self, pos: int, speed: int, acc: int) -> None:
        # TODO: decide to finish or not
        pos *= 10000
        speed *= 21
        speed //= 10
        acc //= 10

        pos_data = self._put_data_in_array(pos, 4)
        speed_data = self._put_data_in_array(speed, 2)
        acc_data = self._put_data_in_array(acc, 2)

        self._send_command(self.MOTOR_MODES["speed_pos"] << 8 | self.id, pos_data + speed_data + acc_data)

    def _send_brake(self, amp: int) -> None:
        amp *= 1000
        data = self._put_data_in_array(amp, 4)
        self._send_command(self.MOTOR_MODES["brake"] << 8 | self.id, data)

    def _send_command(self, id: int, data: List[int]) -> None:
        # ensure we aren't overloading CAN line
        # now = time.time()
        # if self.last_update + CMD_DELTA_TIME > now:
        #     time.sleep(self.last_update + CMD_DELTA_TIME - now)
        # self.last_update = now

        msg = can.Message(
            arbitration_id=id,
            data = data,
            is_extended_id=True
        )
        print(msg)
        self.bus.send(msg)

    def set_speed(self, speed: int):
        self.cur_speed = speed
        # self._send_speed(self.cur_speed)
        if not self.thread or self.thread.is_stopped():
            self.thread = SpeedThread(target=self._maintain_speed)
            self.thread.start()

    def _maintain_speed(self) -> None:
        while not self.thread.is_stopped():
            self._send_speed(self.cur_speed)
            time.sleep(CMD_DELTA_TIME)

        # self._send_brake(BRAKE_CUR)
