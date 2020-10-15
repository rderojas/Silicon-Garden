import time
from typing import Union


class PID:
    def __init__(self, setpoint: float, proportional: float,
                 integral: float, derivative: float) -> 'PID':
        self.r: float = setpoint
        self.Kp: float = proportional
        self.Ki: float = integral
        self.Kd: float = derivative
        self.prev_time: float = time.time()
        self.prev_error: Union[float,None] = None
        self.running_integral: float = 0.0

    def reset(self) -> None:
        self.prev_time: float = time.time()
        self.prev_error: Union[float,None] = None
        self.running_integral: float = 0.0

    def change_setpoint(self, new_setpoint: float) -> None:
        self.r: float = new_setpoint

    def update(self, error) -> float:

        # Initialize the input
        u: float = 0.0

        # Update u using the proportional value:
        proportional_value: float = self.Kp * error
        u += proportional_value

        curr_time: float = time.time()

        if self.prev_error is not None:
            # Update u using the running integral
            time_diff = curr_time - self.prev_time
            avg_error = (error + self.prev_error) / 2
            new_integral = time_diff * avg_error
            self.running_integral += new_integral
            u += self.Ki * self.running_integral

            # Update u using the derivative of the error
            error_diff = error - self.prev_error
            new_derivative = error_diff / time_diff
            u += self.Kd * new_derivative

        # Updating counters
        self.prev_error = error
        self.prev_time = curr_time
        return u


def main():
    pass



if __name__ == '__main__':
    main()
