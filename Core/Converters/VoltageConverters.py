from typing import Callable
import matplotlib.pyplot as plt


# Returns a function that models the output voltage of a voltage divider
# Uses R2 as the force sensor
def voltage_divider_factory(v_s, r_fixed, increasing = True) -> Callable:

    def divider_decreasing(u) -> float:
        return v_s * (r_fixed / (r_fixed + u))

    def divider_increasing(u) -> float:
        return v_s * (u / (u + r_fixed))

    if increasing:
        return divider_increasing

    return divider_decreasing


def main():
    pass



if __name__ == '__main__':
    main()




