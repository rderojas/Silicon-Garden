from typing import Callable
import matplotlib.pyplot as plt


# Converts valve voltage signal to valve % open
# then converts that to flowrate.
def valve_converter_factory(max_signal, max_flowrate):

    def signal_to_flow(u):
        return min(1,(u/max_signal)) * max_flowrate

    return signal_to_flow



def main():
    pass



if __name__ == '__main__':
    main()
