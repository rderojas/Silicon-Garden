import time
import matplotlib.pyplot as plt
from Core.Converters import  EquipmentConverters, VoltageConverters
from Core.PID.PidEngine import PID

def test_pid_with_just_valve():
    controller = PID(0, 4, 1, 0)
    print(controller)

    #Appending time and outputs to PID controller:
    t_values = []
    y_values = []

    #Creating Valve
    test_valve = EquipmentConverters.valve_converter_factory(1000, 10)

    #Simulating PID for 3000 steps
    y = 0
    controller.reset()
    t_values.append(controller.get_time())
    y_values.append(y)
    for i in range(100):
        y = test_valve(controller.update(y))
        t_values.append(controller.get_time())
        y_values.append(y)
        time.sleep(0.001)

    controller.change_setpoint(5)

    for i in range(10000):
        y = test_valve(controller.update(y))
        t_values.append(controller.get_time())
        y_values.append(y)
        time.sleep(0.001)

    plt.plot(t_values, y_values)
    plt.show()

def test_pid_with_valve_and_divider():
    controller = PID(0, 40, 4, 0)
    print(controller)

    # Appending time and outputs to PID controller:
    t_values = []
    y_values = []

    # Creating Valve
    test_valve = EquipmentConverters.valve_converter_factory(1000, 10)

    #Creating Voltage Divider:
    test_divider = VoltageConverters.voltage_divider_factory(2, 1, True)

    # Simulating PID for 3000 steps
    start_time = time.time()
    y = 0
    controller.reset()
    t_values.append(controller.get_time())
    y_values.append(y)
    for i in range(100):
        y = test_valve(controller.update(test_divider(y)))
        t_values.append(controller.get_time())
        y_values.append(y)
        time.sleep(0.001)

    controller.change_setpoint(5)

    for i in range(10000):
        y = test_valve(controller.update(y))
        t_values.append(controller.get_time())
        y_values.append(y)
        time.sleep(0.001)

    plt.plot([t - start_time for t in t_values], y_values)
    plt.show()

def main():
    test_pid_with_valve_and_divider()


if __name__ == '__main__':
    main()



