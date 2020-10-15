from Core.Converters import EquipmentConverters, VoltageConverters
import matplotlib.pyplot as plt

def test_equipment_converters():
    test_max_signal = 200
    test_max_flowrate = 1000
    inputs = []
    outputs = []
    test_valve = EquipmentConverters.valve_converter_factory(
        test_max_signal,
        test_max_flowrate
    )
    for u in range(1000):
        inputs.append(u)
        outputs.append(test_valve(u))

    plt.plot(inputs, outputs)
    plt.show()

def test_voltage_converters():
    test_Vs = 2
    test_R1 = 1
    inputs = []
    outputs = []
    test_divider = VoltageConverters.voltage_divider_factory(
        test_Vs,
        test_R1
    )
    step = 0.0001
    u = 0
    for i in range(10000):
        inputs.append(u)
        outputs.append(test_divider(u))
        u+=step

    plt.plot(inputs, outputs)
    plt.show()





def main():
    test_voltage_converters()



if __name__ == '__main__':
    main()




