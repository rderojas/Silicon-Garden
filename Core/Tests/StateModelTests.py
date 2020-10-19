from Core.Converters.StateSpaceModel import ContinuousStateSpaceModel, DiscreteStateSpaceModel
from Core.PID.PidEngine import PID
from Core.Models import ReactorModels, TankModels

import matplotlib.pyplot as plt
import numpy as np
import time
from math import cos


def testOpenLoop():
    #Testing a typical tank with a hole + solar_evaporation
    model = TankModels.initialize_simple_tank_with_hole_model()
    model.reset()
    #Lets simulate 1000 steps
    t_vals = []
    y_vals = []
    #Valve is half open
    u = np.array([0.5, 0])
    for i in range(1000):
        model.update(u)
        t_vals.append(model.prev_time)
        y_vals.append(model.get_system_state()[0][0])
        time.sleep(0.01)
    print(len(t_vals), len(y_vals))
    plt.plot(t_vals,y_vals)
    plt.show()
    pass

def testSinusoidialIrradiance(amplitude, frequency):
    #Testing a typical tank with a hole + solar_evaporation
    model = TankModels.initialize_simple_tank_model()
    model.reset()
    #Lets simulate 2000 steps
    t_vals = []
    y_vals = []
    #Valve is half open
    u = np.array([0.6, 0])
    for i in range(2000):
        model.update(u)
        u[1] = amplitude * cos(frequency * i) + amplitude
        t_vals.append(model.prev_time)
        y_vals.append(model.get_system_state()[0][0])
        time.sleep(0.01)
    print(len(t_vals), len(y_vals))
    plt.plot(t_vals,y_vals)
    plt.show()

def testSinusoidWithPIDControl(amplitude, frequency, P, I=0, D=0):
    # Initialize Model and controller
    model = TankModels.initialize_simple_tank_with_hole_model()
    controller = PID(2, P, I, D)

    # Lets simulate 1000 steps
    t_vals = []
    valve_vals = []
    y_vals = []
    # Valve is half open
    u = np.array([1, 0])
    model.reset()
    controller.reset()
    for i in range(2000):
        u[0] = controller.update(model.get_system_state()[0][0])
        u[1] = amplitude * cos(frequency * i) + amplitude
        model.update(u)
        t_vals.append(model.prev_time)
        valve_vals.append(u[0])
        y_vals.append(model.get_system_state()[0][0])
        time.sleep(0.01)
    plt.plot(t_vals,y_vals)
    plt.show()

def testConstrainedSinusoidWithPIDControl(amplitude, frequency, P, I=0.0, D=0.0):
    # Initialize Model and controller
    model = TankModels.initialize_simple_tank_model({0: [0,1]})
    controller = PID(setpoint=4, proportional=P, integral=I, derivative=D)

    # Lets simulate 1000 steps
    t_vals = []
    valve_vals = []
    y_vals = []
    # Valve is half open
    u = np.array([1, 0])
    model.reset()
    controller.reset()
    for i in range(2000):
        u[0] = controller.update(model.get_system_state()[0][0])
        u[1] = amplitude * cos(frequency * i) + amplitude
        model.update(u)
        t_vals.append(model.prev_time)
        valve_vals.append(u[0])
        y_vals.append(model.get_system_state()[0][0])
        time.sleep(0.01)
    print(y_vals)
    plt.plot(t_vals,y_vals)
    plt.show()

def discrete_reactor_test():
    test_model = ReactorModels.initialize_discrete_reactor()
    t_vals = []
    y_vals = []
    u = np.array([[1.0]])
    for i in range(10000):
        test_model.update(u)
        t_vals.append(i*test_model.timestep)
        y_vals.append(test_model.get_system_output()[0])
    plt.plot(t_vals, y_vals)
    plt.show()


def main():
    #testConstrainedSinusoidWithPIDControl(amplitude=2,frequency=0.01,P= 0.4, I = 1, D= 0.0)
    discrete_reactor_test()
    pass


if __name__ == '__main__':
    main()
