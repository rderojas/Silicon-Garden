#import copy
import numpy as np
import matplotlib.pyplot as plt
from Core.Models.ReactorModels import initialize_discrete_reactor
from Core.MPC.MPCEngine import MPC



def testMPC_reactor():
    test_time = 0.1
    timestep_ratios = 100
    model = initialize_discrete_reactor(test_time / timestep_ratios)
    mpc = MPC(model.__copy__(), timestep=test_time, model_length = 30, monitored_var=0, prediction_horizon = 20, control_horizon = 10, w = 1)
    mpc.update_setpoint(1)

    inputs = []
    input_indices = []
    outputs = []
    # Simulate for 1000 steps
    for i in range(100):
        # First compute our next input
        if i %10 == 0:
            print(f"At iteration {i}")
        curr_output = model.get_system_output()
        next_input = mpc.update(curr_output)
        inputs.append(next_input[0])
        inputs.append(next_input[0])
        input_indices.append(i)
        input_indices.append(i+1)
        for i in range(timestep_ratios):
            model.update(np.array([next_input]))
            outputs.append(model.get_system_output()[0])

    fig, ax = plt.subplots(1,2)
    ax[0].plot(outputs)
    ax[1].plot(input_indices, inputs)
    plt.show()












def main():
    testMPC_reactor()




if __name__ == '__main__':
    main()
