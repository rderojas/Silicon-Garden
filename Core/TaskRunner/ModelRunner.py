import numpy as np
import matplotlib.pyplot as plt
from Core.Converters.StateSpaceModel import DiscreteStateSpaceModel
from Core.MPC.MPCEngine import MPC


#Simulation params
simulation_length = 10000 # How many updates are done by the MPC


## Params for the state space model:
# Enter your state model Matrices (as numpy arrays)

# Example shown is EPA chemical decomposer
# Outputs are Chemical Concentrations, inputs are flow in and concentration in
A = np.array([[-1.25, 0.0], [0.05, -0.32015]]) #Change if you want
B = np.array([[0.001667, 0.25], [0.0001216, 0.0]]) # Change if you want
C = np.array([[0.0, 1.0]]) #Change if you want
D = np.array([[0.0, 0.0]]) # Change if you want

# Enter your input constraints in the form {<index_of_input> : [<lower_bound>, <upper_bound>]}
constraints = {}

# Select true if the state space model you entered is in continuous form, else select False
continuous_matrices = True


## Params for the model predictive control (You will definitely want to play with this stuff):
setpoint = 1.0 # Select the output you want to reach
timestep = 10.0 # Select how often MPC updates (in the same units as the SS Model)
timestep_ratio = 100 # Select how much faster the model updates over the SS Model
model_length = 50 #Select how long the model remembers past inputs
prediction_horizon = 30 # Select how far the model predicts
control_horizon = 20 # Select how far away the model creates controls
monitored_var = 0 #Select the index of the variable the model monitors
step_responses = None # If you have an array of step responses, use this instead of a model (for experimental data)
w = 0.01 # L2 regularization parameter (suggested you use more than zero to get meaningful inputs

def main():
    model = DiscreteStateSpaceModel(A, B, C, D, timestep=timestep/timestep_ratio, constraints=constraints, continuous_matrices=continuous_matrices)
    mpc = MPC(model.__copy__(), timestep=timestep, model_length=model_length, monitored_var=monitored_var, prediction_horizon=prediction_horizon,
              control_horizon=control_horizon, w=w)
    mpc.update_setpoint(setpoint)

    inputs = []
    input_indices = []
    outputs = []
    # Simulate for 1000 steps
    for i in range(simulation_length):
        # First compute our next input
        if i % 10 == 0:
            print(f"At iteration {i}")
        curr_output = model.get_system_output()
        next_input = np.zeros((model.B.shape[1], 1)) # size of input vector equal to the number of columns in B
        next_input[monitored_var] = mpc.update(curr_output)
        inputs.append(next_input[monitored_var])
        inputs.append(next_input[monitored_var])
        input_indices.append(i)
        input_indices.append(i + 1)
        for i in range(timestep_ratio):
            model.update(next_input)
            outputs.append(model.get_system_output()[0])

    fig, ax = plt.subplots(1, 2)
    ax[0].plot(outputs)
    ax[1].plot(input_indices, inputs)
    plt.show()


if __name__ == "__main__":
    main()
