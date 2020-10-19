import time
import matplotlib.pyplot as plt
import numpy as np
from Core.Converters.StateSpaceModel import DiscreteStateSpaceModel

class MPC:



    def __init__(self, model, timestep, model_length, prediction_horizon, control_horizon, monitored_var, step_responses = None, w = 0):
        assert(0 <= control_horizon <= prediction_horizon <= model_length)
        self.timestep = timestep
        self.model_length = model_length
        self.prediction_horizon = prediction_horizon
        self.control_horizon = control_horizon
        self.monitored_var = monitored_var
        if step_responses is None:
            if model is None:
                raise Exception("Please specify either model or step response matrix")
            self.model: DiscreteStateSpaceModel = model
            self.S = self.compute_step_responses()
        self.step_matrix_past, self.step_matrix_future = self.compute_step_matrices()
        self.past_control_moves = np.zeros((self.model_length, 1))
        self.past_inputs = np.zeros((self.model_length, 1))
        self.input = 0
        self.setpoint = 0
        self.predicted_output = 0
        self.W = np.zeros((self.control_horizon, self.control_horizon))
        np.fill_diagonal(self.W, w)


    """
    The below functions precompute calculation matrices to avoid doing 
    so during the actual control phase
    """

    # For each input we analyze the step response and take params at N steps
    def compute_step_responses(self):

        step = np.zeros((self.model.B.shape[1], 1))
        step[self.monitored_var] = 1.0
        time_passed = 0.0
        input_s = []
        self.model.reset_state()

        while len(input_s) < self.model_length:
            self.model.update(step)
            time_passed+=self.model.timestep
            if time_passed > self.timestep:
                input_s.append(self.model.get_system_output()[0])
                time_passed = 0

        # plt.plot(input_s)
        # plt.show()
        return np.array(input_s)

    # Using the discrete model for state space evolution we compute step evolution matrices
    # For both past and future inputs
    def compute_step_matrices(self):
        step_matrix_future = np.zeros((self.prediction_horizon, self.control_horizon))
        for i in range(self.control_horizon):
            j=0
            while i + j < self.prediction_horizon:
                step_matrix_future[i+j][i] =  self.S[j]
                j+=1
        step_matrix_past = np.zeros((self.prediction_horizon, self.model_length - 2))

        for i in range(self.prediction_horizon):
            start_ix = 2
            for j in range(self.model_length - 2 - i):
                step_matrix_past[i][j] = self.S[j + i + 1]
            start_ix +=1
        return step_matrix_past, step_matrix_future

    def update_setpoint(self, new_setpoint):
        self.setpoint = new_setpoint


    def compute_disturbance_vector(self, curr_output):
        predicted_output = sum([self.S[i] * self.past_control_moves[i] for i in range(self.model_length -1)])
        predicted_output += self.S[-1] * self.past_inputs[-1]
        return np.full((self.prediction_horizon,1), curr_output - predicted_output)

    def compute_control_moves_vector(self):
        past_control_moves_vector = np.zeros((self.model_length - 2, 1))
        for i in range(self.model_length - 2):
            past_control_moves_vector[i] = self.past_control_moves[i]
        return past_control_moves_vector

    def compute_inputs_vector(self):
        # A bit trickier, we need to start in the second to last input and end in the -N + P one
        # So we walk the input array backwards, and the output array forwards
        past_inputs_vector = np.zeros((self.prediction_horizon, 1))
        for i in range(self.prediction_horizon):
            past_inputs_vector[i] = self.past_inputs[-2 - i]
        return past_inputs_vector

    def update_past_vector(self, vector, new_num):
        # Move everything down one column
        for i in range(vector.shape[0] - 2, -1, -1):
            vector[i+1] = vector[i]
        vector[0] = new_num

    def update(self, curr_output):
        disturbances_vector = self.compute_disturbance_vector(curr_output)
        control_moves_vector = self.compute_control_moves_vector()
        inputs_vector = self.compute_inputs_vector()
        unforced_predictions = (np.dot(self.step_matrix_past, control_moves_vector) + np.multiply(self.S[-1], inputs_vector) + disturbances_vector)
        unforced_error = self.setpoint - unforced_predictions
        next_control_moves = np.dot(np.linalg.inv(np.dot(self.step_matrix_future.T, self.step_matrix_future)+ self.W), np.dot(self.step_matrix_future.T, unforced_error))
        next_move = next_control_moves[0] # We only need the next control move
        next_input = self.input + next_move # Update the inputs
        self.update_past_vector(self.past_control_moves, next_move) # Move one step in the future
        self.update_past_vector(self.past_inputs, next_input) # Move one step into the future
        self.input = next_input
        return self.input # Return our next control signal


def main():
    pass


if __name__ == "__main__":
    main()
