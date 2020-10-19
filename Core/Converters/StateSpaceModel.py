import numpy as np
import time
from scipy.linalg import expm
from scipy.integrate import quad
import sys


class StateSpaceModel:

    def __init__(self):
        pass

    def set_initial_state(self, x_initial):
        self.X = x_initial

    def set_initial_output(self, y_initial):
        self.Y = y_initial

    def reset_state(self):
        self.set_initial_state(np.zeros((len(self.A), 1)))
        self.set_initial_output(np.zeros((self.C.shape[0], 1)))

    def get_system_state(self):
        return self.X

    def get_system_output(self):
        return self.Y


class ContinuousStateSpaceModel(StateSpaceModel):

    def __init__(self, A, B, C, D, constraints = {}):
        super().__init__()
        self.A = A
        self.B = B
        self.C = C
        self.D = D
        self.X = np.zeros((len(A), 1))
        self.Y = np.zeros((C.shape[0], 1))
        self.prev_time = time.time()
        self.constraints = constraints

    def __str__(self):
        output_str = f"State Space System\n"
        output_str += f"A: {self.A}\n"
        output_str += f"B: {self.B}\n"
        output_str += f"C: {self.C}\n"
        output_str += f"D: {self.D}\n"
        output_str += f"X: {self.X}\n"
        output_str += f"Y: {self.Y}\n"
        return output_str

    def __copy__(self):
        new_model = ContinuousStateSpaceModel(self.A, self.B, self.C, self.D, self.constraints)
        new_model.X = self.X
        new_model.Y = self.Y
        new_model.prev_time = self.prev_time
        return new_model

    def reset_time(self):
        self.prev_time = time.time()

    def update(self, u):

        for ix in self.constraints.keys():
            bounds = self.constraints[ix]
            u[ix] = min(max(bounds[0], u[ix]),bounds[1])

        x_dot = np.dot(self.A,self.X) + np.dot(self.B, u)
        curr_time = time.time()
        self.X = self.X + (curr_time - self.prev_time) * x_dot
        self.Y = np.dot(self.C, self.X) + np.dot(self.D, u)
        self.prev_time = curr_time






class DiscreteStateSpaceModel(StateSpaceModel):

    def __init__(self, A, B, C, D, timestep, constraints = {}, continuous_matrices = True):
        super().__init__()
        if continuous_matrices:
            self.A = expm(A*timestep)
            function_matrix = A.copy().tolist()
            for i in range(A.shape[0]):
                for j in range(A.shape[1]):
                    vector_fun = (lambda r,c: (lambda t: expm(A*t)[r][c]))(i,j)
                    function_matrix[i][j] = vector_fun
            self.B = np.array(np.vectorize(lambda x,a,b: quad(x, a, b)[0])(function_matrix, 0, timestep)) @ B
        else:
            self.A = A
            self.B = B
        self.C = C
        self.D = D
        self.X = np.zeros((len(A), 1))
        self.Y = np.zeros((C.shape[0], 1))
        self.timestep = timestep
        self.constraints = constraints

    def __str__(self):
        output_str = f"State Space System\n"
        output_str += f"A: {self.A}\n"
        output_str += f"B: {self.B}\n"
        output_str += f"C: {self.C}\n"
        output_str += f"D: {self.D}\n"
        output_str += f"X: {self.X}\n"
        output_str += f"Y: {self.Y}\n"
        output_str += f"Timestep: {self.timestep} seconds"
        return output_str

    def __copy__(self):
        new_model = DiscreteStateSpaceModel(self.A, self.B, self.C, self.D, self.timestep, self.constraints, continuous_matrices=False)
        new_model.X = self.X
        new_model.Y = self.Y
        return new_model

    def reset_state(self):
        self.set_initial_state(np.zeros((len(self.A), 1)))

    def update(self, u):
        #Apply constraints
        for ix in self.constraints.keys():
            bounds = self.constraints[ix]
            u[ix] = min(max(bounds[0], u[ix]),bounds[1])

        # Apply timestep
        a = np.dot(self.D, u)
        self.Y = np.dot(self.C, self.X) + np.dot(self.D, u)
        self.X = np.dot(self.A, self.X) + np.dot(self.B, u)
