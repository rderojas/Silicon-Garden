import numpy as np
from Core.Converters.StateSpaceModel import ContinuousStateSpaceModel, DiscreteStateSpaceModel


def initialize_discrete_reactor(timestep = 0.001):
    test_A = np.array([[-2.48048, 0], [0.83333, -2.2381]])
    test_B = np.array([[7], [-1.117]])
    test_C = np.array([[0, 1]])
    test_D = np.array([[0]])
    test_model = DiscreteStateSpaceModel(test_A, test_B, test_C, test_D, 0.001, constraints = {0: [-3,3]})
    print(test_model)
    return test_model
