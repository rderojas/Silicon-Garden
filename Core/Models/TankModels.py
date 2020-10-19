import numpy as np
from Core.Converters.StateSpaceModel import ContinuousStateSpaceModel, DiscreteStateSpaceModel


def initialize_simple_tank_with_hole_model(constraints = {}):
    test_A = np.zeros((1,1))
    test_B = np.zeros((1,2))
    test_C = np.zeros((1,1))
    test_D = np.zeros((1,2))

    test_A[0][0] = -1  #Hole at bottom of tank, volume out proportional to volume
    test_B[0][0] = 10 #Valve open fully has 10 l/sec out ?
    test_B[0][1] = -1 #Solar irradiance evaporates 1 l per unit of radiance?
    test_C[0][0] = 1 #Volume equals volume

    model = ContinuousStateSpaceModel(test_A, test_B, test_C, test_D, constraints)
    return model

def initialize_simple_tank_model(constraints = {}):
    test_A = np.zeros((1,1))
    test_B = np.zeros((1,2))
    test_C = np.zeros((1,1))
    test_D = np.zeros((1,2))

    test_B[0][0] = 10 #Valve open fully has 10 l/sec out ?
    test_B[0][1] = -1 #Solar irradiance evaporates 1 l per unit of radiance?
    test_C[0][0] = 1 #Volume equals volume

    model = ContinuousStateSpaceModel(test_A, test_B, test_C, test_D, constraints)
    return model
