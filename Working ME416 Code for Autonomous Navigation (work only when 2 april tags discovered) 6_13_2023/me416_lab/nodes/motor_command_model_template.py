"""Functions for modeling ROSBot"""

import numpy as np
from math import cos, sin

def model_parameters():
    """Returns two constant model parameters"""
    return k, d

def system_matrix(theta):
    """Returns a numpy array with the A(theta) matrix for a differential drive robot"""
    return A


def system_field(z, u):
    """Computes the field at a given state for the dynamical model"""
    return dot_z


def euler_step(z, u, stepSize):
    """Integrates the dynamical model for one time step using Euler's method"""
    return zp
