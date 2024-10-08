import numpy as np
from trajectory_generators.trajectory_generator import TrajectoryGenerator


class Poly3(TrajectoryGenerator):
    def __init__(self, start_q, desired_q, T):
        self.T = T
        self.q_0 = start_q
        self.q_k = desired_q
        """
        Please implement the formulas for a_0 till a_3 using self.q_0 and self.q_k
        Assume that the velocities at start and end are zero.
        """
        self.a0 = self.q_0
        self.a3 = self.q_k
        # a1 = qr_0' + 3*a0
        self.a1 = 3 * self.a0
        self.a2 = 3 * self.a3

    def generate(self, t):
        """
        Implement trajectory generator for your manipulator.
        Positional trajectory should be a 3rd degree polynomial going from an initial state q_0 to desired state q_k.
        Remember to derive the first and second derivative of it also.
        Use following formula for the polynomial from the instruction.
        """
        t /= self.T
        q = self.a3 * t**3 + self.a2 * t**2 * (1 - t) + self.a1 * t * (1 - t)**2 + self.a0 * (1 - t)**3
        q_dot = 3*(self.a3 - self.a0 - self.a2 + self.a1)*(t**2) + 2*(self.a2 + 3*self.a0 - 2* self.a1)*t + self.a1 - 3 * self.a0
        q_ddot = 6*(self.a3 - self.a0 - self.a2 + self.a1)*t + 2*(self.a2 + 3*self.a0 - 2* self.a1)
        
        return q, q_dot / self.T, q_ddot / self.T**2
    

 