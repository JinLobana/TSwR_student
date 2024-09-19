import numpy as np


class ManiuplatorModel:
    def __init__(self, Tp):
        self.Tp = Tp
        self.l1 = 0.5
        self.d1 = self.l1/2
        self.r1 = 0.04
        self.m1 = 3.0
        self.l2 = 0.4
        self.d2 = self.l2/2
        self.r2 = 0.04
        self.m2 = 2.4
        self.I_1 = 1 / 12 * self.m1 * (3 * self.r1 ** 2 + self.l1 ** 2)
        self.I_2 = 1 / 12 * self.m2 * (3 * self.r2 ** 2 + self.l2 ** 2)
        self.m3 = 0.1
        self.r3 = 0.05
        self.I_3 = 2. / 5 * self.m3 * self.r3 ** 2
        # parameters for m3 = 0
        # self.alpha = self.m1*(self.d1**2) + self.I_1 + self.m2*((self.l1**2) + (self.d2**2)) + self.I_2
        # self.beta  = self.m2*self.l1*self.d2
        # self.gamma = self.m2*(self.d2**2) + self.I_2
        # parameters for m3 /= 0
        self.alpha = self.m1*self.d1**2 + self.I_1 + self.m2*(self.l1**2 + self.d2**2) + self.I_2 + self.m3*(self.l1**2 + self.l2**2) + self.I_3
        self.beta = self.l1*(self.m2*self.d2 + self.m3*self.l2)
        self.gamma = self.m2*self.d2**2 + self.I_2 + self.m3*self.l2**2 + self.I_3

    def change_parameters(self, m3, r3):
        self.m3 = m3
        self.r3 = r3
        self.alpha = self.m1*self.d1**2 + self.I_1 + self.m2*(self.l1**2 + self.d2**2) + self.I_2 + self.m3*(self.l1**2 + self.l2**2) + self.I_3
        self.beta = self.l1*(self.m2*self.d2 + self.m3*self.l2)
        self.gamma = self.m2*self.d2**2 + self.I_2 + self.m3*self.l2**2 + self.I_3

    # first I implemented these commented out methods with model. I calculated everything, double checked, but something was wrong with that model, 
    # I have no idea what. I even I even multiplied everything again and compared it with the reference, until I switched into working model. 
    def M(self, x):
        """
        Please implement the calculation of the mass matrix, according to the model derived in the exercise
        (2DoF planar manipulator with the object at the tip)
        """
        q1, q2, q1_dot, q2_dot = x
        M = np.array([[self.alpha + 2 * self.beta * np.cos(q2), self.gamma +  self.beta * np.cos(q2)],  [self.gamma + self.beta * np.cos(q2), self.gamma]])
        return M

    def C(self, x):
        """
        Please implement the calculation of the Coriolis and centrifugal forces matrix, according to the model derived
        in the exercise (2DoF planar manipulator with the object at the tip)
        """
        q1, q2, q1_dot, q2_dot = x
        C = np.array([[-self.beta * np.sin(q2) * q2_dot, -self.beta * np.sin(q2) * (q1_dot + q2_dot)],[self.beta * np.sin(q2) * q1_dot, 0]])
        return C

    # def M(self, x):
    #     """
    #     Please implement the calculation of the mass matrix, according to the model derived in the exercise
    #     (2DoF planar manipulator with the object at the tip)
    #     """
    #     q1, q2, q1_dot, q2_dot = x
    #     alpha = self.m1*(self.d1**2) + self.I_1 + self.m2*(self.l1**2 + self.d2**2) + self.I_2 + self.m3*(self.l1**2 + self.d2**2) + self.I_3
    #     beta = self.m2 * self.l1 * self.d2
    #     gamma = self.m2 * (self.d2**2) + self.I_2
    #     delta = self.m3 * self.l1 * self.l2
    #     epsilon = self.m3 * (self.l2**2) + self.I_3

    #     # choose which one to comment out
    #     # without m3
    #     # M_matrix = np.array([[self.alpha + 2*self.beta*np.cos(q2), self.gamma + self.beta*np.cos(q2)],[self.gamma + self.beta*np.cos(q2), self.gamma]])
    #     # with m3
    #     M_matrix = np.array([[alpha + epsilon + (2 * np.cos(q2) * (beta + delta)), epsilon + gamma + np.cos(q2) * (beta + delta)], [gamma + epsilon + np.cos(q2) * (delta + beta), gamma + epsilon]])

    #     return M_matrix

    # def C(self, x):
    #     """
    #     Please implement the calculation of the Coriolis and centrifugal forces matrix, according to the model derived
    #     in the exercise (2DoF planar manipulator with the object at the tip)
    #     """
    #     alpha = self.m1*(self.d1**2) + self.I_1 + self.m2*(self.l1**2 + self.d2**2) + self.I_2 + self.m3*(self.l1**2 + self.d2**2) + self.I_3
    #     beta = self.m2 * self.l1 * self.d2
    #     gamma = self.m2 * (self.d2**2) + self.I_2
    #     delta = self.m3 * self.l1 * self.l2
    #     epsilon = self.m3 * (self.l2**2) + self.I_3
    #     q1, q2, q1_dot, q2_dot = x
    #     # C_matrix = np.array([[-1*self.beta*np.sin(q2)*q2_dot, -1*self.beta*np.sin(q2)*(q1_dot + q2_dot)],[self.beta*np.sin(q2)*q1_dot, 0]])
    #     C_matrix = np.array([[-2*np.sin(q2)*q2_dot*(beta+delta), - (beta + delta)*np.sin(q2)*(q2_dot + q1_dot)],[-(beta + delta)*np.sin(q2)*(q1_dot), 0]])
    #     return C_matrix 
    
    
        