import numpy as np
from .controller import Controller
from models.manipulator_model import ManiuplatorModel


class MMAController(Controller):
    def __init__(self, Tp):
        # Fill the list self.models with 3 models of 2DOF manipulators with different m3 and r3
        # I:   m3=0.1,  r3=0.05
        # II:  m3=0.01, r3=0.01
        # III: m3=1.0,  r3=0.3
        self.Tp = Tp
        model_I = ManiuplatorModel(Tp)
        model_I.change_parameters(0.1, 0.05)
        model_II = ManiuplatorModel(Tp)
        model_II.change_parameters(0.01, 0.01)
        model_III = ManiuplatorModel(Tp)
        model_III.change_parameters(1.0, 0.3)

        self.Kd = np.array([0.1, 0.2])
        self.Kp = np.array([0.1,0.01])
        self.models = [model_I, model_II, model_III]
        self.i = 0
        self.u_p = np.zeros(2)
        self.x_p = np.zeros(4)

    def choose_model(self, x):
        # Implement procedure of choosing the best fitting model from self.models (by setting self.i)
        q_dot = np.reshape(self.x_p[2:], (2, 1))
        self.u_p = np.reshape(self.u_p, (2, 1))
        q_ddot0 = np.linalg.inv(self.models[0].M(self.x_p)) @ (self.u_p - self.models[0].C(self.x_p) @ q_dot)
        q_ddot1 = np.linalg.inv(self.models[1].M(self.x_p)) @ (self.u_p - self.models[1].C(self.x_p) @ q_dot)
        q_ddot2 = np.linalg.inv(self.models[2].M(self.x_p)) @ (self.u_p - self.models[2].C(self.x_p) @ q_dot)
        x_dot0 = np.concatenate((q_dot, q_ddot0))
        x_dot1 = np.concatenate((q_dot, q_ddot1))
        x_dot2 = np.concatenate((q_dot, q_ddot2))
        # print(f"xdot0: {x_dot0}")
        # print(f"self.models0.TP: {self.models[0].Tp}")
        # print(f"x {x}")
        z0 = self.x_p + np.squeeze(np.transpose(x_dot0 * self.Tp))
        z1 = self.x_p + np.squeeze(np.transpose(x_dot1 * self.Tp))
        z2 = self.x_p + np.squeeze(np.transpose(x_dot2 * self.Tp))
        e0 = np.linalg.norm(x - z0)
        e1 = np.linalg.norm(x - z1)
        e2 = np.linalg.norm(x - z2)
        if e0 <= e1:
            if e0 <= e2:
                self.i = 0
            else:
                self.i = 2
        else:
            if e1 <= e2:
                self.i = 1
            else:
                self.i = 2
        print(f"i: {self.i}")
        

    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        self.choose_model(x)
        q = x[:2]
        q_dot = x[2:]

        v = q_r_ddot#self.feedback(x, q_r, q_r_dot, q_r_ddot)
        M = self.models[self.i].M(x)
        C = self.models[self.i].C(x)
        u = M @ v[:, np.newaxis] + C @ q_dot[:, np.newaxis]

        self.u_p = u
        self.x_p = x
        
        return u
    
    def feedback(self, x, q_r, q_r_dot, q_r_ddot):
        q1, q2, q1_dot, q2_dot = x
        q = np.array([[q1],[q2]])
        q_dot = np.array([[q1_dot],[q2_dot]])
        v = q_r_ddot + self.Kd@(q_dot - q_r_dot) + self.Kp@(q - q_r)
        return v
    
    # def calculate_control_for_model_choosing(self, x, q_r, q_r_dot, q_r_ddot):

    #     for model in self.models:
    #         x_dot = (u - model.C*q_dot) / M
    #         self.x[0] = self.x[0] + self.Tp*x_dot
    #     v = q_r_ddot
    #     tau = np.dot(self.model.M(x), v) + np.dot(self.model.C(x), q_r_dot)
