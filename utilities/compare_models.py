import numpy as np
from pydrake.systems.framework import LeafSystem

class Pravecek_2025_Model(LeafSystem):
    '''
    Derived from Equation (8) in:
    D. J. Pravecek et al. "Empirically Compensated Setpoint Tracking for Spherical Robots 
    With Pressurized Soft-Shells," in IEEE Robotics and Automation Letters,
    vol. 10, no. 3, pp. 2136-2143, March 2025

    All defined on LHS such as: 
    calc_M() @ \ddot{q} + calc_C() @ \dot{q} + G() = U^T \\tau_m

    Note the paper defines the pipe angle in an
    '''
    def __init__(self):
        super().__init__()

        self.g = 9.81
        ''' params copied from URDF file for steer direction'''
        #masses
        self.m_pc = 2.77934 # [kg] mass of pitch_center
        self.m_p = 22.03    # [kg] mass of pendulm
        self.m_s = 16.53649 # [kg] mass of pipe assembly
        
        # geometry
        self.r_pc = 0.05    # [m] COM of pitch_center
        self.r_p = 0.095   # [m] COM of pendulum
        self.R = 0.3175      # [m] outer radius of bed_collision spherical mesh

        # inertias
        self.I_s = 1.04     # [kgm^2] Ix == Iz for shell or pipe_assembly
        self.I_pc = 0.01841 # [kgm^2] Ix for pitch_center
        self.I_p = 0.44235  # [kgm^2] Ix for pendulum

        # drake input and output ports
        self.DeclareContinuousState(4)
        self.get_state_output_port = self.DeclareVectorOutputPort("prav_state_output", 4, self.CalcModelResponse, {self.all_state_ticket()})

    def calc_M(self, phi, theta_g):
        I_eq = self.I_s + self.I_pc + self.m_pc*self.r_pc**2 + (self.m_s + self.m_pc + self.m_p)*self.R**2 - 2*self.m_pc*self.r_pc*self.R*np.cos(phi)
        M11 = I_eq
        M12 = -self.m_p*self.r_p*self.R*np.cos(theta_g)
        M22 = self.I_p+self.m_p*self.r_p**2

        return np.array([[M11, M12],
                         [M12, M22]])
    def calc_C(self, phi, theta_g, phi_dot, theta_g_dot):
        C1 = self.m_pc*self.r_pc*self.R*np.sin(phi)*phi_dot
        C2 = self.m_p*self.r_p*self.R*np.sin(theta_g)*theta_g_dot
        return np.array([[C1, C2],
                         [ 0,  0]])

    def calc_G(self, phi, theta_g):
        return np.array([[self.m_pc*self.r_pc*self.g*np.sin(phi)],
                         [self.m_p*self.r_p*self.g*np.sin(theta_g)]])
    
    def DoCalcTimeDerivatives(self, context, derivatives):
        # q of the form: phi, theta_g, phi_dot, theta_g_dot
        
        q = context.get_continuous_state_vector().CopyToVector()
        
        phi = q[0]
        phi_dot = q[2]
        theta_g = q[1]
        theta_g_dot = q[3]
        q_dot = np.array([[phi_dot],[theta_g_dot]])

        m_inv = np.linalg.inv(self.calc_M(phi, theta_g))
        c_q = self.calc_C(phi, theta_g, phi_dot, theta_g_dot) @ q_dot
        v_q = self.calc_G(phi, theta_g)
        q_ddot = m_inv @ (-c_q-v_q)
        # print(np.vstack((q_dot,q_ddot)))
        derivatives.get_mutable_vector().SetFromVector(np.vstack((q_dot,q_ddot)))

    def CalcModelResponse(self, context, output):

        q = context.get_continuous_state_vector().CopyToVector()
        q[1] = -q[0] + q[1] # theta = phi - theta_g
        q[3] = -q[2] + q[3]
        output.SetFromVector(q)
