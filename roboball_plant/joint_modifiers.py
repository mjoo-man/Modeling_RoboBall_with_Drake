from pydrake.systems.framework import LeafSystem

class StictionModel(LeafSystem):
    """Defines a Coulomb based friction model to add to a specified joint

        @params params: 3x1 list or array [static_friction  [Nm], 
                                           dynamic_friction [Nm], 
                                           viscous damping  [Nms/rad]]

        @params Wiring Input Port: joint_w --> self.velocity_input_port
        @params Wiring Output Port: torque_output_port --> self.plant_actuation_port
    """
    def __init__(self, params=[0,0,0]):
        super().__init__()
       
        self.static_friction = params[0]  # Nm
        self.dynamic_friction = params[1]  # Nm
        self.viscous_damping = params[2]  # Nm*s/rad

        # Declare input ports for velocity
        self.velocity_input_port = self.DeclareVectorInputPort("velocity", 1)
        
        # Declare output port for torque to apply to the joint
        self.torque_output_port = self.DeclareVectorOutputPort("friction_torque", 1, self.calc_friction_torque)

    def stiction_model(self, velocity):
        # calculates the friction torque
        from numpy import sign
        # Example advanced friction model (e.g., stick-slip behavior)
        if abs(velocity) < 1e-3:  # Stick condition
            torque = -self.static_friction * sign(velocity)
        else:  # Slip condition
            torque = -self.dynamic_friction * sign(velocity) - self.viscous_damping * velocity

        return torque
    
    def calc_friction_torque(self, context, output):
        # wrap the output for tha leafsystem
        velocity = self.get_input_port(0).Eval(context)

        torque = self.stiction_model(velocity)
        
        output.SetFromVector([torque])


class StictionModel_Majd(LeafSystem):
    """ 
        based on eq6 from 
        https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=555719
        
    """
    def __init__(self, params=[0,0,0,0,0]):
        super().__init__()

        self.f_w = params[0] #0.23
        self.f_c = params[1] #0.7
        self.sigma = params[2] #10
        self.w_c = params[3] #1
        self.n = params[4] #1
        # Declare input ports for velocity
        self.velocity_input_port = self.DeclareVectorInputPort("velocity", 1)
        
        # Declare output port for torque to apply to the joint
        self.torque_output_port = self.DeclareVectorOutputPort("friction_torque", 1, self.calc_friction_torque)

    def majd_model(self, v):
        from numpy import sign, exp
        '''
        based on eq6 from 
        https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=555719
        '''
        
        param = -abs(v/self.w_c)
        
        return -self.f_w*v - (self.f_c + self.sigma*exp(param) - (self.sigma-self.f_c)*exp(self.n*param))*sign(v)
    
    def calc_friction_torque(self, context, output):
        from numpy import sign
        # Get joint velocity from the input port
        velocity = self.get_input_port(0).Eval(context)

        torque = self.majd_model(velocity)

        output.SetFromVector([torque])

if __name__=="__main__":
    from ballParams import RoboBall2Params
    import numpy as np
    import matplotlib.pyplot as plt

    # plot the different models
    stiction_params =  [RoboBall2Params().steer_static_friction,
                        RoboBall2Params().steer_dynamic_friction,
                        RoboBall2Params().steer_viscous_damping]
    majd_stiction_params = [0.23, 0.7, 10, 1, 1] # [f_w, f_c, sigma, w_c, n ]
    viscous_params = [0,0,RoboBall2Params().steer_viscous_damping]
    print(f"Running Test for stiction")
    stiction = StictionModel(stiction_params)
    print(f"Running Test for  Madj_et_al")
    majd_model =  StictionModel_Majd(majd_stiction_params)
    print("Running Test for Viscous Damping")
    visc_model = StictionModel(viscous_params)

    # velocities
    vels = np.linspace(-5, 5, 1000)
    torques = np.zeros_like(vels)
    plt.figure()
    plt.plot(vels, [stiction.stiction_model(v) for v in vels], color='#ff7f0e', label="Coulomb+Viscous Friction Model")
    plt.plot(vels, [visc_model.stiction_model(v) for v in vels], 'g', label="Viscous Model")
    plt.plot(vels, [majd_model.majd_model(v) for v in vels], 'r', label="Majd et al.")
    plt.legend()
    plt.grid()
    plt.title(f"Graphical Representation of Friction Models")
    plt.xlabel("velocity (m/s)")
    plt.ylabel("torque (Nm)")
    
    plt.show()