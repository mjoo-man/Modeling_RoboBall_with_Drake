from pydrake.all import (
    Demultiplexer, 
    Multiplexer,  
    DiagramBuilder,
    SceneGraphConfig,
    AddMultibodyPlant,
    MultibodyPlantConfig,
    Simulator,
    AddDefaultVisualization
)
from pydrake.systems.primitives import LogVectorOutput
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import find_peaks

from roboball_plant.create_ball_plant import add_RoboBall_plant
from roboball_plant.joint_modifiers import (
    StictionModel, 
    StictionModel_Majd
)

from roboball_plant.ballParams import RoboBall2Params

def plot_test_data(ax,log_directory, return_data=False):
    peaks = None
    first_time = None
    data = pd.read_csv(log_directory, header=0)
    
    peaks, _ = find_peaks(data['pend_angle'], height=0.05)
    ax.plot(data[['steer_timestamp']].values,data[['pend_angle']].values, label="Robot Data")
    first_time = data['steer_timestamp'].iloc[peaks].values[0]
    peaks_vals = data['pend_angle'].iloc[peaks].values[0]

    return first_time, peaks_vals
    

def run_test(meshcat, starting_angle, friction_model_class, friction_params):
   
    builder = DiagramBuilder()
    sceneConfig = SceneGraphConfig()
    sceneConfig.default_proximity_properties.compliance_type = "compliant"
    plant, scene_graph = AddMultibodyPlant(
        MultibodyPlantConfig(
            time_step=0.001,
            penetration_allowance=0.001,
            contact_surface_representation="polygon",
            contact_model="hydroelastic"
            ), sceneConfig, 
        builder)

    plant, model_idx = add_RoboBall_plant(plant, 
                                          place_in_stand="hanging")
    
    steer_q_idx = plant.GetStateNames().index("RoboBall_URDF_steer_q")
    steer_w_idx = plant.GetStateNames().index("RoboBall_URDF_steer_w")
    drive_w_idx = plant.GetStateNames().index("RoboBall_URDF_drive_w")

    drive_motor_idx = plant.GetActuatorNames().index("RoboBall_URDF_drive")
    steer_motor_idx = plant.GetActuatorNames().index("RoboBall_URDF_steer")
    
    drive_friction = builder.AddSystem(friction_model_class(friction_params))
    steer_friction = builder.AddSystem(friction_model_class(friction_params))
    # set up meshcat visualizer
    if meshcat != None:
        AddDefaultVisualization(builder, meshcat)

    state_demuxer = builder.AddSystem(Demultiplexer(plant.num_multibody_states()))
    control_muxer = builder.AddSystem(Multiplexer(2))
    
    # demux the states
    builder.Connect(plant.get_state_output_port(), # split the state vector
                    state_demuxer.get_input_port())
    
    builder.Connect(state_demuxer.get_output_port(steer_w_idx),  # connect the steer speed
                    steer_friction.velocity_input_port)
    builder.Connect(state_demuxer.get_output_port(drive_w_idx),  # connect the drive speed
                    drive_friction.velocity_input_port)
    # mux the forces
    builder.Connect(steer_friction.torque_output_port, control_muxer.get_input_port(steer_motor_idx))
    builder.Connect(drive_friction.torque_output_port, control_muxer.get_input_port(drive_motor_idx))
    
    # Apply the calculated friction torque back to the joint
    builder.Connect(control_muxer.get_output_port(), plant.get_actuation_input_port())

    logger = LogVectorOutput(plant.get_state_output_port(), builder)
    diagram = builder.Build()
    
    diagram_context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)

    # set initial consitoins to match the data file
    plant.SetPositions(plant_context, [0, starting_angle])
    
    simulator = simulator = Simulator(diagram, diagram_context)
    if meshcat !=None:
        meshcat.StartRecording()
    simulator.AdvanceTo(5)
    if meshcat !=None:
        meshcat.PublishRecording()
    
    log = logger.FindLog(simulator.get_context())
    times = log.sample_times()
    data = log.data().transpose()
    return times, data[:,steer_q_idx] # return the steer data

if __name__=="__main__":
   
    steer_log = "./roboball_plant/data/steering_static_stand_response.csv"
   
    meshcat = None
      
    fig, ax = plt.subplots()
    time_offset, starting_angle = plot_test_data(ax, steer_log)
    stiction_params =  [RoboBall2Params().steer_static_friction,
                        RoboBall2Params().steer_dynamic_friction,
                        RoboBall2Params().steer_viscous_damping]
    majd_stiction_params = [0.23, 0.7, 10, 1, 1] # [f_w, f_c, sigma, w_c, n ]
    viscous_params = [0,0,RoboBall2Params().steer_viscous_damping]
    print(f"Running Test for stiction")
    times, data = run_test(meshcat, starting_angle, StictionModel, stiction_params)
    print(f"Running Test for  Madj_et_al")
    times_maj, data_maj =  run_test(meshcat, starting_angle, StictionModel_Majd, majd_stiction_params)
    print("Running Test for Viscous Damping")
    times_visc, data_visc = run_test(meshcat, starting_angle, StictionModel, viscous_params)
    print(f"finishing tests, plotting ... ")
    plt.plot(times + time_offset, data, label="Coulomb + Viscous Friction Model")
    plt.plot(times + time_offset, data_visc, label="Viscous Model")
    plt.plot(times_maj + time_offset, data_maj, label="Majd et al.")
    plt.legend()
    plt.grid()
    plt.title(f"Robot Data vs Friction Models")
    plt.xlabel("time(s)")
    plt.ylabel("Angle (rad)")
    plt.show()