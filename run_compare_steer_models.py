import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
import pandas as pd

from pydrake.systems.framework import DiagramBuilder

from pydrake.multibody.plant import (
    AddMultibodyPlant, 
    MultibodyPlantConfig
    )
from pydrake.geometry import SceneGraphConfig
from pydrake.all import (
    Simulator, 
    LogVectorOutput, 
    AddDefaultVisualization, 
    Meshcat, 
    Demultiplexer
)
from utilities.world_features import add_plate
from utilities.compare_models import Pravecek_2025_Model

from roboball_plant.create_ball_plant import (
    add_RoboBall_plant, 
    update_bedliner_properties
)
from roboball_plant.joint_modifiers import StictionModel


def plot_test_data(ax,log_directory):
    peaks = None
    first_time = None
    data = pd.read_csv(log_directory, header=0)
    # timestep is in nanoseconds
    data[['timestamp']] = data[['timestamp']] *1e-9

    peaks, _ = find_peaks(data['ball_roll.velocity'], height=0.05, distance=50)
    idx = 2 # the manually selected peak to start the sim at
    first_time = data['timestamp'].iloc[peaks].values
    ax.plot(data[['timestamp']].values- first_time[idx],data[['ball_roll.position']].values, label="pipe angle data")
    
    return first_time[idx]

def plot_prav_model(ax, init_conditions, tf, configs=(None,)):
    # set up the drake sim
    
    # Create a simple block diagram containing the pravecek system
    sys = Pravecek_2025_Model()

    builder = DiagramBuilder()
    mySys = builder.AddSystem(sys)  # add SimplerLinearSystem to diagram
   # wire loggers for data logging 
    logger_output = LogVectorOutput(mySys.get_output_port(), builder) # log the state output port
    
    diagram = builder.Build()

    # set ic's for pravecek model
    q0_p = [init_conditions[0], 
            init_conditions[0] + init_conditions[2],
            init_conditions[1], 
            init_conditions[1] + init_conditions[3]] # [phi, theta_g, dphi, dtheta_g]
    context = diagram.CreateDefaultContext()
    context.SetContinuousState(q0_p)

    # create the simulator, the modified context with the ICs must be included
    simulator = Simulator(diagram, context)
    simulator.AdvanceTo(tf)

    # Grab output results from Logger:
    log = logger_output.FindLog(context) # find output log with that context
    time = log.sample_times()
    data = log.data().transpose()

    # Grab input results from Logger:

    ax.plot(time, data[:, 0], label=f'Pravecek et al. - {configs}')
    
    return ax

def plot_drake_model(ax, initi_conditions, tf, config, new_proximity=None, meshcat=None):
    builder = DiagramBuilder()
    
    sceneConfig = SceneGraphConfig()
    sceneConfig.default_proximity_properties.compliance_type = "compliant"

    # check config conflicts:
    if ("soft" in config) and ("point" in config):
        raise ValueError("Cannot model both point and soft models")
    
    # check configs to set up sim
    if "soft" in config:
        plant, scene_graph = AddMultibodyPlant(
            MultibodyPlantConfig(
                time_step=0.001,
                penetration_allowance=0.001,
                contact_surface_representation="polygon",
                contact_model="hydroelastic"
                ), 
                sceneConfig, 
            builder)
    elif "point" in config:
        plant, scene_graph = AddMultibodyPlant(
            MultibodyPlantConfig(
                time_step=0.001,
                contact_model="point"
                ), builder)

    # insert a table (plant, angle [deg])
    plant = add_plate(plant, 0.0, visible=False)

    plant, model_idx = add_RoboBall_plant(plant, place_in_stand="steer", lumpy_bedliner=("lumpy" in config))
    
    if new_proximity:
        update_bedliner_properties(scene_graph, new_proximity)

    drake_logger = LogVectorOutput(plant.get_state_output_port(), builder)
    
    # impulse = builder.AddSystem(ExternalForce(plant, model_idx, 40, 0.01, 0.0))
    # builder.Connect(impulse.GetOutputPort("tool_head_force"), plant.get_applied_spatial_force_input_port())
    
    if "stiction" in config:
        steer_friction = builder.AddSystem(StictionModel([0.5, 0.8, 0.23]))
        steer_w_idx = plant.GetStateNames().index("RoboBall_URDF_steer_w")
        
        state_demuxer = builder.AddSystem(Demultiplexer(plant.num_multibody_states()))
        # demux the states
        builder.Connect(plant.get_state_output_port(), # split the state vector
                        state_demuxer.get_input_port())
        
        builder.Connect(state_demuxer.get_output_port(steer_w_idx),  # connect the steer speed
                        steer_friction.velocity_input_port)

        # Apply the calculated friction torque back to the joint
        builder.Connect(steer_friction.torque_output_port, plant.get_actuation_input_port())
    
    if meshcat:
        AddDefaultVisualization(builder, meshcat)
    
    diagram = builder.Build()

    diagram_context = diagram.CreateDefaultContext()
    
    drake_model_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
    # print(plant.GetStateNames())
    # set initial positions for drake model
    q0 = [00, 0.304, initi_conditions[0], initi_conditions[2]]
    plant.SetPositions(drake_model_context, q0)
    
    simulator = Simulator(diagram, diagram_context)

    if meshcat:
        meshcat.StartRecording()
    simulator.AdvanceTo(tf)
    if meshcat:
        meshcat.PublishRecording()


    # plot the models
    drake_log = drake_logger.FindLog(simulator.get_context())
    drake_data = drake_log.data().transpose()

    config_str = f""
    if "point" in config:
        config_str = f"{config} "
    elif "soft" in config:
        if new_proximity:
            config_str = f"{config} \n modulus (Pa): {np.round(new_proximity[0],1):e} - dissipation (s/m): {new_proximity[1]}"
        else:
            config_str = f"{config} \n modulus (Pa): 1.8e6 - dissipation (s/m): 0.5"

    # plot pipe angle
    ax.plot(drake_log.sample_times(), drake_data[:, 2], label=f'drake - '+ config_str)

    return ax
   


if __name__=="__main__":
    meshcat = Meshcat()
    log_diretory = "./roboball_plant/data/dynamic_data.csv"
    fig, ax_point = plt.subplots()
    fig, ax_soft =  plt.subplots()
    fig, ax_prav =  plt.subplots()
   
    # pull the initial condition from the datafile
    _ = plot_test_data(ax_prav, log_diretory) # same log same function dont need those vals
    _ = plot_test_data(ax_soft, log_diretory) # same log same function dont need those vals
    peak_times = plot_test_data(ax_point, log_diretory)
    # # make sure IC's are in the form:
    # [phi, dphi, theta, dtheta]
    # pulled from data 
    q0 = [-0.4, 0, 1, 0]
    tuned_proximity = [1.2e5, 0.65]
    tf = 5 # final integrating time

    ax_prav = plot_prav_model(ax_prav, q0, tf,  ("tau_flat",))
    ax_prav = plot_drake_model(ax_prav, q0, tf, ("point",))
    ax_prav = plot_drake_model(ax_prav, q0, tf, ("soft", "stiction"), tuned_proximity)


    ax_point = plot_prav_model(ax_point, q0, tf,  ("point", ))
    ax_point = plot_drake_model(ax_point, q0, tf, ("point",))
    ax_point = plot_drake_model(ax_point, q0, tf, ("point", "stiction"))
    ax_point = plot_drake_model(ax_point, q0, tf, ("point", "stiction", "lumpy"))


    ax_soft = plot_drake_model(ax_soft, q0, tf, ("soft", "stiction"))
    ax_soft = plot_drake_model(ax_soft, q0, tf, ("soft", "stiction", "lumpy"))
    ax_soft = plot_drake_model(ax_soft, q0, tf, ("soft", "stiction"), tuned_proximity, meshcat=meshcat)
    ax_soft = plot_drake_model(ax_soft, q0, tf, ("soft", "stiction", "lumpy"), tuned_proximity)

    
    ax_soft.set_title("Responses of Models with Hydroelastic Contact")
    ax_soft.set_xlabel("time (s)")
    ax_soft.set_ylabel("angle (rad)")
    ax_soft.set_xlim(left=0, right=tf+1)
    ax_soft.legend()
    ax_soft.grid()

    ax_point.set_title("Responses of Models with Point Contact")
    ax_point.set_xlabel("time (s)")
    ax_point.set_ylabel("angle (rad)")
    ax_point.set_xlim(left=0, right=tf+1)
    ax_point.legend()
    ax_point.grid()

    ax_prav.set_title("Pravecek's Models Compared with Modular Model")
    ax_prav.set_xlabel("time (s)")
    ax_prav.set_ylabel("angle (rad)")
    ax_prav.set_xlim(left=0, right=tf+1)
    ax_prav.legend()
    ax_prav.grid()

    plt.show()
