# RoboBall-Drake

This repository contains files that support the tests conducted in this paper

Oevermann et al. "A Modular Dynamic Model for A Soft Pendulum-Driven Spherical
Robots Using Drake," submitted to IEEE Robotics and Automation Letters

## Environment Setup
Then run the following on the command line. This will create a virtual environment in the env/ folder. You may choose to call it something else, but you will need to update the `.gitignore`
```
python3 -m venv env
```

activate the virtual environment

```
source env/bin/activate
```

Finally, install drake

```
pip install drake
```

you are now ready to run the examples in this repo!


## Figure Generation
the following commands will run the following experiments to generate plots from the paper

generates the friction model comparison graph

```
python3 roboball_plant/joint_modifiers.py
```
generates the friction models when implemented on the robot in the steering stand

```
python3 run_compare_friction_models.py
```
generates the model comparisons in the final section of the paper
```
python3 run_compare_steer_models.py
```
## Respository outline

`roboball_controllers/` is a directory that will house future controller prototypes to test with the simulated model

`roboball_plant/` contains useful setup scripts to create modular models of the robot
 - `create_ball_plant.py` defines two useful functions:  
    `add_RoboBall_plant()`loads in the robot urdf as a multibody plant in different useful configurations       
    `update_bedliner_properties()` to update the systems hydroelastic contact parameters onthe fly  

`roboball_controllers/` a directory to house future controller to add to experiments files

`utilities/` contains other non-robot specific features that are used in the paper.
 - `compare_models.py` contains a class that wraps the rigid body model from [1]
 - `world_features.py` contains a function to add a plate to the simulation for the robot to roll on

 
<img src="utilities/repo_structure.png" width=350 height=250>

## References
[1] D. J. Pravecek, M. J. Oevermann, G. C. Thomas and R. O. Ambrose, "Empirically Compensated Setpoint Tracking for Spherical Robots With Pressurized Soft-Shells," in IEEE Robotics and Automation Letters, vol. 10, no. 3, pp. 2136-2143, March 2025