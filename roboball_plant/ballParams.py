class RoboBall2Params():
    """
    Important Measurable Values for the Ball Plant
    NOT found in the URDF
    """
    def __init__(self):
        self.steer_dynamic_friction = 1
        self.steer_static_friction = 0.8
        self.steer_viscous_damping = 0.22

        # robot urdf
        self.package_path = "./roboball_plant/RoboBall_URDF/package.xml"
        self.robot_file = "./roboball_plant/RoboBall_URDF/urdf/RoboBall_URDF.urdf"
        self.lumpy_robot_file = "./roboball_plant/RoboBall_URDF/urdf/RoboBall_URDF_lumpy.urdf"
      