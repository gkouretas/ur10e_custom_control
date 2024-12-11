import numpy as np

UR10E_FORCE_TORQUE_NODE = "tcp_fts_sensor"
UR10E_FORCE_PUBLISHER = "force"
UR10E_TORQUE_PUBLISHER = "torque"

UR_QOS_PROFILE = 1

UR_JOINT_LIST: list[str] = [
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
    "shoulder_pan_joint"
]

UR_HOME_POSE = [np.radians(-90.0), 0.0, np.radians(-90.0), 0.0, 0.0, 0.0]