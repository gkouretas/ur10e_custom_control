"""
Simulated sensor. For now, just publishes the x, y, z force/torque components as a sine wave
"""
import numpy as np
import time, timeit

from rclpy.publisher import Publisher
from geometry_msgs.msg import Vector3
from ur10e_configs import *
from python_utils.ros2_utils.comms.node_manager import create_simple_node

def main() -> None:
    force_torque_node = create_simple_node(UR10E_FORCE_TORQUE_NODE)

    force_publisher: Publisher = force_torque_node.create_publisher(Vector3, UR10E_FORCE_TORQUE_NODE + "/" + UR10E_FORCE_PUBLISHER, 0)
    torque_publisher: Publisher = force_torque_node.create_publisher(Vector3, UR10E_FORCE_TORQUE_NODE + "/" + UR10E_TORQUE_PUBLISHER, 0)

    f = 500.0
    T = (1.0 * 2*np.pi) # 1.0 second period
    X = 1.0 # 1.0 magnitude

    sampling_rate = 1/f

    while True:
        t = timeit.default_timer()

        force_out = Vector3()
        torque_out = Vector3()

        force_out.x = torque_out.x = X*np.sin(t*T)
        force_out.y = torque_out.y = X*np.sin(t*T + np.pi/2)
        force_out.z = torque_out.z = X*np.sin(t*T + np.pi)

        force_publisher.publish(force_out)
        torque_publisher.publish(torque_out)

        delay_s = max(0.0, sampling_rate - (timeit.default_timer() - t))
        time.sleep(delay_s)