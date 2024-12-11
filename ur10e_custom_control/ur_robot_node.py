import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.client import Client, SrvTypeRequest, Future
from rclpy.publisher import Publisher

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController
from std_srvs.srv import Trigger

from ur10e_typedefs import URService

from ur10e_configs import (
    UR_JOINT_LIST, 
    UR_QOS_PROFILE
)

from ur10e_typedefs import (
    URControlModes
)

from typing import Optional

_DEFAULT_SERVICE_TIMEOUT_SEC = 10
_DEFAULT_ACTION_TIMEOUT_SEC = 10

class URRobot:
    def __init__(self, node: Optional[Node] = None, **kwargs) -> None:
        if node is None:
            self._node = Node(**kwargs)
        else:
            self._node = node

        self.service_clients: dict[URService.URServiceType, Optional[Client]] = {
            k: None for k in URService.URServices
        }

        self.jtc_action_clients: dict[URControlModes, ActionClient] = \
            {mode: None for mode in URControlModes if mode.has_action_client}
        
        self._cyclic_publishers: dict[URControlModes, Publisher] = \
            {mode: None for mode in URControlModes if mode.is_cyclic}
        
        self._control_msg: dict[URControlModes, type] = \
            {mode: mode.publish_topic() for mode in URControlModes if mode.is_cyclic}

    def initialize_service(self, srv: URService.URServiceType) -> None:
        if self.service_clients[srv] is not None: 
            self._node.get_logger().info(f"Already have defined service for {srv}, ignoring")
        else:
            self.service_clients[srv] = URService.init_service(
                self._node, 
                srv,
                timeout = _DEFAULT_SERVICE_TIMEOUT_SEC
            )
            
    def wait_for_action(self, action_name: str, action_type: type, timeout: int =_DEFAULT_ACTION_TIMEOUT_SEC):
        client = ActionClient(self._node, action_type, action_name)
        if client.wait_for_server(timeout) is False:
            raise Exception(
                f"Could not reach action server '{action_name}' within timeout of {timeout}"
            )

        self._node.get_logger().info(f"Successfully connected to action '{action_name}'")
        return client
    
    def call_service(self, srv: URService.URServiceType, request: SrvTypeRequest):
        service = self.service_clients.get(srv)
        
        if service is None:
            self.service_clients[srv] = URService.init_service(self._node, srv, _DEFAULT_SERVICE_TIMEOUT_SEC)
            service = self.service_clients[srv]
        if request is None:
            request: SrvTypeRequest = URService.get_service_type(srv)
            assert request is not None, "Unable to get service type"
            request = request.Request()

        future: Future = service.call_async(request)
        rclpy.spin_until_future_complete(self._node, future)
        if future.result() is not None:
            return future.result()
        else:
            raise Exception(f"Exception while calling service: {future.exception()}")

    def call_action(self, ac_client: ActionClient, goal, blocking: bool):
        future = ac_client.send_goal_async(goal)
        if blocking:
            rclpy.spin_until_future_complete(self._node, future)
        else:
            future.add_done_callback(self.get_result_callback)

        if future.result() is not None:
            return future.result()
        else:
            raise Exception(f"Exception while calling action: {future.exception()}")

    def get_result(self, ac_client: ActionClient, goal_response):
        future_res = ac_client._get_result_async(goal_response)
        rclpy.spin_until_future_complete(self._node, future_res)
        if future_res.result() is not None:
            return future_res.result().result
        else:
            raise Exception(f"Exception while calling action: {future_res.exception()}")

    def set_controllers(self, controllers: list[URControlModes]):
        return self.call_service(
            URService.ControllerManager.SRV_SWITCH_CONTROLLER,
            SwitchController.Request(
                activate_controllers = controllers,
                start_controllers = controllers
            )
        )
    
    def stop_robot(self):
        return self.call_service(
            URService.DashboardClient.SRV_STOP,
            Trigger.Request()
        )

    def send_trajectory(self, waypts: list[list[float]], time_vec: list[Duration], blocking: bool = True):
        """Send robot trajectory."""
        if len(waypts) != len(time_vec):
            raise Exception("waypoints vector and time vec should be same length")
        
        # TODO: verify joint trajectory controller has been activated

        # Construct test trajectory
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = UR_JOINT_LIST
        for i in range(len(waypts)):
            point = JointTrajectoryPoint()
            point.positions = waypts[i]
            point.time_from_start = time_vec[i]
            joint_trajectory.points.append(point)

        # Sending trajectory goal
        if self.jtc_action_clients[URControlModes.SCALED_JOINT_TRAJECTORY] is None:
            self.jtc_action_clients[URControlModes.SCALED_JOINT_TRAJECTORY] = \
                self.wait_for_action(URControlModes.SCALED_JOINT_TRAJECTORY.action_type_topic, URControlModes.SCALED_JOINT_TRAJECTORY.action_type)
        
        goal_response = self.call_action(
            self.jtc_action_clients[URControlModes.SCALED_JOINT_TRAJECTORY], FollowJointTrajectory.Goal(trajectory = joint_trajectory), blocking = True
        )

        if not goal_response.accepted:
            raise Exception(f"Trajectory was not accepted: {goal_response}")

        # Verify execution
        # TODO: make option for non-blocking
        if blocking:
            result: FollowJointTrajectory.Result = self.get_result(self.jtc_action_clients[URControlModes.SCALED_JOINT_TRAJECTORY], goal_response)
            return result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        else:
            # TODO: return future
            raise RuntimeError("Non-blocking support for now...")

    def run_position_control(self):
        if self._cyclic_publishers[URControlModes.FORWARD_POSITION] is None:
            self._cyclic_publishers[URControlModes.FORWARD_POSITION] = \
                self._create_controller_publisher(
                    URControlModes.FORWARD_POSITION
                )
            
        # TODO: get current position
        self._control_msg[URControlModes.FORWARD_POSITION].data = [0.0] * len(UR_JOINT_LIST)

    def run_velocity_control(self):
        # self.set_controllers(controllers = [URControlModes.FORWARD_VELOCITY])
        
        if self._cyclic_publishers[URControlModes.FORWARD_VELOCITY] is None:
            self._cyclic_publishers[URControlModes.FORWARD_VELOCITY] = \
                self._create_controller_publisher(
                    URControlModes.FORWARD_VELOCITY
                )
            
        self._control_msg[URControlModes.FORWARD_VELOCITY].data = [0.0] * len(UR_JOINT_LIST)
        
    def publish_cyclic_commands(self):
        for mode in self._cyclic_publishers.keys():
            if self._cyclic_publishers[mode] is not None:
                msg = mode.publish_topic()
                msg = self._control_msg[mode]
                self._node.get_logger().info(f"Publishing: {msg} for {mode} from {self._cyclic_publishers[mode]}")
                self._cyclic_publishers[mode].publish(msg)

    def _create_controller_publisher(self, control_mode: URControlModes):
        return self._node.create_publisher(
            control_mode.publish_topic, 
            control_mode.publish_topic_name, 
            UR_QOS_PROFILE
        )
    
    def set_position(self, x: list[float]):
        # TODO: mutex
        self._control_msg[URControlModes.FORWARD_POSITION].data = x

    def set_position_by_joint_index(self, x: float, index: int):
        # TODO: mutex
        assert index >= 0 and index < len(UR_JOINT_LIST), f"Invalid joint index: {index}"
        self._control_msg[URControlModes.FORWARD_POSITION].data[index] = x

    def set_velocity(self, v: list[float]):
        # TODO: mutex
        self._control_msg[URControlModes.FORWARD_VELOCITY].data = v

    def set_velocity_by_joint_index(self, v: float, index: int):
        # TODO: mutex
        assert index >= 0 and index < len(UR_JOINT_LIST), f"Invalid joint index: {index}"
        self._control_msg[URControlModes.FORWARD_VELOCITY].data[index] = v