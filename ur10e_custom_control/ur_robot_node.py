import rclpy
import time
import threading
import copy

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from ur_msgs.action import DynamicForceModePath
from geometry_msgs.msg import Pose, PoseStamped, Wrench, Vector3, Quaternion, Twist, Point
from nav_msgs.msg import Path
from std_msgs.msg import Header

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.client import Client, SrvTypeRequest, Future
from rclpy.publisher import Publisher
from rclpy.executors import SingleThreadedExecutor

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController, ListControllers
from std_srvs.srv import Trigger

from ur10e_typedefs import URService

from ur10e_configs import (
    UR_JOINT_LIST, 
    UR_QOS_PROFILE
)

from ur10e_typedefs import (
    URControlModes
)

from ur_msgs.srv import SetFreedriveParams

from typing import Optional, Iterable, Literal

_DEFAULT_SERVICE_TIMEOUT_SEC = 120
_DEFAULT_ACTION_TIMEOUT_SEC = 10

class URRobot:
    def __init__(self, node: Optional[Node] = None, **kwargs) -> None:
        if node is None:
            self._node = Node(**kwargs)
        else:
            self._node = node

        self._lock = threading.RLock()

        self._service_node = Node("ur_robot_srv_node")

        self.service_clients: dict[URService.URServiceType, Optional[Client]] = {
            k: None for k in URService.URServices
        }

        self.jtc_action_clients: dict[URControlModes, ActionClient] = \
            {mode: None for mode in URControlModes if mode.has_action_client}
        
        self._cyclic_publishers: dict[URControlModes, Publisher] = \
            {mode: self._create_controller_publisher(mode) for mode in URControlModes if mode.is_cyclic}
        
        self._control_msg: dict[URControlModes, type] = \
            {mode: mode.publish_topic() for mode in URControlModes if mode.is_cyclic}
        
        self._freedrive_signal = threading.Event()
        
        self._control_loop_timer = self._node.create_timer(
            timer_period_sec = 1.0 / 500.0, # 500 Hz
            callback = self.publish_cyclic_commands
        )

        self._future_exec = SingleThreadedExecutor()

        # No autostart in ROS2 humble, it is in rolling though ._.
        # https://github.com/ros2/rclpy/pull/1138
        #self._control_loop_timer.cancel()

        self._freedrive_timer = self._node.create_timer(
            timer_period_sec = 1.0,
            callback = self.disable_freedrive
        )

        self._freedrive_dofs: list[bool] = [True, True, True, True, True, True]
        self._feature: Optional[int] = None

        self._active_control_mode: URControlModes = None

    def _get_all_active_controllers(self, exclude: str | None = None):
        response = self.list_controllers()
        active_controllers = []
        for controller in response.controller:
            if controller.name in URControlModes.controller_names() \
                and controller.state == "active" and \
                    ((controller.name != exclude) if exclude is not None else True):
                active_controllers.append(controller.name)

        return active_controllers

    def _start_cyclic_control(self):
        self._node.get_logger().info("Reset timer")
        self._control_loop_timer.reset()

    def _stop_cyclic_control(self):
        with self._lock:
            self._active_control_mode = None

    def get_freedrive_dofs(self):
        return copy.deepcopy(self._freedrive_dofs)

    def initialize_service(self, srv: URService.URServiceType) -> None:
        if self.service_clients[srv] is not None: 
            self._node.get_logger().info(f"Already have defined service for {srv}, ignoring")
        else:
            self.service_clients[srv] = URService.init_service(
                self._service_node, 
                srv,
                timeout = _DEFAULT_SERVICE_TIMEOUT_SEC
            )
            
    def wait_for_action(self, action_name: str, action_type: type, timeout: int =_DEFAULT_ACTION_TIMEOUT_SEC):
        self._node.get_logger().info(f"Attempting to start action {action_name} (type: {action_type})")
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
            self.service_clients[srv] = URService.init_service(self._service_node, srv, _DEFAULT_SERVICE_TIMEOUT_SEC)
            service = self.service_clients[srv]
        if request is None:
            request: SrvTypeRequest = URService.get_service_type(srv)
            assert request is not None, "Unable to get service type"
            request = request.Request()

        future: Future = service.call_async(request)
        rclpy.spin_until_future_complete(self._service_node, future)
        #while not future.done(): time.sleep(0.01)
        if future.result() is not None:
            return future.result()
        else:
            raise Exception(f"Exception while calling service: {future.exception()}")

    def call_action(self, ac_client: ActionClient, goal, blocking: bool):
        self._node.get_logger().info(f"Calling action client: {ac_client}")
        future = ac_client.send_goal_async(goal, feedback_callback=self._action_feedback)
        if False:
            # rclpy.spin_until_future_complete(self._service_node, future)
            self._node.get_logger().info("Result received")
            #while not future.done(): time.sleep(0.01)
        else:
            future.add_done_callback(self.get_result)
            return future

        # if future.result() is not None:
        #     return future.result()
        # else:
        #     raise Exception(f"Exception while calling action: {future.exception()}")

    def _action_feedback(self, feedback):
        self._node.get_logger().info(f"Feedback: {feedback}")

    def get_result(self, ac_client: ActionClient, goal_response):
        future_res = ac_client._get_result_async(goal_response)
        rclpy.spin_until_future_complete(self._service_node, future_res)
        self._node.get_logger().info("Result received")
        #while not future_res.done(): time.sleep(0.01)
        if future_res.result() is not None:
            return future_res.result().result
        else:
            raise Exception(f"Exception while calling action: {future_res.exception()}")

    def list_controllers(self) -> ListControllers.Response:
        return self.call_service(
            URService.ControllerManager.SRV_LIST_CONTROLLERS,
            ListControllers.Request()
        )

    def set_controllers(self, start: list[URControlModes], stop: list[URControlModes]):
        return self.call_service(
            URService.ControllerManager.SRV_SWITCH_CONTROLLER,
            SwitchController.Request(
                start_controllers = start,
                stop_controllers = stop
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
        
        self.set_controllers(
            start=[URControlModes.SCALED_JOINT_TRAJECTORY],
            stop=self._get_all_active_controllers(exclude=URControlModes.SCALED_JOINT_TRAJECTORY)
        )

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
            self.jtc_action_clients[URControlModes.SCALED_JOINT_TRAJECTORY], FollowJointTrajectory.Goal(trajectory = joint_trajectory), blocking = blocking
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
            # raise RuntimeError("Non-blocking support for now...")
            return True
        
    def run_dynamic_force_mode(self, poses: list[PoseStamped], blocking: bool = True):
        """Send robot trajectory."""
        self.set_controllers(
            start=[URControlModes.DYNAMIC_FORCE_MODE], 
            stop=self._get_all_active_controllers(exclude=URControlModes.DYNAMIC_FORCE_MODE)
        )

        # Construct test trajectory
        path = Path(
            poses = poses
        )

        goal = DynamicForceModePath.Goal(
            # task_frame = PoseStamped(
            #     pose=Pose(position=Point(x=0.0,y=0.0,z=0.0), orientation=Quaternion(w=1.0,x=0.0,y=0.0,z=0.0)),
            #     header=Header(frame_id='base')
            # ),
            task_frame = poses[0],
            wrench_baseline=Wrench(force=Vector3(x=0.0,y=0.0,z=0.0), torque=Vector3(x=0.0,y=0.0,z=0.0)),
            type=DynamicForceModePath.Goal.TCP_TO_ORIGIN,
            speed_limits=Twist(linear=Vector3(x=0.01,y=0.01,z=0.01),angular=Vector3(x=0.01,y=0.01,z=0.01)),
            force_mode_path=path,
            waypoint_tolerances=[0.025,0.025,0.025,0.025,0.025,0.025],
            deviation_limits=[1.0,1.0,1.0,1.0,1.0,1.0],
            compliance_tolerances=[DynamicForceModePath.Goal.ALWAYS_INACTIVE,
                                   DynamicForceModePath.Goal.ALWAYS_ACTIVE,
                                   DynamicForceModePath.Goal.ALWAYS_INACTIVE,
                                   DynamicForceModePath.Goal.ALWAYS_ACTIVE,
                                   DynamicForceModePath.Goal.ALWAYS_ACTIVE,
                                   DynamicForceModePath.Goal.ALWAYS_ACTIVE]
        )

        # Sending trajectory goal
        if self.jtc_action_clients[URControlModes.DYNAMIC_FORCE_MODE] is None:
            self.jtc_action_clients[URControlModes.DYNAMIC_FORCE_MODE] = \
                self.wait_for_action(URControlModes.DYNAMIC_FORCE_MODE.action_type_topic, URControlModes.DYNAMIC_FORCE_MODE.action_type)
        
        goal_response = self.call_action(
            self.jtc_action_clients[URControlModes.DYNAMIC_FORCE_MODE], goal, blocking = True
        )

        if not goal_response.accepted:
            raise Exception(f"Trajectory was not accepted: {goal_response}")

        # Verify execution
        # TODO: make option for non-blocking
        if blocking:
            result: DynamicForceModePath.Result = self.get_result(self.jtc_action_clients[URControlModes.DYNAMIC_FORCE_MODE], goal_response)
            return result.error_code == DynamicForceModePath.Result.SUCCESSFUL
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
        
    def run_freedrive_control(self):
        self.set_controllers(
            start = [URControlModes.FREEDRIVE_MODE],
            stop = self._get_all_active_controllers(exclude=URControlModes.FREEDRIVE_MODE)
        )

        if self._cyclic_publishers[URControlModes.FREEDRIVE_MODE] is None:
            self._cyclic_publishers[URControlModes.FREEDRIVE_MODE] = \
                self._create_controller_publisher(
                    URControlModes.FREEDRIVE_MODE
                )
            
        self._active_control_mode = URControlModes.FREEDRIVE_MODE
        self._start_cyclic_control()

    def stop_cyclic_control(self):
        self._active_control_mode = None
        self._stop_cyclic_control()

    stop_position_control = stop_cyclic_control
    stop_velocity_control = stop_cyclic_control
    stop_freedrive_control = stop_cyclic_control

    def publish_cyclic_commands(self):
        with self._lock:
            mode = self._active_control_mode

        if mode is None: return

        msg = mode.publish_topic()
        msg = self._control_msg[mode]

        self._node.get_logger().debug(f"Publishing: {msg} for {mode} from {self._cyclic_publishers[mode]}")
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

    def set_dof_state(self, state: bool, dofs: Iterable[int]):
        _kwargs = {"type": SetFreedriveParams.Request.TYPE_STRING}
        if self._feature is not None:
            _kwargs["feature_constant"] = self._feature

        for dof in dofs:
            self._freedrive_dofs[dof] = state
            self._node.get_logger().info(f"DoF index {dof} to {state}")
        self._node.get_logger().info(f"Free axes: {self._freedrive_dofs}")
        _kwargs["free_axes"] = self._freedrive_dofs

        self.call_service(URService.FreedriveController.SRV_SET_FREEDRIVE_PARAMS,
                          SetFreedriveParams.Request(**_kwargs))
        
    def set_feature(self, feature: Optional[int]):
        _kwargs = {"type": SetFreedriveParams.Request.TYPE_STRING, "free_axes": self._freedrive_dofs}

        if feature is None:
            self._feature = None
        else:
            assert feature == SetFreedriveParams.Request.FEATURE_BASE or feature == SetFreedriveParams.Request.FEATURE_TOOL
            self._feature = feature
            _kwargs["feature_constant"] = self._feature

        # TODO: add support for freedrive pose
        
        self.call_service(URService.FreedriveController.SRV_SET_FREEDRIVE_PARAMS,
                          SetFreedriveParams.Request(**_kwargs))

    def ping_freedrive(self):
        self._control_msg[URControlModes.FREEDRIVE_MODE].data = True
        self._freedrive_timer.reset()

    def disable_freedrive(self):
        self._control_msg[URControlModes.FREEDRIVE_MODE].data = False