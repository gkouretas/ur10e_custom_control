import time

from rclpy.node import Node
from ur10e_typedefs import URService
from ur_robot_node import URRobot
from rclpy.subscription import Subscription
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle

from ur_dashboard_msgs.msg import RobotMode, SafetyMode
from ur_dashboard_msgs.action import SetMode
from ur10e_configs import UR_QOS_PROFILE

class URRobotSM(URRobot):
    def __init__(self, node: Node | None = None, **kwargs) -> None:
        """
        Python implementation `robot_state_helper` developed on https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/pull/933
        
        Parent class to `URRobot` object, which already handles lower-level service calls.
        
        Args:
            node_name (str): Node name
        """
        super().__init__(node, **kwargs)

        self._in_action = False
        self._current_robot_mode = RobotMode.NO_CONTROLLER
        self._current_safety_mode = SafetyMode.UNDEFINED_SAFETY_MODE
        self._goal = SetMode.Goal()

        # Result seemingly not implemented in ServerGoalGHandle methods, so unused
        self._result = SetMode.Result()
        
        self._feedback = SetMode.Feedback()
        self._current_goal_handle: ServerGoalHandle = None
        self._set_mode_action_server: ActionServer = None

        self._robot_state_sub: Subscription = self._node.create_subscription(
            msg_type = RobotMode,
            topic = "/io_and_status_controller/robot_mode",
            callback = self._robot_mode_callback,
            qos_profile = UR_QOS_PROFILE
        )

        self._safety_mode_sub: Subscription = self._node.create_subscription(
            msg_type = SafetyMode,
            topic = "/io_and_status_controller/safety_mode",
            callback = self._safety_mode_callback,
            qos_profile = UR_QOS_PROFILE
        )

        # TODO: callback group for services???

        self.initialize_service(URService.DashboardClient.SRV_UNLOCK_PROTECTIVE_STOP)
        self.initialize_service(URService.DashboardClient.SRV_RESTART_SAFETY)
        self.initialize_service(URService.DashboardClient.SRV_POWER_ON)
        self.initialize_service(URService.DashboardClient.SRV_POWER_OFF)
        self.initialize_service(URService.DashboardClient.SRV_BRAKE_RELEASE)
        self.initialize_service(URService.DashboardClient.SRV_PLAY)

        # Get current robot and safety modes
        self._current_robot_mode: RobotMode = self.call_service(
            URService.DashboardClient.SRV_GET_ROBOT_MODE,
            request = URService.get_service_type(URService.DashboardClient.SRV_GET_ROBOT_MODE).Request()
        )

        assert self._current_robot_mode is not None, "Could not determine the current robot mode"

        self._current_safety_mode: RobotMode = self.call_service(
            URService.DashboardClient.SRV_GET_SAFETY_MODE,
            request = URService.get_service_type(URService.DashboardClient.SRV_GET_SAFETY_MODE).Request()
        )

        assert self._current_safety_mode is not None, "Could not determine the current safety mode"

    @property
    def _is_started(self):
        return self._set_mode_action_server is not None

    def _start_set_mode_action_server(self):
        self._set_mode_action_server = ActionServer(
            node = self._node,
            action_type = SetMode,
            action_name = "set_mode",
            execute_callback = self._set_mode_execute_callback,
            goal_callback = self._set_mode_goal_callback,
            cancel_callback = self._set_mode_cancel_callback,
            handle_accepted_callback = self._set_mode_accept_callback
        )
        
    def _set_mode_goal_callback(self, _):
        return GoalResponse.ACCEPT

    def _set_mode_cancel_callback(self, _):
        return CancelResponse.REJECT

    def _set_mode_execute_callback(self, goal_handle: ServerGoalHandle):
        self._set_mode_execute(goal_handle)
        return self._result

    def _set_mode_accept_callback(self, goal_handle: ServerGoalHandle):
        # TODO: dispatch thread?
        self._set_mode_execute(goal_handle)

    def _set_mode_execute(self, goal_handle: ServerGoalHandle):
        self._in_action = True
        self._current_goal_handle = goal_handle
        self._goal: SetMode.Goal = goal_handle.request

        if self._is_illegal_mode(self._goal.target_robot_mode):
            self._result.message = "Requested illegal mode"
            self._result.success = False
            self._node.get_logger().error(f"Target mode illegal: {self._goal.target_robot_mode}")
            self._current_goal_handle.abort()
        else:
            self._node.get_logger().info(f"Target mode: {self._goal.target_robot_mode}")
            _mode = RobotMode(mode = self._goal.target_robot_mode)
            if _mode == RobotMode.POWER_OFF \
                or _mode == RobotMode.IDLE \
                or _mode == RobotMode.RUNNING:
                    if self._current_robot_mode != _mode or self._current_safety_mode > SafetyMode.REDUCED:
                        if self._goal.stop_program:
                            self._node.get_logger().info(f"Stop request: {self.call_service(URService.DashboardClient.SRV_STOP)}")
                        self._do_transition()
                    else:
                        self._update_robot_state()
            elif _mode == RobotMode.NO_CONTROLLER \
                or _mode == RobotMode.DISCONNECTED \
                or _mode == RobotMode.CONFIRM_SAFETY \
                or _mode == RobotMode.BOOTING \
                or _mode == RobotMode.BACKDRIVE \
                or _mode == RobotMode.UPDATING_FIRMWARE:
                    self._result.message = f"Selected target mode {_mode} that cannot be explicitly set"
                    self._result.success = False
                    self._node.get_logger().error(f"Selected target mode {_mode} that cannot be explicitly set")
                    self._current_goal_handle.abort()
            else:
                self._result.message = f"Illegal mode: {_mode}"
                self._result.success = False
                self._node.get_logger().error(f"Illegal mode: {_mode}")
                self._current_goal_handle.abort()

    def _robot_mode_callback(self, msg: RobotMode):
        if self._current_robot_mode != msg:
            self._node.get_logger().info(f"Robot mode {self._current_robot_mode} -> {msg}")
            self._current_robot_mode = msg

            if self._in_action:
                self._update_robot_state()
                if not self._is_started:
                    self._start_set_mode_action_server()

    def _safety_mode_callback(self, msg: SafetyMode):
        if self._current_safety_mode != msg:
            self._node.get_logger().info(f"Safety mode {self._current_safety_mode} -> {msg}")
            self._current_safety_mode = msg

            self._update_robot_state()
            if not self._is_started:
                self._start_set_mode_action_server()

    def _update_robot_state(self):
        if self._is_started:
            self._feedback.current_robot_mode = self._current_robot_mode.mode
            self._feedback.current_safety_mode = self._current_safety_mode.mode
            
            assert self._current_goal_handle is not None, "Null goal handle"

            self._current_goal_handle.publish_feedback(self._feedback)

            if self._current_robot_mode.mode < self._goal.target_robot_mode \
                or self._current_safety_mode.mode > SafetyMode.REDUCED:
                    self._node.get_logger().debug(f"Current mode: {self._current_robot_mode.mode}, target mode: {self._goal.target_robot_mode}")
                    self._do_transition()
            elif self._current_robot_mode.mode == self._goal.target_robot_mode:
                self._in_action = False
                self._result.success = True
                self._result.message = "Reached target robot mode"
                self._node.get_logger().info("Reached target robot mode")

                if self._current_robot_mode == RobotMode.RUNNING \
                    and self._goal.target_robot_mode == self._goal.play_program:
                    time.sleep(1)
                    self.call_service(URService.DashboardClient.SRV_PLAY)
                
                self._current_goal_handle.succeed()
            else:
                self._result.success = False
                self._result.message = f"Robot reached higher mode {self._current_robot_mode} than requested {self._goal.target_robot_mode} during recovery"
                self._node.get_logger().error(f"Robot reached higher mode {self._current_robot_mode} than requested {self._goal.target_robot_mode} during recovery")
                self._current_goal_handle.abort()

    def _do_transition(self):
        if self._goal.target_robot_mode < self._current_robot_mode.mode:
            self._node.get_logger().error(f"Target mode lower {self._goal.target_robot_mode} than current mode {self._current_robot_mode.mode}, powering off...")
            self._node.get_logger().info(f"Power off response: {self.call_service(URService.DashboardClient.SRV_POWER_OFF)}")
        else:
            if self._current_safety_mode == SafetyMode.PROTECTIVE_STOP:
                self._node.get_logger().info(f"Unlock protective stop response: {self.call_service(URService.DashboardClient.SRV_UNLOCK_PROTECTIVE_STOP)}")
            elif self._current_safety_mode == SafetyMode.SYSTEM_EMERGENCY_STOP or self._current_safety_mode == SafetyMode.ROBOT_EMERGENCY_STOP:
                self._node.get_logger().warning(f"The robot is currently in safety mode {self._current_safety_mode}, release EM-STOP / clear error")
            elif self._current_safety_mode == SafetyMode.VIOLATION or self._current_safety_mode == SafetyMode.FAULT:
                self._node.get_logger().info(f"Restart safety response: {self.call_service(URService.DashboardClient.SRV_RESTART_SAFETY)}")
            else:
                if self._current_robot_mode == RobotMode.CONFIRM_SAFETY:
                    self._node.get_logger().warning(f"The robot is currently in mode {self._current_robot_mode}, you must interact with the pendant")
                elif self._current_robot_mode == RobotMode.BOOTING:
                    self._node.get_logger().info(f"The robot is currently in mode {self._current_robot_mode}, wait until it is booted")
                elif self._current_robot_mode == RobotMode.POWER_OFF:
                    self._node.get_logger().info(f"Power off service response: {self.call_service(URService.DashboardClient.SRV_POWER_OFF)}")
                elif self._current_robot_mode == RobotMode.POWER_ON:
                    self._node.get_logger().info(f"The robot is currently in mode {self._current_robot_mode}, wait until robot is in IDLE mode")
                elif self._current_robot_mode == RobotMode.BACKDRIVE:
                    self._node.get_logger().info(f"The robot is currently in mode {self._current_robot_mode}, it will return to IDLE mode in the next state")
                elif self._current_robot_mode == RobotMode.RUNNING:
                    self._node.get_logger().info(f"The robot is operational (mode {self._current_robot_mode})")
                else:
                    self._node.get_logger().warning(f"The robot is currently in mode {self._current_robot_mode}. There are no configured actions for this state.")

    def _is_illegal_mode(self, mode: int) -> bool:
        return mode > 8 or mode < -1