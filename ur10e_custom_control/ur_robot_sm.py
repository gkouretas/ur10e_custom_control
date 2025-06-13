import time

from rclpy.node import Node
from ur10e_custom_control.ur10e_typedefs import URService
from ur10e_custom_control.ur_robot_node import URRobot
from rclpy.subscription import Subscription
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle

from std_msgs.msg import Bool
from ur_dashboard_msgs.msg import RobotMode, SafetyMode
from ur_dashboard_msgs.action import SetMode
from ur10e_custom_control.ur10e_configs import UR_QOS_PROFILE

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
        self._current_robot_mode = RobotMode(mode=RobotMode.NO_CONTROLLER)
        self._current_safety_mode = SafetyMode(mode=SafetyMode.UNDEFINED_SAFETY_MODE)
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

        self._program_running: bool = False

        self._program_running_sub = self._node.create_subscription(
            msg_type=Bool,
            topic="io_and_status_controller/robot_program_running", 
            qos_profile=UR_QOS_PROFILE,
            callback=self._update_program_running)

        # TODO: callback group for services???

        self.initialize_service(URService.DashboardClient.SRV_UNLOCK_PROTECTIVE_STOP)
        self.initialize_service(URService.DashboardClient.SRV_RESTART_SAFETY)
        self.initialize_service(URService.DashboardClient.SRV_POWER_ON)
        self.initialize_service(URService.DashboardClient.SRV_POWER_OFF)
        self.initialize_service(URService.DashboardClient.SRV_BRAKE_RELEASE)
        self.initialize_service(URService.DashboardClient.SRV_PLAY)

        # Get current robot and safety modes
        # if mode := self.call_service(
        #     URService.DashboardClient.SRV_GET_ROBOT_MODE,
        #     request = URService.get_service_type(URService.DashboardClient.SRV_GET_ROBOT_MODE).Request()
        # ):
        #     self._current_robot_mode = mode.robot_mode
        # else:
        #     raise RuntimeError("Could not determine the current robot mode")

        # if mode := self.call_service(
        #     URService.DashboardClient.SRV_GET_SAFETY_MODE,
        #     request = URService.get_service_type(URService.DashboardClient.SRV_GET_SAFETY_MODE).Request()
        # ):
        #     self._current_safety_mode = mode.safety_mode
        # else:
        #     raise RuntimeError("Could not determine the current safety mode")

    @property
    def _is_started(self):
        return self._set_mode_action_server is not None
    
    @property
    def current_mode(self) -> RobotMode:
        return self._current_robot_mode
    
    @property
    def current_safety_mode(self) -> SafetyMode:
        return self._current_safety_mode
    
    @property
    def program_running(self) -> bool:
        return self._program_running

    def _update_program_running(self, msg: Bool):
        self._program_running = msg.data

    def _start_set_mode_action_server(self):
        self._set_mode_action_server = ActionServer(
            node = self._node,
            action_type = SetMode,
            action_name = "~/set_mode",
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
        goal_handle.execute()

    def _set_mode_execute(self, goal_handle: ServerGoalHandle):
        self._in_action = True
        self._current_goal_handle = goal_handle
        self._goal: SetMode.Goal = goal_handle.request
        self._node.get_logger().info(f"Status: {goal_handle.status}")

        if self._is_illegal_mode(self._goal.target_robot_mode):
            self._result.message = "Requested illegal mode"
            self._result.success = False
            self._node.get_logger().error(f"Target mode illegal: {self._goal.target_robot_mode}")
            self._current_goal_handle.abort()
        else:
            self._node.get_logger().info(f"Target mode: {self._goal.target_robot_mode}")
            _mode = self._goal.target_robot_mode
            if _mode == RobotMode.POWER_OFF \
                or _mode == RobotMode.IDLE \
                or _mode == RobotMode.RUNNING:
                    if self._current_robot_mode != _mode or self._current_safety_mode > SafetyMode.REDUCED:
                        if self._goal.stop_program:
                            self._node.get_logger().info(f"Stop request: {self.call_service(URService.DashboardClient.SRV_STOP)}")
                        if not self._do_transition():
                            self._result.message = f"Transition to target mode {_mode} failed"
                            self._result.success = False
                            self._node.get_logger().error(f"Transition to target mode {_mode} failed")
                            self._current_goal_handle.abort()
                            self._node.get_logger().info("here 1...")
                            return
                    
                        # self._update_robot_state()
            elif _mode == RobotMode.NO_CONTROLLER \
                or _mode == RobotMode.DISCONNECTED \
                or _mode == RobotMode.CONFIRM_SAFETY \
                or _mode == RobotMode.BOOTING \
                or _mode == RobotMode.BACKDRIVE \
                or _mode == RobotMode.UPDATING_FIRMWARE:
                    self._result.message = f"Selected target mode {_mode} that cannot be explicitly set"
                    self._result.success = False
                    self._node.get_logger().error(f"Selected target mode {_mode} that cannot be explicitly set")
                    self._node.get_logger().info("here 2...")
                    self._current_goal_handle.abort()
                    return
            else:
                self._result.message = f"Illegal mode: {_mode}"
                self._result.success = False
                self._node.get_logger().error(f"Illegal mode: {_mode}")
                self._current_goal_handle.abort()
                return

        while self._current_robot_mode.mode != self._goal.target_robot_mode:
            time.sleep(0.5)

        self._update_robot_state()

    def _robot_mode_callback(self, msg: RobotMode):
        if self._current_robot_mode != msg:
            self._node.get_logger().info(f"Robot mode {self._current_robot_mode} -> {msg}")
            self._current_robot_mode = msg

            if self._in_action:
                #self._update_robot_state()
                if not self._is_started:
                    self._start_set_mode_action_server()

    def _safety_mode_callback(self, msg: SafetyMode):
        if self._current_safety_mode != msg:
            self._node.get_logger().info(f"Safety mode {self._current_safety_mode} -> {msg}")
            self._current_safety_mode = msg

            #self._update_robot_state()
            if not self._is_started:
                self._start_set_mode_action_server()

    def _update_robot_state(self):
        if self._is_started:
            # self._feedback.current_robot_mode = self._current_robot_mode.mode
            # self._feedback.current_safety_mode = self._current_safety_mode.mode
            
            # assert self._current_goal_handle is not None, "Null goal handle"

            # self._current_goal_handle.publish_feedback(self._feedback)

            # if self._current_robot_mode.mode < self._goal.target_robot_mode \
            #     or self._current_safety_mode.mode > SafetyMode.REDUCED:
            #         self._node.get_logger().debug(f"Current mode: {self._current_robot_mode.mode}, target mode: {self._goal.target_robot_mode}")
            #         self._do_transition()
            if self._current_robot_mode.mode == self._goal.target_robot_mode:
                self._in_action = False
                self._result.success = True
                self._result.message = "Reached target robot mode"
                self._node.get_logger().info("Reached target robot mode")

                if self._current_robot_mode.mode == RobotMode.RUNNING \
                    and self._goal.play_program:

                    
                    # TODO(george): do this or below based upon if headless
                    if result := self.call_service(URService.IOAndStatusController.SRV_RESEND_ROBOT_PROGRAM):
                        self._result.success = result.success

                    # time.sleep(1)
                    # self.call_service(URService.DashboardClient.SRV_PLAY)

                if self._result.success:  
                    self._node.get_logger().info(f"Status: {self._current_goal_handle.status}")              
                    self._current_goal_handle.succeed()
                else:
                    self._current_goal_handle.abort()
            else:
                self._result.success = False
                self._result.message = f"Robot reached higher mode {self._current_robot_mode} than requested {self._goal.target_robot_mode} during recovery"
                self._node.get_logger().error(f"Robot reached higher mode {self._current_robot_mode} than requested {self._goal.target_robot_mode} during recovery")
                self._current_goal_handle.abort()

    def _target_to_srv(self, target: int):
        match target:
            case RobotMode.POWER_OFF:
                return URService.DashboardClient.SRV_POWER_OFF
            case RobotMode.IDLE:
                return URService.DashboardClient.SRV_POWER_ON
            case RobotMode.RUNNING:
                return URService.DashboardClient.SRV_BRAKE_RELEASE
            case _:
                self._node.get_logger().error(f"Invalid target for service ({target})")
                return None

    def _do_transition(self):
        if self._current_safety_mode == SafetyMode.PROTECTIVE_STOP:
            self._node.get_logger().info(f"Unlock protective stop response: {self.call_service(URService.DashboardClient.SRV_UNLOCK_PROTECTIVE_STOP)}")
        elif self._current_safety_mode == SafetyMode.SYSTEM_EMERGENCY_STOP or self._current_safety_mode == SafetyMode.ROBOT_EMERGENCY_STOP:
            self._node.get_logger().warning(f"The robot is currently in safety mode {self._current_safety_mode}, release EM-STOP / clear error")
        elif self._current_safety_mode == SafetyMode.VIOLATION or self._current_safety_mode == SafetyMode.FAULT:
            self._node.get_logger().info(f"Restart safety response: {self.call_service(URService.DashboardClient.SRV_RESTART_SAFETY)}")
        else:
            match self._current_robot_mode.mode:
                case RobotMode.CONFIRM_SAFETY:
                    self._node.get_logger().warning(f"The robot is currently in mode {self._current_robot_mode}, you must interact with the pendant")
                case RobotMode.BOOTING:
                    self._node.get_logger().info(f"The robot is currently in mode {self._current_robot_mode}, wait until it is booted")
                case RobotMode.POWER_ON:
                    self._node.get_logger().info(f"The robot is currently in mode {self._current_robot_mode}, wait until robot is in IDLE mode")
                case RobotMode.BACKDRIVE:
                    self._node.get_logger().info(f"The robot is currently in mode {self._current_robot_mode}, it will return to IDLE mode in the next state")
                case RobotMode.POWER_OFF | RobotMode.IDLE:
                    if service := self._target_to_srv(self._goal.target_robot_mode):
                        return self.call_service(service).success
                    else:
                        return False
                case RobotMode.RUNNING:
                    if self._goal.target_robot_mode == RobotMode.IDLE:
                        if not self.call_service(URService.DashboardClient.SRV_POWER_OFF).success:
                            self._node.get_logger().error("Failed to power off")
                            return False
                        
                    if service := self._target_to_srv(self._goal.target_robot_mode):
                        self._node.get_logger().info("Here")
                        return self.call_service(service).success
                    else:
                        return False
                case _:
                    self._node.get_logger().warning(f"The robot is currently in mode {self._current_robot_mode}. There are no configured actions for this state.")

        return False

    def _is_illegal_mode(self, mode: int) -> bool:
        return mode > 8 or mode < -1