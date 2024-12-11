from enum import Enum
from typing import Optional

from rclpy.node import Node
from rclpy.client import Client, SrvTypeRequest

from std_srvs.srv import Trigger

from controller_manager_msgs.srv import ListControllers, SwitchController
from ur_dashboard_msgs.srv import (
    GetLoadedProgram,
    GetProgramState,
    GetRobotMode,
    GetSafetyMode,
    IsProgramRunning,
    Load,
)
from ur_msgs.srv import SetIO, GetRobotSoftwareVersion, SetSpeedSliderFraction

# Control mode types
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg._float64_multi_array import Float64MultiArray

class URService:
    # TODO: getter for srv
    class DashboardClient(str, Enum):
        __namespace = "/dashboard_client"
        
        SRV_POWER_ON = __namespace + "/power_on"
        SRV_POWER_OFF = __namespace + "/power_off"
        SRV_BRAKE_RELEASE = __namespace + "/brake_release"
        SRV_UNLOCK_PROTECTIVE_STOP = __namespace + "/unlock_protective_stop"
        SRV_RESTART_SAFETY = __namespace + "/restart_safety"
        SRV_GET_ROBOT_MODE = __namespace + "/get_robot_mode"
        SRV_GET_SAFETY_MODE = __namespace + "/get_safety_mode"
        SRV_LOAD_INSTALLATION = __namespace + "/load_installation"
        SRV_LOAD_PROGRAM = __namespace + "/load_program"
        SRV_CLOSE_POPUP = __namespace + "/close_popup"
        SRV_GET_LOADED_PROGRAM = __namespace + "/get_loaded_program"
        SRV_PROGRAM_STATE = __namespace + "/program_state"
        SRV_PROGRAM_RUNNING = __namespace + "/program_running"
        SRV_PLAY = __namespace + "/play"
        SRV_STOP = __namespace + "/stop"

    class ControllerManager(str, Enum):
        __namespace = "/controller_manager"

        SRV_SWITCH_CONTROLLER = __namespace + "/switch_controller"
        SRV_LIST_CONTROLLERS = __namespace + "/list_controllers"
    
    class IOAndStatusController(str, Enum):
        __namespace = "/io_and_status_controller"

        SRV_SET_IO = __namespace + "/set_io"
        SRV_RESEND_ROBOT_PROGRAM = __namespace + "/resend_robot_program"
        SRV_SET_SPEED_SLIDER_FRACTION = __namespace + "/set_speed_slider"
    
    class URConfigurationController(str, Enum):
        __namespace = "/ur_configuration_controller"

        SRV_GET_ROBOT_SOFTWARE_VERSION = __namespace + "/get_robot_software_version"

    URServiceType = DashboardClient | ControllerManager | IOAndStatusController | URConfigurationController

    _UR_SERVICE_MAP: dict[str, SrvTypeRequest] = {
        DashboardClient.SRV_POWER_ON: Trigger,
        DashboardClient.SRV_POWER_OFF: Trigger,
        DashboardClient.SRV_BRAKE_RELEASE: Trigger,
        DashboardClient.SRV_UNLOCK_PROTECTIVE_STOP: Trigger,
        DashboardClient.SRV_RESTART_SAFETY: Trigger,
        DashboardClient.SRV_GET_ROBOT_MODE: GetRobotMode,
        DashboardClient.SRV_GET_SAFETY_MODE: GetSafetyMode,
        DashboardClient.SRV_LOAD_INSTALLATION: Load,
        DashboardClient.SRV_CLOSE_POPUP: Trigger,
        DashboardClient.SRV_GET_LOADED_PROGRAM: GetLoadedProgram,
        DashboardClient.SRV_PROGRAM_STATE: GetProgramState,
        DashboardClient.SRV_PROGRAM_RUNNING: IsProgramRunning,
        DashboardClient.SRV_PLAY: Trigger,
        DashboardClient.SRV_STOP: Trigger,
        ControllerManager.SRV_SWITCH_CONTROLLER: SwitchController,
        ControllerManager.SRV_LIST_CONTROLLERS: ListControllers,
        IOAndStatusController.SRV_SET_IO: SetIO,
        IOAndStatusController.SRV_RESEND_ROBOT_PROGRAM: Trigger,
        IOAndStatusController.SRV_SET_SPEED_SLIDER_FRACTION: SetSpeedSliderFraction,
        URConfigurationController.SRV_GET_ROBOT_SOFTWARE_VERSION: GetRobotSoftwareVersion
    }

    URServices: tuple[URServiceType] = tuple(_UR_SERVICE_MAP.keys())

    @classmethod
    def get_service_type(cls, service: URServiceType) -> Optional[SrvTypeRequest]:
        return cls._UR_SERVICE_MAP.get(service)

    @classmethod
    def init_service(cls, node: Node, service: URServiceType, timeout: float) -> Client:
        ur_service_type = cls.get_service_type(service)
        if ur_service_type is None:
            raise TypeError(f"{service} is not a valid service")
        
        client = node.create_client(ur_service_type, service.value)
        if not client.wait_for_service(timeout):
            raise TimeoutError(f"Timed out waiting for {service} to connect")
        
        return client
    
class URControlModes(str, Enum):
    JOINT_TRAJECTORY = "joint_trajectory_controller"
    SCALED_JOINT_TRAJECTORY = "scaled_joint_trajectory_controller"
    FORWARD_VELOCITY = "forward_velocity_controller"
    FORWARD_POSITION = "forward_position_controller"

    @property
    def has_action_client(self) -> bool:
        return self.value == self.JOINT_TRAJECTORY or self.value == self.SCALED_JOINT_TRAJECTORY

    @property
    def action_type(self) -> type:
        if self.value == self.JOINT_TRAJECTORY or self.value == self.SCALED_JOINT_TRAJECTORY:
            return FollowJointTrajectory
        else:
            return None

    @property
    def action_type_topic(self) -> str:
        if self.value == self.JOINT_TRAJECTORY or self.value == self.SCALED_JOINT_TRAJECTORY:
            return self.value + "/follow_joint_trajectory"
        else:
            raise None
        
    @property
    def publish_topic(self) -> type:
        if self.value == self.FORWARD_VELOCITY or self.value == self.FORWARD_POSITION:
            return Float64MultiArray
        else:
            return None
        
    @property
    def publish_topic_name(self) -> str:
        if self.value == self.FORWARD_VELOCITY or self.value == self.FORWARD_POSITION:
            return "/" + self.value + "/commands"
        else:
            return None
        
    @property
    def is_cyclic(self) -> bool:
        return self.value == self.FORWARD_VELOCITY or self.value == self.FORWARD_POSITION
