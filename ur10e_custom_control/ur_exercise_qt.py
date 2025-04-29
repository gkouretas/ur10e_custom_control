import sys
import threading
import pickle
import os
import numpy as np
import transforms3d
import tf2_ros

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.clock import Clock
from rclpy.time import Time as _Time

from PyQt5.QtWidgets import *
from PyQt5.QtCore import QTimer, QThread

from ur_control_qt import URControlQtWindow
from ur10e_configs import UR_QOS_PROFILE
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration, Time
from tf2_ros import TransformBroadcaster
from std_srvs.srv import Trigger
from exercise_decoder_node.exercise_decoder_node_configs import (
    MINDROVE_ACTIVATION_SERVICE,
    MINDROVE_DEACTIVATION_SERVICE
)
from ur_dashboard_msgs.msg import RobotMode
from ur_dashboard_msgs.action import SetMode
from ur_msgs.srv import SetIO

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from visualization_msgs.msg import Marker

from geometry_msgs.msg import Transform, TransformStamped, Vector3, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from ur10e_typedefs import URService

from typing import Callable
from functools import partial

from dataclasses import dataclass, asdict
from python_utils.ros2_utils.comms.rosbag_manager import RosbagManager
from python_utils.utils.datetime_utils import postfix_string_with_current_time

from ros2_mindrove.mindrove_configs import MINDROVE_ROS_TOPIC_NAME
from ros2_plux_biosignals.plux_configs import PLUX_ROS_TOPIC_NAME
from ros2_mindrove.mindrove_configs import MINDROVE_ROS_TOPIC_NAME
from pi_user_input_node.user_input_node_config import USER_INPUT_TOPIC_NAME
from fatigue_classifier.fatigue_classifier_configs import FATIGUE_OUTPUT_TOPIC_NAME

_MOVE_CAMERA_WITH_EXERCISE = True
_ALIGN_WITH_PATH = False
_HANDEDNESS = "right"

# For now tform is the same, but may want to offset left/right to align with user perspective
# Orientation = rotx(180)

def pose_to_ndarray(pose: Pose):
    rotmat = transforms3d.quaternions.quat2mat([
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z
    ])

    return np.array([
        [rotmat[0,0], rotmat[0,1], rotmat[0,2], pose.position.x],
        [rotmat[1,0], rotmat[1,1], rotmat[1,2], pose.position.y],
        [rotmat[2,0], rotmat[2,1], rotmat[2,2], pose.position.z],
        [0, 0, 0, 1]
    ]) 

if _HANDEDNESS == "right":
    _CONSTANT_TRANSFORM = pose_to_ndarray(
        Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(w=0.0, x=0.0, y=1.0, z=0.0)
        )
    )
else:
    _CONSTANT_TRANSFORM = pose_to_ndarray(
        Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(w=0.0, x=0.0, y=1.0, z=0.0)
        )
    )

UR_TOPICS = [
    "/io_and_status_controller/io_states",
    "/tcp_pose_broadcaster/pose",
    "/force_torque_sensor_broadcast/wrench",
    "/joint_states"
]

VISUALIZATION_TOPICS = [
    "/dynamic_force_target_pose", # For visualization
    "/tf"
]

SENSOR_TOPICS = [
    MINDROVE_ROS_TOPIC_NAME,
    PLUX_ROS_TOPIC_NAME,
    FATIGUE_OUTPUT_TOPIC_NAME,
    USER_INPUT_TOPIC_NAME,
]

LOGGED_TOPICS = set(UR_TOPICS + VISUALIZATION_TOPICS + SENSOR_TOPICS)

# TODO: make config or move elsewhere
_DISTANCE_THRESHOLD = 30/1000
_LOG_DATA = False

@dataclass
class Exercise:
    name: str
    poses: list[PoseStamped]
    joint_angles: list[list[float]]
    duration: list[Duration]

class RclpySpinner(QThread):
    def __init__(self, nodes: list[Node]):
        super().__init__()
        self._primary_node = nodes[0]
        self._nodes = nodes
        self._abort = False

    def run(self):
        self._primary_node.get_logger().info('Start called on RclpySpinner, spinning ros2 node')
        from rclpy.executors import MultiThreadedExecutor

        executor = MultiThreadedExecutor()
        for node in self._nodes:
            executor.add_node(node)
        while rclpy.ok() and not self._abort:
            executor.spin_once(timeout_sec=1.0)

    def quit(self):  # noqa: A003
        self._primary_node.get_logger().info('Quit called on RclpySpinner')
        self._abort = True
        super().quit()

class URExerciseControlWindow(URControlQtWindow):
    def __init__(self, node: Node):
        super().__init__(node)

        self.service_tab = self._create_tab(name = "Exercise Tab", layout = QVBoxLayout(), tab_create_func = self.__conf_exercise_tab)
        self._state_lock = threading.RLock()
        self._clock = Clock()

        self._pose_reception_counter = 0
        self._joint_angle_reception_counter = 0
        self._exercise_traj_poses: list[PoseStamped] = []
        self._exercise_traj_joint_angles: list[list[float]] = []
        self._exercise_traj_time: list[Duration] = []
        self._exercise_traj_ready: bool = False
        self._current_pose: PoseStamped = None

        self._activate_mindrove_service = self._node.create_client(
            Trigger, 
            MINDROVE_ACTIVATION_SERVICE
        )

        self._deactivate_mindrove_service = self._node.create_client(
            Trigger, 
            MINDROVE_DEACTIVATION_SERVICE
        )

        self._fps_signal = threading.Event()

        self._target_pose_publisher = self._node.create_publisher(PoseStamped, "dynamic_force_target_pose", 0)
        self._path_publisher = self._node.create_publisher(Path, "dynamic_force_path", 0)
        self._error_vector_publisher = self._node.create_publisher(Marker, "dynamic_force_error_vector", 0)
        self._rviz_camera_tform_publisher = TransformBroadcaster(self._node, 0)

        assert self.create_subscriber(msg_type = JointState, topic = "/joint_states", callback = self._update_joint_angles, qos_profile = UR_QOS_PROFILE), \
            "Unable to create joint position subscriber"
        assert self.create_subscriber(msg_type = PoseStamped, topic = "tcp_pose_broadcaster/pose", callback = self._update_pose, qos_profile = UR_QOS_PROFILE), \
            "Unable to create TCP pose broadcaster subscriber"

        self._exercise_freedrive_timer = QTimer(self)
        self._exercise_freedrive_timer.timeout.connect(self._robot.ping_freedrive)
        self._rosbag_manager = RosbagManager(node=self._node, topics=LOGGED_TOPICS, monitor_frequency_hz=1.0)
        self._follow_eef = threading.Event()

        self._fps_timer = QTimer()
        self._fps_timer.timeout.connect(lambda: self._fps_signal.set())
        self._fps_timer.setInterval(16) # 60 FPS
        self._fps_timer.start()

        # if not _MOVE_CAMERA_WITH_EXERCISE:
        #     self._node.get_logger().info("Adding interactive marker")

        #     int_marker = InteractiveMarker(
        #         header=Header(frame_id="base"),
        #         name="user_location",
        #         pose=Pose(position=Point(x=0.0,y=0.0,z=0.0), orientation=Quaternion(w=1.0,x=0.0,y=0.0,z=0.0))
        #     )

        #     # Create the visual part of the marker (a sphere representing a point)
        #     marker = Marker(
        #         type=Marker.SPHERE,
        #         scale=Vector3(x=0.1,y=0.1,z=0.1),
        #         color=ColorRGBA(r=1.0,g=1.0,b=1.0,a=0.5),
        #         frame_locked=False
        #     )

        #     int_marker.controls.append(
        #         InteractiveMarkerControl(
        #             orientation_mode=InteractiveMarkerControl.MOVE_ROTATE_3D, 
        #             interaction_mode=InteractiveMarkerControl.MOVE_ROTATE_3D, 
        #             markers=[marker]
        #         )
        #     )

        #     self._interactive_marker_server = InteractiveMarkerServer(self._node, namespace="marker")

        #     # Add the interactive marker to the server
        #     self._interactive_marker_server.insert(int_marker)

        #     # Commit changes to the server
        #     self._interactive_marker_server.applyChanges()

        #     self._node.get_logger().info("Interactive marker added")


    def _get_trajectory(self, poses: list[PoseStamped], distance_threshold: float = _DISTANCE_THRESHOLD) -> list[PoseStamped]:
        def __dist3d(_t1: PoseStamped, _t2: PoseStamped) -> float:
            return ((_t1.pose.position.x-_t2.pose.position.x)**2 + \
                    (_t1.pose.position.y-_t2.pose.position.y)**2 + \
                    (_t1.pose.position.z-_t2.pose.position.z)**2)**0.5
        
        last_pose: PoseStamped = poses[0]
        trajectory: list[PoseStamped] = [] # Do not include initial pose in the trajectory
        for pose in poses:
            if __dist3d(last_pose, pose) >= distance_threshold:
                last_pose = pose
                trajectory.append(pose)

        return trajectory
    
    def __conf_exercise_tab(self, layout: QLayout) -> None:
        _run_loop = False

        def _start_robot_program(_):
            if self._robot is None: return

            self._robot.request_mode(
                SetMode.Goal(
                    target_robot_mode=RobotMode.RUNNING,
                    play_program=True,
                    stop_program=False
                ),
                blocking=True
            )

            # Set tool voltage to 12V
            self._robot.call_service(
                URService.IOAndStatusController.SRV_SET_IO,
                request=SetIO.Request(fun=SetIO.Request.FUN_SET_TOOL_VOLTAGE, 
                                      state=float(SetIO.Request.STATE_TOOL_VOLTAGE_12V))
            )

        def _stop_robot_program(_):
            if self._robot is None: return

            # Power the tool off
            self._robot.call_service(
                URService.IOAndStatusController.SRV_SET_IO,
                request=SetIO.Request(fun=SetIO.Request.FUN_SET_TOOL_VOLTAGE, 
                                      state=float(SetIO.Request.STATE_TOOL_VOLTAGE_0V))
            )

            self._robot.request_mode(
                SetMode.Goal(
                    target_robot_mode=RobotMode.POWER_OFF,
                    play_program=False,
                    stop_program=False
                ),
                blocking=True
            ) 

        def _start_freedrive_exercise_trajectory(_):
            # TODO: add ability to restrict DOF's
            self._robot.run_freedrive_control()
            self._exercise_freedrive_timer.start(100)

            self._state_lock.acquire()
            self._exercise_traj_poses.clear()
            self._exercise_traj_time.clear()
            self._pose_reception_counter = 0
            self._joint_angle_reception_counter = 0
            self._state_lock.release()

        def _stop_freedrive_exercise_trajectory(_):
            # TODO: this crashes the program, idk why
            # Remove subscriber since we don't need it
            # self.remove_subscriber("/joint_states")

            # Subtract initial timestamp
            self._state_lock.acquire()
            self._pose_reception_counter = -1
            self._joint_angle_reception_counter = -1
            self._exercise_traj_ready = True
            self._state_lock.release()

            self._robot.stop_freedrive_control()
            self._exercise_freedrive_timer.stop()

            self._node.get_logger().debug("Trajectory")
            for w, d in zip(self._exercise_traj_poses, self._exercise_traj_time):
                self._node.get_logger().debug(f"Duration: {d.sec + d.nanosec*1e-9}s, pos: {w}")

        def _move_to_start(_):
            # 10s trajectory to get to home pose
            self._robot.send_trajectory(
                waypts = [self._exercise_traj_joint_angles[0]],
                time_vec = [Duration(sec = 10)],
                blocking = False
            )

        def _run_exercise_trajectory(_):
            if self._robot is None: return
            if len(self._exercise_traj_joint_angles) > 1:
                self._robot.send_trajectory(
                    waypts = self._exercise_traj_joint_angles[1:], 
                    time_vec = self._exercise_traj_time[1:],
                    blocking = False
                )
            else:
                self._node.get_logger().warning("No waypoints configured")

        def _toggle_follow_end_effector(_):
            if self._follow_eef.is_set(): self._follow_eef.clear()
            else: self._follow_eef.set()

        def _run_dynamic_force_mode(_):
            from ur_msgs.action._dynamic_force_mode_path import DynamicForceModePath_FeedbackMessage
            def dynamic_force_mode_feedback(feedback: DynamicForceModePath_FeedbackMessage):
                if _MOVE_CAMERA_WITH_EXERCISE:  
                    if _ALIGN_WITH_PATH:              
                        t1 = feedback.feedback.pose_actual
                        t2 = feedback.feedback.pose_desired
                        
                        y = np.array([0.0, 1.0, 0.0])

                        x = np.array([
                            t1.position.x-t2.position.x,
                            t1.position.y-t2.position.y,
                            t1.position.z-t2.position.z
                        ])

                        # Align the frames along the x-axis, since this is the axis tracked by rviz's third-person follower
                        x /= np.linalg.norm(x)
                        
                        y -= (np.dot(x, y)*x)
                        y /= np.linalg.norm(y)
                        z = np.cross(x, y)

                        q_new = transforms3d.quaternions.mat2quat(
                            np.array([x, y, z]).T
                        )

                        self._compute_and_send_camera_frame(Transform(
                            translation = Vector3(
                                x=feedback.feedback.pose_actual.position.x,
                                y=feedback.feedback.pose_actual.position.y,
                                z=feedback.feedback.pose_actual.position.z
                            ),
                            rotation = Quaternion(w=q_new[0], x=q_new[1], y=q_new[2], z=q_new[3])
                        ))
                    else:
                        self._compute_and_send_camera_frame(feedback.feedback.pose_actual)
                # else:
                #     # Set the camera frame to be at the eef position with the orientation
                #     # aligned with the target frame
                #     if marker := self._interactive_marker_server.get("user_location"):
                #         marker: Marker
                #         eef_with_orientation = TransformStamped(
                #             header=Header(frame_id="base"),
                #             child_frame_id="tf_dynamic_path",
                #             transform=Transform(
                #                 translation = Vector3(
                #                     x=marker.pose.position.x,
                #                     y=marker.pose.position.y,
                #                     z=marker.pose.position.z
                #                 ),
                #                 rotation = marker.pose.orientation
                #             )
                #         )
                
                # TODO(george): this should probably just already be a PoseStamped object
                target_pose = PoseStamped(
                    header=Header(frame_id="base"),
                    pose=feedback.feedback.pose_desired,
                )

                error_vector = Marker(
                    header=Header(frame_id='base'),
                    type=Marker.ARROW,
                    action=Marker.ADD,
                    scale=Vector3(x=0.01, y=0.02, z=0.01),
                    points=[
                        Point(x=feedback.feedback.pose_actual.position.x,
                              y=feedback.feedback.pose_actual.position.y,
                              z=feedback.feedback.pose_actual.position.z),
                        Point(x=feedback.feedback.pose_desired.position.x,
                              y=feedback.feedback.pose_desired.position.y,
                              z=feedback.feedback.pose_desired.position.z)
                    ],
                    color=ColorRGBA(r=1.0,g=0.0,b=0.0,a=1.0)
                )

                self._target_pose_publisher.publish(target_pose)
                self._error_vector_publisher.publish(error_vector)
                
            if self._robot is None: 
                self._node.get_logger().error("System not initialized")
                return

            if _LOG_DATA and not self._rosbag_manager.cycle_logging(name=postfix_string_with_current_time("dynamic_force_feedback")):
                self._node.get_logger().error("Failed to cycle data logging")
                return

            if len(self._exercise_traj_joint_angles) > 1:
                self._follow_eef.clear()
                self._robot.set_action_feedback_callback(dynamic_force_mode_feedback)

                self._robot.run_dynamic_force_mode(
                    poses = self._get_trajectory(
                        self._exercise_traj_poses,
                    ), 
                    blocking = False
                )
            else:
                self._node.get_logger().warning("No waypoints configured")


        def _save_exercise_trajectory(_):
            if not self._exercise_traj_ready:
                self._node.get_logger().warning("No exercise is ready, nothing to save")
                return
            
            options = QFileDialog.Options()
            fp, _ = QFileDialog.getSaveFileName(
                self, 
                "Save exercise trajectory", 
                "", 
                "Exercise file (*.exercise)", 
                options = options
            )

            if fp:
                self._node.get_logger().info(f"Saving exercise to {fp}")

                # Clamp the length to the shortest list between the logged poses and joint angles, since they
                # may have a different # of elements.
                _min_length = min((len(self._exercise_traj_poses), len(self._exercise_traj_joint_angles)))
                
                _exercise = Exercise(
                    name = os.path.basename(fp).split(".")[0],
                    poses = self._exercise_traj_poses[:_min_length],
                    joint_angles = self._exercise_traj_joint_angles[:_min_length],
                    duration = self._exercise_traj_time[:_min_length]
                )

                with open(fp, "wb") as _pickled_file:
                    pickle.dump(asdict(_exercise), _pickled_file)
            else:
                self._node.get_logger().info("No output selected, not saving file")

        def _load_exercise_trajectory(_):
            options = QFileDialog.Options()
            fp, _ = QFileDialog.getOpenFileName(
                self, 
                "Select exercise trajectory", 
                "", 
                "Exercise file (*.exercise)", 
                options = options
            )

            if fp:
                self._node.get_logger().info(f"Getting exercise {fp}")

                with open(fp, "rb") as _pickled_file:
                    _obj = pickle.load(_pickled_file)
                    try:
                        _obj = Exercise(**_obj)
                    except Exception:
                        self._node.get_logger().error("Invalid exercise")
                    else:
                        self._node.get_logger().info(f"Loaded trajectory {_obj.name}")
                        with self._state_lock:
                            self._exercise_traj_joint_angles = _obj.joint_angles
                            self._exercise_traj_poses = _obj.poses
                            self._exercise_traj_time = _obj.duration

                            self._pose_reception_counter = -1
                            self._joint_angle_reception_counter = -1
                            self._exercise_traj_ready = True
                        
            else:
                self._node.get_logger().info("No input exercise selected")

        
        def _run_exercise_loop():
            reverse = False
            while True:
                if len(self._exercise_traj_poses) > 1:
                    self._robot.send_trajectory(
                        waypts = self._exercise_traj_joint_angles[1:] if not reverse else self._exercise_traj_joint_angles[::-1][1:], 
                        time_vec = self._exercise_traj_time[1:], # TODO: reverse time vector
                        blocking = True
                    )

                    reverse = not reverse
                else:
                    self._node.get_logger().warning("No waypoints configured")
                    break

                nonlocal _run_loop
                if not _run_loop: break

        def _loop_exercise_trajectory(_):
            _dialog = QDialog()
            _dialog.setLayout(QVBoxLayout())
            _dialog.layout().addWidget(QLabel("Close this window to stop trajectories"))

            nonlocal _run_loop
            _run_loop = True

            # Dispatch thread
            threading.Thread(target = _run_exercise_loop, daemon = True).start()

            # TODO: nonblocking popup???
            _dialog.exec_()

            # TODO: cleaner signaling to thread
            _run_loop = False

        def _activate_mindrove(_):
            self._node.get_logger().info(
                f"Activate mindrove service: {self._activate_mindrove_service.call(Trigger.Request())}"
            )

        def _deactivate_mindrove(_):
            self._node.get_logger().info(
                f"Deactivate mindrove service: {self._deactivate_mindrove_service.call(Trigger.Request())}"
            )

        def _publish_path_for_rviz(_):
            path = Path()
            path.header = Header(frame_id="base")
            path.poses = [x for x in self._get_trajectory(self._exercise_traj_poses)]

            self._path_publisher.publish(path)

        # def _preview_camera(_):
        #     if marker := self._interactive_marker_server.get("user_location"):
        #         marker: Marker
        #         eef_with_orientation = TransformStamped(
        #             header=Header(frame_id="base"),
        #             child_frame_id="tf_dynamic_path",
        #             transform=Transform(
        #                 translation = Vector3(
        #                     x=marker.pose.position.x,
        #                     y=marker.pose.position.y,
        #                     z=marker.pose.position.z
        #                 ),
        #                 rotation = marker.pose.orientation
        #             )
        #         )

        #         self._rviz_camera_tform_publisher.sendTransform(eef_with_orientation)

            # Launch tab
        _launch_map: dict[QPushButton, Callable] = {
            QPushButton("INITIALIZE ROBOT", self): _start_robot_program,
            QPushButton("SHUTDOWN ROBOT", self): _stop_robot_program,
            QPushButton("START FREEDRIVE EXERCISE", self): _start_freedrive_exercise_trajectory,
            QPushButton("STOP FREEDRIVE EXERCISE", self): _stop_freedrive_exercise_trajectory,
            QPushButton("SAVE TRAJECTORY", self): _save_exercise_trajectory,
            QPushButton("LOAD TRAJECTORY", self): _load_exercise_trajectory,
            QPushButton("MOVE TO START", self): _move_to_start,
            QPushButton("RUN DYNAMIC FORCE MODE", self): _run_dynamic_force_mode,
            QPushButton("PUBLISH PATH FOR RVIZ", self): _publish_path_for_rviz,
            QPushButton("TOGGLE FOLLOW EEF", self): _toggle_follow_end_effector

            # QPushButton("RUN EXERCISE TRAJECTORY", self): _run_exercise_trajectory,
            # QPushButton("LOOP EXERCISE TRAJECTORY", self): _loop_exercise_trajectory,
            # QPushButton("ACTIVATE MINDROVE", self): _activate_mindrove,
            # QPushButton("DEACTIVATE MINDROVE", self): _deactivate_mindrove,
        }

        # if not _MOVE_CAMERA_WITH_EXERCISE:
        #     _launch_map.update({QPushButton("PREVIEW_CAMERA", self): _preview_camera})

        for button, callback_func in _launch_map.items():
            button.clicked.connect(partial(callback_func, button))
            layout.addWidget(button)

    def _compute_and_send_camera_frame(self, frame: TransformStamped | Pose | PoseStamped):
        if not self._fps_signal.is_set(): return
        if _ALIGN_WITH_PATH:
            if isinstance(frame, TransformStamped): 
                transform = frame
            elif isinstance(frame, Pose): 
                transform = Transform(
                    translation = Vector3(
                        x=frame.position.x,
                        y=frame.position.y,
                        z=frame.position.z
                    ),
                    rotation = frame.orientation
                )
            elif isinstance(frame, PoseStamped):
                transform = Transform(
                    translation = Vector3(
                        x=frame.pose.position.x,
                        y=frame.pose.position.y,
                        z=frame.pose.position.z
                    ),
                    rotation = frame.pose.orientation
                )
            else:
                raise ValueError
        else:
            if isinstance(frame, TransformStamped): 
                raise ValueError
            elif isinstance(frame, Pose): 
                nd_tform = pose_to_ndarray(frame) @ _CONSTANT_TRANSFORM
            elif isinstance(frame, PoseStamped):
                nd_tform = pose_to_ndarray(frame.pose) @ _CONSTANT_TRANSFORM
            else:
                raise ValueError
            
            q_new = transforms3d.quaternions.mat2quat(nd_tform[:3,:3])
            
            transform = Transform(
                translation = Vector3(
                    x=nd_tform[0,3],
                    y=nd_tform[1,3],
                    z=nd_tform[2,3],
                ),
                rotation = Quaternion(w=q_new[0], x=q_new[1], y=q_new[2], z=q_new[3])
            )
            
            self._rviz_camera_tform_publisher.sendTransform(
                    TransformStamped(
                    header=Header(frame_id="base"),
                    child_frame_id="tf_dynamic_path",
                    transform=transform
                )
            )

            self._fps_signal.clear()

    def _update_pose(self, msg: PoseStamped):
        self._state_lock.acquire()

        self._current_pose = msg.pose
        if self._follow_eef.is_set():
            self._compute_and_send_camera_frame(self._current_pose)

        if self._pose_reception_counter == -1:
            # "Invalid" counter, somewhat hacky...
            self._state_lock.release()
            return

        # Add 50 waypoints/sec
        # TODO: make waypoint decimation configurable
        if self._pose_reception_counter % 10 == 0:
            self._node.get_logger().debug(f"Adding {msg.pose} at {msg.header.stamp}")
            self._exercise_traj_poses.append(msg)
        
        self._pose_reception_counter += 1
        self._state_lock.release()

    def _update_joint_angles(self, msg: JointState):
        self._state_lock.acquire()

        if self._joint_angle_reception_counter == -1:
            # "Invalid" counter, somewhat hacky...
            self._state_lock.release()
            return

        # Add 50 waypoints/sec
        # TODO: make waypoint decimation configurable
        if self._joint_angle_reception_counter % 10 == 0:
            self._node.get_logger().debug(f"Adding {msg.position} at {msg.header.stamp}")
            self._exercise_traj_joint_angles.append(msg.position)
            if len(self._exercise_traj_time) > 0:
                _time_f64 = (msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9) - \
                (self._exercise_traj_time[0].sec + self._exercise_traj_time[0].nanosec*1e-9)
                self._exercise_traj_time.append(
                    Duration(sec = int(_time_f64), nanosec = int((_time_f64 - int(_time_f64)) * 1e9))
                )
            else:
                self._exercise_traj_time.append(
                    Duration(sec = msg.header.stamp.sec, nanosec = msg.header.stamp.nanosec)
                )
        
        self._joint_angle_reception_counter += 1
        self._state_lock.release()

def main():
    # Create the application
    app = QApplication(sys.argv)

    rclpy.init()

    node = Node("ur_custom_node")
    
    # Create the main window
    main_window = URExerciseControlWindow(node)
    main_window.show()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(main_window._robot._node)
    executor.add_node(main_window._robot._service_node)

    rclpy_spinner = RclpySpinner([node, main_window._robot._node, main_window._robot._service_node])
    rclpy_spinner.start()
    
    # Run the application's event loop
    app.exec_()
    rclpy_spinner.quit()
    sys.exit()