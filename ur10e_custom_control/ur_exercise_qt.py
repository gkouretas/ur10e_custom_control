import sys
import threading
import pickle
import os
import numpy as np
import transforms3d

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from PyQt5.QtWidgets import *
from PyQt5.QtCore import QTimer, QThread

from ur_control_qt import URControlQtWindow
from ur10e_configs import UR_QOS_PROFILE
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
from tf2_ros import TransformBroadcaster
from std_srvs.srv import Trigger
from exercise_decoder_node.exercise_decoder_node_configs import (
    MINDROVE_ACTIVATION_SERVICE,
    MINDROVE_DEACTIVATION_SERVICE
)

from geometry_msgs.msg import Transform, TransformStamped, Vector3
from nav_msgs.msg import Path

from typing import Callable
from functools import partial

from dataclasses import dataclass, asdict

# TODO: make config or move elsewhere
_DISTANCE_THRESHOLD = 20/1000

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

        self._target_pose_publisher = self._node.create_publisher(PoseStamped, "dynamic_force_target_pose", 0)
        self._path_publisher = self._node.create_publisher(Path, "dynamic_force_path", 0)
        self._rviz_camera_tform_publisher = TransformBroadcaster(self._node, 0)

        self._exercise_freedrive_timer = QTimer(self)
        self._exercise_freedrive_timer.timeout.connect(self._robot.ping_freedrive)

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
        def _start_freedrive_exercise_trajectory(_):
            assert self.create_subscriber(msg_type = JointState, topic = "/joint_states", callback = self._update_joint_angles, qos_profile = UR_QOS_PROFILE), \
                "Unable to create joint position subscriber"
            assert self.create_subscriber(msg_type = PoseStamped, topic = "tcp_pose_broadcaster/pose", callback = self._update_pose, qos_profile = UR_QOS_PROFILE), \
                "Unable to create TCP pose broadcaster subscriber"
            
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

        def _run_dynamic_force_mode(_):
            from ur_msgs.action._dynamic_force_mode_path import DynamicForceModePath_FeedbackMessage
            def dynamic_force_mode_feedback(feedback: DynamicForceModePath_FeedbackMessage):                
                t1 = feedback.feedback.pose_actual
                t2 = feedback.feedback.pose_desired
                
                y = np.array([0.0, 1.0, 0.0])

                x = np.array([
                    t1.position.x-t2.position.x,
                    t1.position.y-t2.position.y,
                    t1.position.z-t2.position.z
                ])

                x /= np.linalg.norm(x)
                y -= (np.dot(x, y)*x)
                y /= np.linalg.norm(y)

                z = np.cross(x, y)

                R = np.array([
                    [ 0, -1,  0],  # X_mpl → Y_ros
                    [ 1,  0,  0],  # Y_mpl → -X_ros
                    [ 0,  0,  1]   # Z_mpl → Z_ros
                ])

                q_new = transforms3d.quaternions.mat2quat(
                    np.array([x, y, z]).T
                )

                # Set the camera frame to be at the eef position with the orientation
                # aligned with the target frame
                eef_with_orientation = TransformStamped(
                    header=Header(frame_id="base"),
                    child_frame_id="tf_dynamic_path",
                    transform=Transform(
                        translation = Vector3(
                            x=feedback.feedback.pose_actual.position.x,
                            y=feedback.feedback.pose_actual.position.y,
                            z=feedback.feedback.pose_actual.position.z
                        ),
                        rotation = Quaternion(w=q_new[0], x=q_new[1], y=q_new[2], z=q_new[3])
                    )
                )
                
                # TODO(george): this should probably just already be a PoseStamped object
                target_pose = PoseStamped(
                    header=Header(frame_id="base"),
                    pose=feedback.feedback.pose_desired,
                )

                self._rviz_camera_tform_publisher.sendTransform(eef_with_orientation)
                self._target_pose_publisher.publish(target_pose)
                
            if self._robot is None: return

            if len(self._exercise_traj_joint_angles) > 1:

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

            # Launch tab
        _launch_map: dict[QPushButton, Callable] = {
            QPushButton("START FREEDRIVE EXERCISE", self): _start_freedrive_exercise_trajectory,
            QPushButton("STOP FREEDRIVE EXERCISE", self): _stop_freedrive_exercise_trajectory,
            QPushButton("SAVE TRAJECTORY", self): _save_exercise_trajectory,
            QPushButton("LOAD TRAJECTORY", self): _load_exercise_trajectory,
            QPushButton("MOVE TO START", self): _move_to_start,
            QPushButton("RUN EXERCISE TRAJECTORY", self): _run_exercise_trajectory,
            QPushButton("LOOP EXERCISE TRAJECTORY", self): _loop_exercise_trajectory,
            QPushButton("ACTIVATE MINDROVE", self): _activate_mindrove,
            QPushButton("DEACTIVATE MINDROVE", self): _deactivate_mindrove,
            QPushButton("RUN DYNAMIC FORCE MODE", self): _run_dynamic_force_mode,
            QPushButton("PUBLISH PATH FOR RVIZ", self): _publish_path_for_rviz,
        }

        for button, callback_func in _launch_map.items():
            button.clicked.connect(partial(callback_func, button))
            layout.addWidget(button)

    def _update_pose(self, msg: PoseStamped):
        self._state_lock.acquire()

        if self._pose_reception_counter == -1:
            # "Invalid" counter, somewhat hacky...
            self._state_lock.release()
            return

        # Add 50 waypoints/sec
        # TODO: make waypoint decimation configurable
        if self._pose_reception_counter % 10 == 0:
            self._node.get_logger().debug(f"Adding {msg.pose} at {msg.header.stamp}")
            self._exercise_traj_poses.append(msg)
        
        self._current_pose = msg.pose
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