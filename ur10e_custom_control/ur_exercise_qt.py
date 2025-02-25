import sys
import threading
import pickle
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from PyQt5.QtWidgets import *

from ur_control_qt import URControlQtWindow
from ur10e_configs import UR_QOS_PROFILE
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration

from std_srvs.srv import Trigger
from exercise_decoder_node.exercise_decoder_node_configs import (
    MINDROVE_ACTIVATION_SERVICE,
    MINDROVE_DEACTIVATION_SERVICE
)

from typing import Callable
from functools import partial

from dataclasses import dataclass, asdict

@dataclass
class Exercise:
    name: str
    poses: list[PoseStamped]
    joint_angles: list[list[float]]
    duration: list[Duration]

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

        self._path_publisher = self._node.create_publisher(PoseArray, "dynamic_force_path", 0)

    def __conf_exercise_tab(self, layout: QLayout) -> None:
        _run_loop = False
        def _start_freedrive_exercise_trajectory(_):
            # TODO: set freedrive mode
            # TODO: verify we are in freedrive mode???

            assert self.create_subscriber(msg_type = JointState, topic = "/joint_states", callback = self._update_joint_angles, qos_profile = UR_QOS_PROFILE), \
                "Unable to create joint position subscriber"
            assert self.create_subscriber(msg_type = PoseStamped, topic = "tcp_pose_broadcaster/pose", callback = self._update_pose, qos_profile = UR_QOS_PROFILE), \
                "Unable to create TCP pose broadcaster subscriber"
            
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
            if self._robot is None: return
            if len(self._exercise_traj_joint_angles) > 1:
                self._robot.run_dynamic_force_mode(
                    poses = self._exercise_traj_poses, 
                    blocking = True
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
            pose_array = PoseArray()
            pose_array.header = Header(frame_id="base") # TODO: path path name
            pose_array.poses = [x.pose for x in self._exercise_traj_poses]
            # self._node.get_logger().info(pose_array.poses)
            self._path_publisher.publish(pose_array)

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
            self._node.get_logger().info(f"Adding {msg.pose} at {msg.header.stamp}")
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
            self._node.get_logger().info(f"Adding {msg.position} at {msg.header.stamp}")
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

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(main_window._robot._node)
    executor.add_node(main_window._robot._service_node)

    def _spin():
        while executor._context.ok() and not executor._is_shutdown:
            node.get_logger().debug("Executor spin alive")
            executor.spin_once() 

    threading.Thread(target = _spin, daemon = False).start()
    
    # Run the application's event loop
    sys.exit(app.exec_())