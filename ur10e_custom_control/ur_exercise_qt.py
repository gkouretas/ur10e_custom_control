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
from builtin_interfaces.msg import Duration

from typing import Callable
from functools import partial

from dataclasses import dataclass

@dataclass
class Exercise:
    name: str
    waypoints: list[list[float]]
    duration: list[Duration] # TODO: can a Duration object be pickled?

class URExerciseControlWindow(URControlQtWindow):
    def __init__(self, node: Node):
        super().__init__(node)

        self.service_tab = self._create_tab(name = "Exercise Tab", layout = QVBoxLayout(), tab_create_func = self.__conf_exercise_tab)
        self._state_lock = threading.RLock()

        self._state_reception_counter = 0
        self._exercise_traj_waypoints: list[list[float]] = []
        self._exercise_traj_time: list[Duration] = []
        self._exercise_traj_ready: bool = False

    def __conf_exercise_tab(self, layout: QLayout) -> None:
        _run_loop = False
        def _start_freedrive_exercise_trajectory(_):
            # TODO: set freedrive mode
            # TODO: verify we are in freedrive mode???

            assert self.create_subscriber(msg_type = JointState, topic = "/joint_states", callback = self._update_state, qos_profile = UR_QOS_PROFILE), \
                "Unable to create joint position subscriber"
            
            self._state_lock.acquire()
            self._exercise_traj_waypoints.clear()
            self._exercise_traj_time.clear()
            self._state_reception_counter = 0
            self._state_lock.release()

        def _stop_freedrive_exercise_trajectory(_):
            # TODO: this crashes the program, idk why
            # Remove subscriber since we don't need it
            # self.remove_subscriber("/joint_states")

            # Subtract initial timestamp
            self._state_lock.acquire()
            self._state_reception_counter = -1
            self._exercise_traj_ready = True
            self._state_lock.release()

            print("Trajectory")
            for w, d in zip(self._exercise_traj_waypoints, self._exercise_traj_time):
                print(f"Duration: {d.sec + d.nanosec*1e-9}s, pos: {w}")

        def _move_to_start(_):
            # 10s trajectory to get to home pose
            self._robot.send_trajectory(
                waypts = [self._exercise_traj_waypoints[0]],
                time_vec = [Duration(sec = 10)]
            )

        def _run_exercise_trajectory(_):
            if self._robot is None: return
            if len(self._exercise_traj_waypoints) > 1:
                self._robot.send_trajectory(
                    waypts = self._exercise_traj_waypoints[1:], 
                    time_vec = self._exercise_traj_time[1:],
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

                _exercise = Exercise(
                    name = os.path.basename(fp).split(".")[0],
                    waypoints = self._exercise_traj_waypoints,
                    duration = self._exercise_traj_time
                )

                with open(fp, "wb") as _pickled_file:
                    pickle.dump(_exercise, _pickled_file)
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
                    if not isinstance(_obj, Exercise):
                        self._node.get_logger().error("Invalid exercise")
                    else:
                        self._node.get_logger().info(f"Loaded trajectory {_obj.name}")
                        with self._state_lock:
                            self._exercise_traj_waypoints = _obj.waypoints
                            self._exercise_traj_time = _obj.duration

                            self._state_reception_counter = -1
                            self._exercise_traj_ready = True
                        
            else:
                self._node.get_logger().info("No input exercise selected")

        
        def _run_exercise_loop():
            reverse = False
            while True:
                if len(self._exercise_traj_waypoints) > 1:
                    self._robot.send_trajectory(
                        waypts = self._exercise_traj_waypoints[1:] if not reverse else self._exercise_traj_waypoints[::-1][1:], 
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

            # Launch tab
        _launch_map: dict[QPushButton, Callable] = {
            QPushButton("START FREEDRIVE EXERCISE", self): _start_freedrive_exercise_trajectory,
            QPushButton("STOP FREEDRIVE EXERCISE", self): _stop_freedrive_exercise_trajectory,
            QPushButton("SAVE TRAJECTORY", self): _save_exercise_trajectory,
            QPushButton("LOAD TRAJECTORY", self): _load_exercise_trajectory,
            QPushButton("MOVE TO START", self): _move_to_start,
            QPushButton("RUN EXERCISE TRAJECTORY", self): _run_exercise_trajectory,
            QPushButton("LOOP EXERCISE TRAJECTORY", self): _loop_exercise_trajectory,
        }

        for button, callback_func in _launch_map.items():
            button.clicked.connect(partial(callback_func, button))
            layout.addWidget(button)

    def _update_state(self, msg: JointState):
        self._state_lock.acquire()

        if self._state_reception_counter == -1:
            # "Invalid" counter, somewhat hacky...
            self._state_lock.release()
            return

        # Add 50 waypoints/sec
        # TODO: make waypoint decimation configurable
        if self._state_reception_counter % 10 == 0:
            print(f"Adding {msg.position} at {msg.header.stamp}")
            self._exercise_traj_waypoints.append(msg.position)
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
        
        self._state_reception_counter += 1
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

    threading.Thread(target = executor.spin, daemon = False).start()
    
    # Run the application's event loop
    sys.exit(app.exec_())