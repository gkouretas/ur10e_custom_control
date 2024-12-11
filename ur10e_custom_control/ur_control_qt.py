import sys
import inspect
import numpy as np
import threading

import rclpy
from rclpy.subscription import Subscription

from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QTimer

from typing import Optional, Callable, Any
from ur_robot_sm import URRobotSM
from ur10e_typedefs import URService
from ur10e_configs import (
    UR_HOME_POSE, UR_JOINT_LIST
)

from rclpy.node import Node

from functools import partial
from builtin_interfaces.msg import Duration

class URControlQtWindow(QMainWindow): # TODO: make ROS node
    def __init__(self, node: Node):
        super().__init__()

        self._node = node

        # self._robot: Optional[URRobotSM] = None
        self._robot = URRobotSM(node_name = "ur_custom_robot")
        # Set the main window properties
        self.setWindowTitle("UR Control Node")
        self.setGeometry(0, 0, 400, 400)

        self.tab_widget = QTabWidget()
        self.setCentralWidget(self.tab_widget)

        self._launch_buttons = []

        self.robot_tab = self._create_tab(name = "Robot Tab", layout = QVBoxLayout(), tab_create_func = self._conf_robot_tab)
        self.service_tab = self._create_tab(name = "Service Tab", layout = QVBoxLayout(), tab_create_func = self.__conf_service_tab)

        self._subscribers: dict[str, Subscription] = {}

    def get_subscriber(self, topic: str) -> Optional[Subscription]:
        if self._robot is None:
            print("Initialize robot")
            return None
        
        return self._subscribers.get(topic)

    def create_subscriber(self, **kwargs) -> bool:
        if "topic" not in kwargs: 
            print("Need topic keyword argument")
            return False
        if kwargs.get("topic") in self._subscribers.keys():
            print("Subscriber already created, use get_subscriber")
            return True
        
        try:
            self._subscribers[kwargs["topic"]] = self._node.create_subscription(**kwargs)
            print(self._subscribers[kwargs["topic"]])
            return True
        except Exception as ex:
            print(f"Exception: {ex}")
            return False
        
    def remove_subscriber(self, topic: str) -> None:
        sub = self._subscribers.pop(topic)
        if sub is not None:
            sub.destroy()   

    def _create_tab(self, name: str, layout: QLayout, tab_create_func: Optional[Callable[[QLayout], None]] = None):
        _tab_widget = QWidget()
        _tab_layout = layout
        _tab_widget.setLayout(_tab_layout)

        if tab_create_func is not None:
           tab_create_func(_tab_layout)

        self.tab_widget.addTab(_tab_widget, name)

        return _tab_widget
    
    def _conf_robot_tab(self, layout: QLayout) -> None:
        def __init_robot(button: QPushButton):
            # TODO: remove need to do this
            # self._robot = URRobotSM(self._node)
            # self._robot = URRobotSM(node_name = "ur_custom_robot")

            button.setEnabled(False) # disable button so we can't re-initialize the class
            for button in self._launch_buttons:
                # Now that robot node is active, we can enable services
                # NOTE: this isn't actually a pre-requisite, but we use
                # the robot class' methods for processing of services, and
                # the robot node being able to run only can occur when UR
                # control has been activated...
                button.setEnabled(True)
            
        def __send_trajectory(_):
            trajectory_kwargs = self._create_signature_kwargs_from_dialog(
                "send_trajectory args", inspect.signature(self._robot.send_trajectory)
            )

            if trajectory_kwargs is not None:
                print(f"Configured trajectories: {trajectory_kwargs}")
                self._robot.send_trajectory(**trajectory_kwargs)

        def __home_robot(_):
            self._robot.send_trajectory(
                [UR_HOME_POSE], [Duration(sec = 10)], True
            )

        def __run_forward_position_control(_):
            def __on_position_control_completion():
                nonlocal _flag
                _flag = False

            def __position_control_loop():
                nonlocal _timer, _flag
                if not _flag: 
                    _timer.stop()
                    print(f"Stopped robot response: {self._robot.stop_robot()}")
                else: 
                    self._robot.publish_cyclic_commands()

            _flag = True
            self._robot.run_position_control()
            _dialog = self._create_joint_slider_dialog(self._robot.set_position_by_joint_index, -np.pi/2, np.pi/2, np.pi/100)
            _dialog.finished.connect(__on_position_control_completion)
            
            _timer = QTimer(self)
            _timer.timeout.connect(__position_control_loop)
            _timer.start(2)

            _dialog.exec_()

            print(f"Stopped robot: {self._robot.stop_robot()}")

        def __run_forward_velocity_control(_):
            def __on_velocity_control_completion():
                nonlocal _flag
                _flag = False

            def __velocity_control_loop():
                nonlocal _timer, _flag
                if not _flag: 
                    _timer.stop()
                    print(f"Stopped robot response: {self._robot.stop_robot()}")
                else: 
                    self._robot.publish_cyclic_commands()

            _flag = True
            self._robot.run_velocity_control()
            _dialog = self._create_joint_slider_dialog(self._robot.set_velocity_by_joint_index, -np.pi/2, np.pi/2, np.pi/100)
            _dialog.finished.connect(__on_velocity_control_completion)
            
            _timer = QTimer(self)
            _timer.timeout.connect(__velocity_control_loop)
            _timer.start(2)

            _dialog.exec_()

            print(f"Stopped robot: {self._robot.stop_robot()}")


        # Launch tab
        _launch_map: dict[QPushButton, Callable] = {
            QPushButton("INIT ROBOT", self): __init_robot,
            QPushButton("SEND TRAJECTORY", self): __send_trajectory,
            QPushButton("HOME ROBOT", self): __home_robot,
            QPushButton("FORWARD POSITION", self): __run_forward_position_control,
            QPushButton("FORWARD VELOCITY", self): __run_forward_velocity_control
        }

        for button, callback_func in _launch_map.items():
            button.clicked.connect(partial(callback_func, button))
            layout.addWidget(button)

    def __conf_service_tab(self, layout: QLayout) -> None:
        """
        Configure service tab for PyQt window.

        Args:
            layout (QLayout): QLayout object for tab
        """
        def __service_name(_service: URService): 
            """Private function to digest the enum name for the UR service"""
            return str(_service).split("SRV_")[-1]

        for service in URService.URServices:
            # Create button, set callback for requesting the given service
            _button = QPushButton(__service_name(service), self)
            _button.clicked.connect(partial(self._request_service, service))
            _button.setEnabled(False) # disable until launch is initiated

            self._launch_buttons.append(_button)
            layout.addWidget(_button)

    def _request_service(self, service: URService.URServiceType): 
        _service_type = URService.get_service_type(service)
        assert _service_type is not None, f"Unknown service: {service}"

        _fields_and_field_types = _service_type.Request.get_fields_and_field_types()
        if _fields_and_field_types == {}:
            print(f"Request return: {self._robot.call_service(service, _service_type.Request())}")
        else:
            service_kwargs = self._create_service_kwargs_from_dialog(service, _fields_and_field_types)
            if service_kwargs is not None:
                print(_service_type.Request(**service_kwargs))
                print(f"Request return: {self._robot.call_service(service, _service_type.Request(**service_kwargs))}")

    def _create_signature_kwargs_from_dialog(self, name: str, signature: inspect.Signature):
        if signature.parameters == {}:
            return {}
        else:
            # Create dialog window
            _dialog = QDialog()

            # Title it with the name of the service
            _dialog.setWindowTitle(name + " arguments")
            
            # Create grid, where column 0 has the input fields 
            # and column 1 has the user inputs
            _layout = QGridLayout()
            _dialog.setLayout(_layout)

            # Store user inputs, which will then be mapped to 
            # typed inputs
            _user_inputs: dict[str, QLineEdit] = {}
            _typed_inputs: dict[str, Any] = {}

            for cnt, (field, field_type) in enumerate(signature.parameters.items()):
                _user_inputs[field] = QLineEdit()
                _layout.addWidget(QLabel(field + f" [type: {field_type.annotation}]"), cnt, 0)
                _layout.addWidget(_user_inputs[field], cnt, 1)
            
            _apply_button = QPushButton("Apply", _dialog)
            _apply_button.clicked.connect(_dialog.accept)

            _cancel_button = QPushButton("Cancel", _dialog)
            _cancel_button.clicked.connect(_dialog.reject)

            _layout.addWidget(_apply_button, cnt + 1, 0)
            _layout.addWidget(_cancel_button, cnt + 1, 1)

            # Execute dialog window
            if _dialog.exec_():
                # Custom inputs requested to be applied
                for field, field_type in signature.parameters.items():
                    _user_input = _user_inputs[field].text()
                    if _user_input != '':
                        import types
                        if isinstance(field_type.annotation, types.GenericAlias):
                            _typed_inputs[field] = self._typed_data(_user_input, str(field_type.annotation))
                        else:
                            _typed_inputs[field] = field_type.annotation(_user_inputs)
                
                return _typed_inputs
            else:
                # Service request canceled
                return None

    def _create_service_kwargs_from_dialog(self, service: URService.URServiceType, fields_and_field_types: dict[str, str]) -> dict[str, Any]:
        # Create dialog window
        _dialog = QDialog()

        # Title it with the name of the service
        _dialog.setWindowTitle(service.value + " arguments")
        
        # Create grid, where column 0 has the input fields 
        # and column 1 has the user inputs
        _layout = QGridLayout()
        _dialog.setLayout(_layout)

        # Store user inputs, which will then be mapped to 
        # typed inputs
        _user_inputs: dict[str, QLineEdit] = {}
        _typed_inputs: dict[str, Any] = {}

        for cnt, (field, field_type) in enumerate(fields_and_field_types.items()):
            _user_inputs[field] = QLineEdit()
            _layout.addWidget(QLabel(field + f" [type: {field_type}]"), cnt, 0)
            _layout.addWidget(_user_inputs[field], cnt, 1)
        
        _apply_button = QPushButton("Apply", _dialog)
        _apply_button.clicked.connect(_dialog.accept)

        _cancel_button = QPushButton("Cancel", _dialog)
        _cancel_button.clicked.connect(_dialog.reject)

        _layout.addWidget(_apply_button, cnt + 1, 0)
        _layout.addWidget(_cancel_button, cnt + 1, 1)

        # Execute dialog window
        if _dialog.exec_():
            # Custom inputs requested to be applied
            for field, field_type in fields_and_field_types.items():
                _user_input = _user_inputs[field].text()
                if _user_input != '':
                    _typed_inputs[field] = self._typed_data(_user_input, field_type)

            return _typed_inputs
        else:
            # Service request canceled
            return None
    
    def _create_joint_slider_dialog(self, joint_update_callback: Callable[[float, int], None], min_val: float = 0.0, max_val: float = 100.0, val_step: float = 1.0) -> QDialog:
        _dialog = QDialog()
        _layout = QGridLayout()

        _range = (max_val - min_val)
        _half = (max_val - min_val) / 2
        _middle = (max_val + min_val) / 2

        def __map(x, in_min, in_max, out_min, out_max):
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

        def __map_slider_to_spinbox(__spinbox: QDoubleSpinBox, __joint_index: int, __joint_update_callback: Callable[[float, int], None], value: int):
            __joint_angle = __map(value, 0, 100, min_val, max_val)
            __spinbox.setValue(__joint_angle)
            __joint_update_callback(__joint_angle, __joint_index)

        def __map_spinbox_to_slider(__slider: QSlider, value: float):
            """
            Map spinbox value to slider value

            Args:
                __slider (QSlider): QSlider object
                value (float): Value from spinbox object. Will be (min_val, max_val)
            """

            # [slider] = ([spinbox] - [spinbox]) * [slider/spinbox]
            __slider.setValue(
                int(__map(value, min_val, max_val, 0, 100))
            )

        for joint_index, joint_label in enumerate(UR_JOINT_LIST):
            _layout.addWidget(QLabel(joint_label), joint_index, 0)

            _slider = QSlider(Qt.Horizontal)

            # TODO: joint limits
            _slider.setMinimum(0)
            _slider.setMaximum(100)
            _slider.setSingleStep(1)

            _spinbox = QDoubleSpinBox()
            _spinbox.setMinimum(min_val)
            _spinbox.setMaximum(max_val)
            _spinbox.setSingleStep(val_step)

            # TODO: set initial values to actual values?
            _slider.setValue(50)
            _spinbox.setValue(_middle)

            # Callback to joint update callback
            _slider.valueChanged.connect(partial(__map_slider_to_spinbox, _spinbox, joint_index, joint_update_callback))
            _spinbox.valueChanged.connect(partial(__map_spinbox_to_slider, _slider))

            _layout.addWidget(_slider, joint_index, 1)
            _layout.addWidget(_spinbox, joint_index, 2)

        _dialog.setLayout(_layout)
        return _dialog

    def _typed_data(self, text: str, field_type: str):
        _INT_TYPE = ('int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64')
        _FLOAT_TYPE = ('float', 'double')

        if field_type == 'string':
            return text
        elif "sequence" in field_type:
            _inner_type = field_type.split("<")[-1].split(">")[0] # assuming format of `sequence<type>`...
            return [self._typed_data(x.strip(), field_type = _inner_type) for x in text.split(",")]
        elif "list" == field_type[:4].lower():
            _inner_type = field_type[4:][1:-1]
            if "list" in _inner_type:
                return [self._typed_data(text[1:-1].strip(), field_type = _inner_type)]
            else:
                return [self._typed_data(x.strip(), field_type = _inner_type) for x in text[1:-1].split(",")]
        elif "boolean" in field_type:
            return True if text.lower() == "true" else False
        elif field_type in _INT_TYPE:
            return int(text)
        elif field_type in _FLOAT_TYPE:
            return float(text)
        elif "Duration" in field_type:
            return Duration(
                sec = int(float(text)), 
                nanosec = int((float(text) - int(float(text))) * 1e9)
            )
        else:
            raise TypeError(f"Unknown type: {field_type}")

def main():
    # Create the application
    app = QApplication(sys.argv)

    node = Node("ur_custom_ui")
    
    # Create the main window
    main_window = URControlQtWindow(node)
    main_window.show()

    threading.Thread(target = app.exec_, daemon = False).start()
    
    rclpy.spin()