#!/usr/bin/env python

import math

import rospy
import actionlib
import std_msgs.msg
import vigir_footstep_planning_msgs.msg

from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtCore import Qt, QObject, Slot, QSignalMapper
from python_qt_binding.QtWidgets import QHBoxLayout, QVBoxLayout, QCheckBox, QLineEdit, QPushButton, QDoubleSpinBox

from vigir_footstep_planning_msgs.msg import StepPlanRequest, StepPlanRequestAction, StepPlanRequestGoal, StepPlanRequestResult, PatternParameters, Feet
from vigir_footstep_planning_lib.execute_step_plan_widget import *
from vigir_footstep_planning_lib.error_status_widget import *
from vigir_footstep_planning_lib.parameter_set_widget import *
from vigir_footstep_planning_lib.qt_helper import *


class StepInterfaceDialog(Plugin):

    def __init__(self, context):
        super(StepInterfaceDialog, self).__init__(context)
        self.setObjectName('StepInterfaceDialog')

        self._parent = QWidget()
        self._widget = StepInterfaceWidget(self._parent)

        context.add_widget(self._parent)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()


class StepInterfaceWidget(QObject):

    command_buttons = []
    start_feet = Feet()

    def __init__(self, context, add_execute_widget=True):
        super(StepInterfaceWidget, self).__init__()

        # init signal mapper
        self.command_mapper = QSignalMapper(self)
        self.command_mapper.mapped.connect(self._publish_step_plan_request)

        # start widget
        widget = context
        error_status_widget = QErrorStatusWidget()
        self.logger = Logger(error_status_widget)
        vbox = QVBoxLayout()

        # start control box
        controls_hbox = QHBoxLayout()

        # left coloumn
        left_controls_vbox = QVBoxLayout()
        left_controls_vbox.setContentsMargins(0,0,0,0)

        self.add_command_button(left_controls_vbox, "Rotate Left", PatternParameters.ROTATE_LEFT)
        self.add_command_button(left_controls_vbox, "Strafe Left", PatternParameters.STRAFE_LEFT)
        self.add_command_button(left_controls_vbox, "Step Up", PatternParameters.STEP_UP)
        self.add_command_button(left_controls_vbox, "Center on Left", PatternParameters.FEET_REALIGN_ON_LEFT)

        left_controls_vbox.addStretch()
        controls_hbox.addLayout(left_controls_vbox, 1)

        # center coloumn
        center_controls_vbox = QVBoxLayout()
        center_controls_vbox.setContentsMargins(0,0,0,0)

        self.add_command_button(center_controls_vbox, "Forward", PatternParameters.FORWARD)
        self.add_command_button(center_controls_vbox, "Backward", PatternParameters.BACKWARD)
        self.add_command_button(center_controls_vbox, "Step Over", PatternParameters.STEP_OVER)
        self.add_command_button(center_controls_vbox, "Center Feet", PatternParameters.FEET_REALIGN_ON_CENTER)
        self.add_command_button(center_controls_vbox, "Wide Stance", PatternParameters.WIDE_STANCE)

        center_controls_vbox.addStretch()
        controls_hbox.addLayout(center_controls_vbox, 1)

        # right coloumn
        right_controls_vbox = QVBoxLayout()
        right_controls_vbox.setContentsMargins(0,0,0,0)

        self.add_command_button(right_controls_vbox, "Rotate Right", PatternParameters.ROTATE_RIGHT)
        self.add_command_button(right_controls_vbox, "Strafe Right", PatternParameters.STRAFE_RIGHT)
        self.add_command_button(right_controls_vbox, "Step Down", PatternParameters.STEP_DOWN)
        self.add_command_button(right_controls_vbox, "Center on Right", PatternParameters.FEET_REALIGN_ON_RIGHT)

        right_controls_vbox.addStretch()
        controls_hbox.addLayout(right_controls_vbox, 1)

        # end control box
        add_layout_with_frame(vbox, controls_hbox, "Commands:")

        # start settings
        settings_hbox = QHBoxLayout()
        settings_hbox.setContentsMargins(0,0,0,0)

        # start left column
        left_settings_vbox = QVBoxLayout()
        left_settings_vbox.setContentsMargins(0,0,0,0)

        # frame id
        self.frame_id_line_edit = QLineEdit("/world")
        add_widget_with_frame(left_settings_vbox, self.frame_id_line_edit, "Frame ID:")

        # do closing step
        self.close_step_checkbox = QCheckBox()
        self.close_step_checkbox.setText("Do closing step")
        self.close_step_checkbox.setChecked(True)
        left_settings_vbox.addWidget(self.close_step_checkbox)

        # extra seperation
        self.extra_seperation_checkbox = QCheckBox()
        self.extra_seperation_checkbox.setText("Extra Seperation")
        self.extra_seperation_checkbox.setChecked(False)
        left_settings_vbox.addWidget(self.extra_seperation_checkbox)

        left_settings_vbox.addStretch()

        # number of steps
        self.step_number = generate_q_double_spin_box(1, 1, 50, 0, 1.0)
        add_widget_with_frame(left_settings_vbox, self.step_number, "Number Steps:")

        # start step index
        self.start_step_index = generate_q_double_spin_box(0, 0, 1000, 0, 1.0)
        add_widget_with_frame(left_settings_vbox, self.start_step_index, "Start Step Index:")

        # end left column
        settings_hbox.addLayout(left_settings_vbox, 1)

        # start center column
        center_settings_vbox = QVBoxLayout()
        center_settings_vbox.setContentsMargins(0,0,0,0)

        # start foot selection
        self.start_foot_selection_combo_box = QComboBox()
        self.start_foot_selection_combo_box.addItem("AUTO")
        self.start_foot_selection_combo_box.addItem("LEFT")
        self.start_foot_selection_combo_box.addItem("RIGHT")
        add_widget_with_frame(center_settings_vbox, self.start_foot_selection_combo_box, "Start foot selection:")

        center_settings_vbox.addStretch()

        # step Distance
        self.step_distance = generate_q_double_spin_box(0.0, 0.0, 0.5, 2, 0.01)
        add_widget_with_frame(center_settings_vbox, self.step_distance, "Step Distance (m):")

        # side step distance
        self.side_step = generate_q_double_spin_box(0.0, 0.0, 0.2, 2, 0.01)
        add_widget_with_frame(center_settings_vbox, self.side_step, "Side Step (m):")

        # rotation per step
        self.step_rotation = generate_q_double_spin_box(0.0, -30.0, 30.0, 0, 1.0)
        add_widget_with_frame(center_settings_vbox, self.step_rotation, "Step Rotation (deg):")

        # end center column
        settings_hbox.addLayout(center_settings_vbox, 1)

        # start right column
        right_settings_vbox = QVBoxLayout()
        right_settings_vbox.setContentsMargins(0,0,0,0)

        # roll
        self.roll = generate_q_double_spin_box(0.0, -30.0, 30.0, 0, 1.0)
        add_widget_with_frame(right_settings_vbox, self.roll, "Roll (deg):")

        # pitch
        self.pitch = generate_q_double_spin_box(0.0, -30.0, 30.0, 0, 1.0)
        add_widget_with_frame(right_settings_vbox, self.pitch, "Pitch (deg):")

        # use terrain model
        self.use_terrain_model_checkbox = QCheckBox()
        self.use_terrain_model_checkbox.setText("Use Terrain Model")
        self.use_terrain_model_checkbox.setChecked(False)
        self.use_terrain_model_checkbox.stateChanged.connect(self.use_terrain_model_changed)
        right_settings_vbox.addWidget(self.use_terrain_model_checkbox)

        # override mode
        self.override_checkbox = QCheckBox()
        self.override_checkbox.setText("Override 3D")
        self.override_checkbox.setChecked(False)
        right_settings_vbox.addWidget(self.override_checkbox)

        right_settings_vbox.addStretch()

        # delta z
        self.dz = generate_q_double_spin_box(0.0, -0.5, 0.5, 2, 0.01)
        add_widget_with_frame(right_settings_vbox, self.dz, "delta z per step (m):")

        # end right column
        settings_hbox.addLayout(right_settings_vbox, 1)

        # end settings
        add_layout_with_frame(vbox, settings_hbox, "Settings:")

        # parameter set selection
        self.parameter_set_widget = QParameterSetWidget(logger = self.logger)
        add_widget_with_frame(vbox, self.parameter_set_widget, "Parameter Set:")

        # execute option
        if add_execute_widget:
            add_widget_with_frame(vbox, QExecuteStepPlanWidget(logger = self.logger, step_plan_topic = "step_plan"), "Execute:")

        # add error status widget
        add_widget_with_frame(vbox, error_status_widget, "Status:")

        # end widget
        widget.setLayout(vbox)
        #context.add_widget(widget)

        # init widget
        self.parameter_set_widget.param_cleared_signal.connect(self.param_cleared)
        self.parameter_set_widget.param_changed_signal.connect(self.param_selected)
        self.commands_set_enabled(False)

        # subscriber
        self.start_feet_sub = rospy.Subscriber("set_start_feet", Feet, self.set_start_feet_callback)

        # publisher
        self.step_plan_pub = rospy.Publisher("step_plan", StepPlan, queue_size=1)

        # action clients
        self.step_plan_request_client = actionlib.SimpleActionClient("step_plan_request", StepPlanRequestAction)

    def shutdown_plugin(self):
        print "Shutting down ..."
        print "Done!"

    def add_command_button(self, parent, text, command):
        button = QPushButton(text)
        self.command_mapper.setMapping(button, command)
        button.clicked.connect(self.command_mapper.map)
        parent.addWidget(button)
        self.command_buttons.append(button)
        return button

    def set_start_feet_callback(self, feet):
        self.start_feet = feet

    # message publisher
    def _publish_step_plan_request(self, walk_command):
        params = PatternParameters()
        params.steps                = self.step_number.value()
        params.mode                 = walk_command
        params.close_step           = self.close_step_checkbox.isChecked()
        params.extra_seperation     = self.extra_seperation_checkbox.isChecked()
        params.use_terrain_model    = self.use_terrain_model_checkbox.isChecked()
        params.override             = self.override_checkbox.isChecked() and not self.use_terrain_model_checkbox.isChecked()
        params.roll                 = math.radians(self.roll.value())
        params.pitch                = math.radians(self.pitch.value())
        params.dz                   = self.dz.value()

        params.step_distance_forward   = self.step_distance.value()
        params.step_distance_sideward  = self.side_step.value()
        params.turn_angle              = math.radians(self.step_rotation.value())

        request = StepPlanRequest()
        request.header = std_msgs.msg.Header()
        request.header.stamp = rospy.Time.now()
        request.header.frame_id = self.frame_id_line_edit.text()
        request.start = self.start_feet
        request.start_step_index = self.start_step_index.value()

        if self.start_foot_selection_combo_box.currentText() == "AUTO":
            request.start_foot_selection = StepPlanRequest.AUTO
        elif self.start_foot_selection_combo_box.currentText() == "LEFT":
            request.start_foot_selection = StepPlanRequest.LEFT
        elif self.start_foot_selection_combo_box.currentText() == "RIGHT":
            request.start_foot_selection = StepPlanRequest.RIGHT
        else:
            self.logger.log_error("Unknown start foot selection mode ('" + self.start_foot_selection_combo_box.currentText() + "')!")
            return;

        if walk_command == PatternParameters.FORWARD:
            params.mode = PatternParameters.SAMPLING
        elif walk_command == PatternParameters.BACKWARD:
            params.mode                      = PatternParameters.SAMPLING
            params.step_distance_forward    *= -1;
            params.step_distance_sideward   *= -1;
            params.turn_angle               *= -1;

        request.pattern_parameters = params
        request.planning_mode = StepPlanRequest.PLANNING_MODE_PATTERN
        request.parameter_set_name.data = self.parameter_set_widget.current_parameter_set_name()

        print "Send request = ", request

        # send request
        if self.step_plan_request_client.wait_for_server(rospy.Duration(0.5)):
            self.logger.log_info("Sending footstep plan request...")

            goal = StepPlanRequestGoal()
            goal.plan_request = request;
            self.step_plan_request_client.send_goal(goal)

            if self.step_plan_request_client.wait_for_result(rospy.Duration(5.0)):
                self.logger.log_info("Received footstep plan!")
                self.logger.log(self.step_plan_request_client.get_result().status)
                self.step_plan_pub.publish(self.step_plan_request_client.get_result().step_plan)
            else:
                self.logger.log_error("Didn't received any results. Check communcation!")
        else:
            self.logger.log_error("Can't connect to footstep planner action server!")

    def commands_set_enabled(self, enable):
        for button in self.command_buttons:
            button.setEnabled(enable)

    @Slot()
    def param_cleared(self):
        self.commands_set_enabled(False)

    @Slot(str)
    def param_selected(self, name):
        self.commands_set_enabled(True)

    @Slot(int)
    def use_terrain_model_changed(self, state):
        enable_override = True
        if state == Qt.Checked:
            enable_override = False
        self.roll.setEnabled(enable_override)
        self.pitch.setEnabled(enable_override)
        self.override_checkbox.setEnabled(enable_override)
        self.dz.setEnabled(enable_override)
