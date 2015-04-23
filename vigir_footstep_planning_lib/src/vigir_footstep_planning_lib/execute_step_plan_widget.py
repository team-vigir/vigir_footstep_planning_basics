#!/usr/bin/env python

import rospy
import actionlib
import std_msgs.msg
import vigir_footstep_planning_msgs.msg

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout, QPushButton, QComboBox

from vigir_footstep_planning_msgs.msg import StepPlan, ExecuteStepPlanAction, ExecuteStepPlanGoal, ErrorStatus
from vigir_footstep_planning_lib.error_status_widget import *
from vigir_footstep_planning_lib.logging import *
from vigir_footstep_planning_lib.topic_widget import *
from vigir_footstep_planning_lib.qt_helper import *

# widget for sending step plans to controller
class QExecuteStepPlanWidget(QWidgetWithLogger):

    step_plan_sub = None
    execute_step_plan_client = None
    step_plan = None

    def __init__(self, parent = None, logger = Logger(), step_plan_topic = str()):
        QWidgetWithLogger.__init__(self, parent, logger)

        # start widget
        vbox = QVBoxLayout()
        vbox.setMargin(0)
        vbox.setContentsMargins(0, 0, 0, 0)

        # step plan input topic selection
        if len(step_plan_topic) == 0:
            step_plan_topic_widget = QTopicWidget(topic_type = 'vigir_footstep_planning_msgs/StepPlan')
            step_plan_topic_widget.topic_changed_signal.connect(self._init_step_plan_subscriber)
            vbox.addWidget(step_plan_topic_widget)
        else:
            self._init_step_plan_subscriber(step_plan_topic)

        # execute action server topic selection
        execute_topic_widget = QTopicWidget(topic_type = 'vigir_footstep_planning_msgs/ExecuteStepPlanAction', is_action_topic = True)
        execute_topic_widget.topic_changed_signal.connect(self._init_execute_action_client)
        vbox.addWidget(execute_topic_widget)

        # start button part
        buttons_hbox = QHBoxLayout()
        buttons_hbox.setMargin(0)

        # execute
        self.execute_command = QPushButton("Execute (Steps: 0)")
        self.execute_command.clicked.connect(self.execute_command_callback)
        self.execute_command.setEnabled(False)
        buttons_hbox.addWidget(self.execute_command)

        # repeat
        self.repeat_command = QPushButton("Repeat")
        self.repeat_command.clicked.connect(self.execute_command_callback)
        self.repeat_command.setEnabled(False)
        buttons_hbox.addWidget(self.repeat_command)

        # stop
        self.stop_command = QPushButton("Stop")
        self.stop_command.clicked.connect(self.stop_command_callback)
        self.stop_command.setEnabled(False)
        buttons_hbox.addWidget(self.stop_command)

        # end button part
        vbox.addLayout(buttons_hbox)

        # end widget
        self.setLayout(vbox)

        # init widget
        if len(step_plan_topic) == 0:
            step_plan_topic_widget.emit_topic_name()
        execute_topic_widget.emit_topic_name()

    def shutdown_plugin(self):
        print "Shutting down ..."
        if self.step_plan_sub is None:
            self.step_plan_sub.unregister()
        print "Done!"

    def _init_step_plan_subscriber(self, topic_name):
        if len(topic_name) > 0:
            if self.step_plan_sub is not None:
                self.step_plan_sub.unregister()
            self.step_plan_sub = rospy.Subscriber(topic_name, StepPlan, self.step_plan_callback)
            print "Step Plan topic changed: " + topic_name

    def _init_execute_action_client(self, topic_name):
        if len(topic_name) > 0:
            self.execute_step_plan_client = actionlib.SimpleActionClient(topic_name, ExecuteStepPlanAction)
            self.execute_command.setEnabled(self.step_plan is not None)
            self.repeat_command.setEnabled(False)
            self.stop_command.setEnabled(False)
            print "Execution topic changed: " + topic_name

    def step_plan_callback(self, step_plan):
        self.step_plan = step_plan
        self.execute_command.setText("Execute (Steps: " + str(len(step_plan.steps)) + ")")
        if len(step_plan.steps) > 0:
            self.execute_command.setEnabled(self.execute_step_plan_client is not None)
            self.repeat_command.setEnabled(False)

    def execute_command_callback(self):
        if (self.execute_step_plan_client.wait_for_server(rospy.Duration(0.5))):
            self.execute_command.setEnabled(False)
            self.repeat_command.setEnabled(True)
            self.stop_command.setEnabled(True)

            self.logger.log_info("Executing footstep plan...")
            goal = ExecuteStepPlanGoal()
            goal.step_plan = self.step_plan
            self.execute_step_plan_client.send_goal(goal)
        else:
            self.logger.log_error("Can't connect to footstep controller action server!")

    def stop_command_callback(self):
        self.stop_command.setEnabled(False)
        self.execute_step_plan_client.cancel_goal()
        self.logger.log_info("Preempting step plan.")

