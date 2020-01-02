import os

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from qt_gui.plugin import Plugin
import rospkg
import rospy
import std_msgs.msg
from std_msgs.msg import Time

from march_shared_resources.msg import Error, GaitInstruction

from .LayoutBuilder import LayoutBuilder
from .MarchButton import MarchButton


class InputDevicePlugin(Plugin):

    def __init__(self, context):
        super(InputDevicePlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('InputDevicePlugin')

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in
        # the 'resource' folder of this package
        ui_file = os.path.join(
            rospkg.RosPack().get_path('march_rqt_input_device'), 'resource',
            'input_device.ui')

        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('Input Device')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle('{0} ({1})'.format(self._widget.windowTitle(),
                                                           context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Create buttons here
        home_sit_button = MarchButton(name='home_sit', image='/home_sit.png',
                                      callback=lambda: self.publish_gait(
                                          'home_sit'))
        home_stand_button = MarchButton(name='home_stand',
                                        image='/home_stand.png',
                                        callback=lambda: self.publish_gait(
                                            'home_stand'))
        gait_sit_button = MarchButton(name='gait_sit', image='/gait_sit.png',
                                      callback=lambda: self.publish_gait(
                                          'gait_sit'))
        gait_walk_button = MarchButton(name='gait_walk',
                                       image='/gait_walk_normal.png',
                                       callback=lambda: self.publish_gait(
                                           'gait_walk'))
        gait_walk_small_button = MarchButton(name='gait_walk_small',
                                             image='/gait_walk_small.png',
                                             callback=lambda: self.publish_gait(
                                                 'gait_walk_small'))
        gait_single_step_small_button = MarchButton(name='gait_single_step_small',
                                                    image='/gait_single_step_small.png',
                                                    callback=lambda: self.publish_gait(
                                                        'gait_single_step_small'))
        gait_single_step_normal_button = MarchButton(name='gait_single_step_normal',
                                                     image='/gait_single_step_normal.png',
                                                     callback=lambda: self.publish_gait(
                                                         'gait_single_step_normal'))
        gait_side_step_left_button = MarchButton(name='gait_side_step_left',
                                                 image='/gait_side_step_left.png',
                                                 callback=lambda: self.publish_gait(
                                                     'gait_side_step_left'))
        gait_side_step_right_button = MarchButton(name='gait_side_step_right',
                                                  image='/gait_side_step_right.png',
                                                  callback=lambda: self.publish_gait(
                                                      'gait_side_step_right'))
        gait_side_step_left_small_button = MarchButton(name='gait_side_step_left_small',
                                                       text='Side step left small',
                                                       callback=lambda: self.publish_gait(
                                                           'gait_side_step_left_small'))
        gait_side_step_right_small_button = MarchButton(name='gait_side_step_right_small',
                                                        text='Side step right small',
                                                        callback=lambda: self.publish_gait(
                                                            'gait_side_step_right_small'))
        gait_stand_button = MarchButton(name='gait_stand',
                                        image='/gait_stand.png',
                                        callback=lambda: self.publish_gait(
                                            'gait_stand'))
        gait_sofa_stand_button = MarchButton(name='gait_sofa_stand',
                                             image='/gait_sofa_stand_up.png',
                                             callback=lambda: self.publish_gait(
                                                 'gait_sofa_stand'))
        gait_sofa_sit_button = MarchButton(name='gait_sofa_sit',
                                           image='/gait_sofa_sit.png',
                                           callback=lambda: self.publish_gait(
                                               'gait_sofa_sit'))
        gait_stairs_up_button = MarchButton(name='gait_stairs_up',
                                            image='/gait_stairs_up.png',
                                            callback=lambda: self.publish_gait(
                                                'gait_stairs_up'))
        gait_stairs_down_button = MarchButton(name='gait_stairs_down',
                                              image='/gait_stairs_down.png',
                                              callback=lambda: self.publish_gait(
                                                  'gait_stairs_down'))
        gait_tilted_path_first_starting_step = MarchButton(name='gait_tilted_path_first_starting_step',
                                                           text='Tilted path first starting step',
                                                           callback=lambda: self.publish_gait(
                                                               'gait_tilted_path_first_starting_step'))
        gait_tilted_path_first_ending_step = MarchButton(name='gait_tilted_path_first_ending_step',
                                                         text='Tilted path first ending step',
                                                         callback=lambda: self.publish_gait(
                                                             'gait_tilted_path_first_ending_step'))
        gait_rough_terrain_high_step = MarchButton(name='gait_rough_terrain_high_step',
                                                   text='Rough terrain high step',
                                                   callback=lambda: self.publish_gait(
                                                       'gait_rough_terrain_high_step'))

        gait_rough_terrain_middle_steps = MarchButton(name='gait_rough_terrain_middle_steps',
                                                      text='Rough terrain middle steps',
                                                      callback=lambda: self.publish_gait(
                                                          'gait_rough_terrain_middle_steps'))

        gait_ramp_door_slope_up = MarchButton(name='gait_ramp_door_slope_up',
                                              text='Ramp and Door slope up',
                                              callback=lambda: self.publish_gait('gait_ramp_door_slope_up'))

        gait_ramp_door_slope_down = MarchButton(name='gait_ramp_door_slope_down',
                                                text='Ramp and Door slope down',
                                                callback=lambda: self.publish_gait('gait_ramp_door_slope_down'))

        gait_ramp_door_last_step = MarchButton(name='gait_ramp_door_last_step',
                                               text='Ramp and Door last step',
                                               callback=lambda: self.publish_gait('gait_ramp_door_last_step'))

        gait_tilted_path_straight_start_right = MarchButton(name='gait_tilted_path_straight_start_right',
                                               text='Tilted Path straight start right',
                                               callback=lambda: self.publish_gait(
                                                   'gait_tilted_path_straight_start_right'))

        gait_tilted_path_straight_start_left = MarchButton(name='gait_tilted_path_straight_start_left',
                                                            text='Tilted Path straight start left',
                                                            callback=lambda: self.publish_gait(
                                                                'gait_tilted_path_straight_start_left'))

        gait_tilted_path_first_start = MarchButton(name='gait_tilted_path_first_start',
                                                            text='Tilted Path side first start',
                                                            callback=lambda: self.publish_gait(
                                                                'gait_tilted_path_first_start'))

        gait_tilted_path_second_start = MarchButton(name='gait_tilted_path_second_start',
                                                   text='Tilted Path side second start',
                                                   callback=lambda: self.publish_gait(
                                                       'gait_tilted_path_second_start'))

        stop_button = MarchButton(name='gait_stop', image='/stop.png',
                                  callback=lambda: self.publish_stop())
        pause_button = MarchButton(name='gait_pause', text='Pause',
                                   callback=lambda: self.publish_pause())
        continue_button = MarchButton(name='gait_continue', text='Continue',
                                      callback=lambda: self.publish_continue())

        error_button = MarchButton(name='error', image='/error.png',
                                   callback=lambda: self.publish_error())

        # The button layout.
        # Position in the array determines position on screen.
        march_button_layout = [
            [home_sit_button, home_stand_button, gait_walk_button, gait_walk_small_button, stop_button],
            [gait_sit_button, gait_stand_button, gait_single_step_normal_button, gait_single_step_small_button,
             pause_button],
            [gait_side_step_left_button, gait_side_step_right_button, gait_side_step_left_small_button,
             gait_side_step_right_small_button, continue_button],
            [gait_sofa_sit_button, gait_sofa_stand_button, gait_rough_terrain_high_step,
             gait_rough_terrain_middle_steps, error_button],
            [gait_stairs_up_button, gait_stairs_down_button, gait_ramp_door_slope_up, gait_ramp_door_slope_down,
             gait_ramp_door_last_step],
            [gait_tilted_path_straight_start_right, gait_tilted_path_straight_start_left,
             gait_tilted_path_first_start, gait_tilted_path_second_start],
        ]

        # Create the qt_layout from the button layout.
        layout_builder = LayoutBuilder(march_button_layout)
        qt_layout = layout_builder.build()
        # Apply the qt_layout to the top level widget.
        self._widget.frame.findChild(QWidget, 'content').setLayout(qt_layout)

        # Make the frame as tight as possible with spacing between the buttons.
        qt_layout.setSpacing(15)
        self._widget.frame.findChild(QWidget, 'content').adjustSize()

        # ROS publishers.
        # It is important that you unregister them in the self.shutdown method.
        self.instruction_gait_pub = rospy.Publisher(
            '/march/input_device/instruction', GaitInstruction, queue_size=10)

        self.error_pub = rospy.Publisher('/march/error', Error, queue_size=10)

        self._ping = rospy.get_param('~ping_safety_node', True)
        if self._ping:
            self._alive_pub = rospy.Publisher(
                '/march/input_device/alive', Time, queue_size=10)
            self._alive_timer = rospy.Timer(rospy.Duration(0.05), self._timer_callback)

    def _timer_callback(self, event):
        self._alive_pub.publish(event.current_real)

    def shutdown_plugin(self):
        if self._ping:
            self._alive_timer.shutdown()
            self._alive_pub.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def publish_gait(self, string):
        rospy.logdebug('Mock Input Device published gait: ' + string)
        self.instruction_gait_pub.publish(GaitInstruction(std_msgs.msg.Header(stamp=rospy.Time.now()),
                                                          GaitInstruction.GAIT,
                                                          string))

    def publish_stop(self):
        rospy.logdebug('Mock Input Device published stop')
        self.instruction_gait_pub.publish(GaitInstruction(std_msgs.msg.Header(stamp=rospy.Time.now()),
                                                          GaitInstruction.STOP,
                                                          ''))

    def publish_continue(self):
        rospy.logdebug('Mock Input Device published continue')
        self.instruction_gait_pub.publish(GaitInstruction(std_msgs.msg.Header(stamp=rospy.Time.now()),
                                                          GaitInstruction.CONTINUE, ''))

    def publish_pause(self):
        rospy.logdebug('Mock Input Device published pause')
        self.instruction_gait_pub.publish(GaitInstruction(std_msgs.msg.Header(stamp=rospy.Time.now()),
                                                          GaitInstruction.PAUSE,
                                                          ''))

    def publish_error(self):
        rospy.logdebug('Mock Input Device published error')
        self.error_pub.publish(Error(std_msgs.msg.Header(stamp=rospy.Time.now()),
                                     'Fake error thrown by the develop input device.', Error.FATAL))

    # def trigger_configuration(self):
    # Comment in to signal that the plugin has a way to configure
    # This will enable a setting button (the gear icon)
    # in each dock widget title bar
    # Usually used to open a modal configuration dialog
