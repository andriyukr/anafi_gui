import typing
from PyQt6 import QtCore, QtGui
from PyQt6.QtMultimedia import *
from PyQt6.QtMultimediaWidgets import *
from PyQt6.QtCore import *
from PyQt6.QtWidgets import *
from anafi_gui.custom_widgets import *
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from timeit import default_timer as timer
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data, qos_profile_services_default, qos_profile_parameters, qos_profile_parameter_events
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange, SetParametersResult
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import UInt8, UInt16, UInt32, UInt64, Int8, Float32, String, Header, Bool
from geometry_msgs.msg import PoseStamped, PointStamped, QuaternionStamped, TwistStamped, Vector3Stamped, Quaternion, Twist, Vector3
from sensor_msgs.msg import Image, CameraInfo, NavSatFix
from builtin_interfaces.msg import Time
from std_srvs.srv import Trigger, SetBool
from cv_bridge import CvBridge, CvBridgeError
from anafi_ros_interfaces.msg import PilotingCommand, MoveByCommand, MoveToCommand, CameraCommand, GimbalCommand
from anafi_ros_interfaces.srv import PilotedPOI, FlightPlan, FollowMe, Location, Photo, Recording, String as StringSRV
from anafi_autonomy.msg import KeyboardCameraCommand, KeyboardDroneCommand


class AnafiGUI(Node):
	def __init__(self):
		self.node = rclpy.create_node("Anafi_GUI")
		self.node.get_logger().info("The GUI is running...")
		
		self.ui = None

	# 	# publishers
		self.pub_camera_command = self.node.create_publisher(CameraCommand, 'camera/command', qos_profile_system_default)
		self.pub_drone_command = self.node.create_publisher(PilotingCommand, 'drone/command', qos_profile_system_default)
		self.pub_moveto_command = self.node.create_publisher(MoveToCommand, 'drone/moveto', qos_profile_system_default)
		self.pub_moveby_command = self.node.create_publisher(MoveByCommand, 'drone/moveby', qos_profile_system_default)
		self.pub_gimbal_command = self.node.create_publisher(GimbalCommand, 'gimbal/command', qos_profile_system_default)
		self.pub_keyboard_action = self.node.create_publisher(UInt8, 'keyboard/action', qos_profile_system_default)
		self.pub_keyboard_camera_command = self.node.create_publisher(KeyboardCameraCommand, 'keyboard/camera_command', qos_profile_system_default)
		self.pub_keyboard_drone_command = self.node.create_publisher(KeyboardDroneCommand, 'keyboard/drone_command', qos_profile_system_default)

		
		# subscribers
		self.node.create_subscription(Image, 'camera/image', self.image_callback, qos_profile_system_default)
		self.node.create_subscription(CameraInfo, 'camera/camera_info', self.camera_info_callback, qos_profile_system_default)
		self.node.create_subscription(Time, 'time', self.time_callback, qos_profile_sensor_data)
		self.node.create_subscription(QuaternionStamped, 'drone/attitude', self.attitude_callback, qos_profile_sensor_data )
		self.node.create_subscription(Float32, 'drone/altitude', self.altitude_callback, qos_profile_sensor_data)
		self.node.create_subscription(PointStamped, 'drone/position', self.position_callback, qos_profile_system_default)
		self.node.create_subscription(PointStamped, 'drone/position_local', self.local_position_callback, qos_profile_system_default)
		self.node.create_subscription(Vector3Stamped, 'drone/speed', self.speed_callback, qos_profile_sensor_data)
		self.node.create_subscription(UInt16, 'link/goodput', self.goodput_callback, qos_profile_system_default)
		self.node.create_subscription(UInt8, 'link/quality', self.quality_callback, qos_profile_system_default)
		self.node.create_subscription(Int8, 'link/rssi', self.rssi_callback, qos_profile_system_default)
		self.node.create_subscription(UInt8, 'battery/percentage', self.percentage_callback, qos_profile_system_default)
		self.node.create_subscription(String, 'drone/state', self.state_callback, qos_profile_system_default)
		self.node.create_subscription(Vector3Stamped, 'drone/rpy', self.rpy_callback, qos_profile_sensor_data)
		self.node.create_subscription(Float32, 'camera/exposure_time', self.exposure_time_callback, qos_profile_system_default)
		self.node.create_subscription(UInt16, 'camera/iso_gain', self.iso_gain_callback, qos_profile_system_default)
		self.node.create_subscription(Float32, 'camera/awb_r_gain', self.awb_r_gain_callback, qos_profile_system_default)
		self.node.create_subscription(Float32, 'camera/awb_b_gain', self.awb_b_gain_callback, qos_profile_system_default)
		self.node.create_subscription(Float32, 'camera/hfov', self.hfov_callback, qos_profile_system_default)
		self.node.create_subscription(Float32, 'camera/vfov', self.vfov_callback, qos_profile_system_default)
		self.node.create_subscription(Bool, 'drone/gps/fix', self.gps_fix_callback, qos_profile_system_default)
		self.node.create_subscription(Bool, 'drone/steady', self.steady_callback, qos_profile_sensor_data)
		self.node.create_subscription(UInt8, 'battery/health', self.health_callback, qos_profile_system_default)
		self.node.create_subscription(UInt8, 'drone/gps/satellites', self.gps_satellite_callback, qos_profile_system_default)
		self.node.create_subscription(Float32, 'camera/zoom', self.zoom_callback, qos_profile_system_default)
		self.node.create_subscription(NavSatFix, 'drone/gps/location', self.gps_location_callback, qos_profile_system_default)
		self.node.create_subscription(Vector3Stamped, 'gimbal/attitude/absolute', self.camera_altitude_callback, qos_profile_system_default)

		# clients
		# self.cli_drone_arm = self.node.create_client(SetBool, 'drone/arm')
		# self.cli_drone_takeoff = self.node.create_client(Trigger, 'drone/takeoff')
		# self.cli_drone_land = self.node.create_client(Trigger, 'drone/land')
		# self.cli_drone_emergency = self.node.create_client(Trigger, 'drone/emergency')
		# self.cli_drone_halt = self.node.create_client(Trigger, 'drone/halt')
		# self.cli_drone_rth = self.node.create_client(Trigger, 'drone/rth')
		# self.cli_drone_reboot = self.node.create_client(Trigger, 'drone/reboot')
		# self.cli_drone_calibrate = self.node.create_client(Trigger, 'drone/calibrate')
		# self.cli_skycontroller_offboard = self.node.create_client(SetBool, 'skycontroller/offboard')
		# self.cli_skycontroller_discover_drones = self.node.create_client(Trigger, 'skycontroller/discover_drones')
		# self.cli_skycontroller_forget_drone = self.node.create_client(Trigger, 'skycontroller/forget_drone')
		# self.cli_home_set = self.node.create_client(Location, 'home/set')
		# self.cli_home_navigate = self.node.create_client(SetBool, 'home/navigate')
		# self.cli_POI_start = self.node.create_client(PilotedPOI, 'POI/start')
		# self.cli_POI_stop = self.node.create_client(Trigger, 'POI/stop')
		# self.cli_flightplan_upload = self.node.create_client(FlightPlan, 'flightplan/upload')
		# self.cli_flightplan_start = self.node.create_client(FlightPlan, 'flightplan/start')
		# self.cli_flightplan_pause = self.node.create_client(Trigger, 'flightplan/pause')
		# self.cli_flightplan_stop = self.node.create_client(Trigger, 'flightplan/stop')
		# self.cli_followme_start= self.node.create_client(FollowMe, 'followme/start')
		# self.cli_followme_stop= self.node.create_client(Trigger, 'followme/stop')
		# self.cli_gimbal_reset= self.node.create_client(Trigger, 'gimbal/reset')
		# self.cli_gimbal_calibrate= self.node.create_client(Trigger, 'gimbal/calibrate')
		# self.cli_camera_reset= self.node.create_client(Trigger, 'camera/reset')
		# self.cli_camera_photo_take = self.node.create_client(Trigger, 'camera/photo/take')
		# self.cli_camera_photo_stop = self.node.create_client(Trigger, 'camera/photo/stop')
		# self.cli_camera_recording_start = self.node.create_client(Recording, 'camera/recording/start')
		# self.cli_camera_recording_stop = self.node.create_client(Recording, 'camera/recording/stop')
		# self.cli_storage_download = self.node.create_client(SetBool, 'storage/download')
		# self.cli_storage_format = self.node.create_client(Trigger, 'storage/format')
		

		# Messages
		# self.msg_camera_command = CameraCommand()
		# self.msg_drone_command = PilotingCommand()
		self.msg_moveto_command = MoveToCommand()
		self.msg_moveby_command = MoveByCommand()
		self.msg_gimbal_command = GimbalCommand()
		self.msg_keyboard_drone_command = KeyboardDroneCommand()
		self.msg_keyboard_camera_command = KeyboardCameraCommand()
		self.msg_keyboard_action = UInt8()
		
	def set_ui(self, ui):
		self.ui = ui

	def keyboard_event(self, key: QtGui.QKeyEvent):
		self.msg_keyboard_drone_command = KeyboardDroneCommand()
		self.msg_keyboard_camera_command = KeyboardCameraCommand()
		self.msg_keyboard_action.data = 0
		
		# Keyboard/drone_command
		if key.key() == Qt.Key.Key_A:
			self.msg_keyboard_drone_command.yaw = 100
		elif key.key() == Qt.Key.Key_D:
			self.msg_keyboard_drone_command.yaw = -100
		elif key.key() == Qt.Key.Key_W:
			self.msg_keyboard_drone_command.z = 100
		elif key.key() == Qt.Key.Key_S:
			self.msg_keyboard_drone_command.z = -100 
		elif key.key() == Qt.Key.Key_Right:
			self.msg_keyboard_drone_command.y = -100 # move right
		elif key.key() == Qt.Key.Key_Left:
			self.msg_keyboard_drone_command.y = 100 # move left
		elif key.key() == Qt.Key.Key_Up:
			self.msg_keyboard_drone_command.x = 100
		elif key.key() == Qt.Key.Key_Down:
			self.msg_keyboard_drone_command.x = -100
		
		# Keyboard/Camera Command
		elif key.key() == Qt.Key.Key_F9:
			self.msg_keyboard_camera_command.action = 111
		elif key.key() == Qt.Key.Key_5:
			self.msg_keyboard_camera_command.action = 111
		elif key.key() == Qt.Key.Key_9:
			self.msg_keyboard_camera_command.roll = 100
		elif key.key() == Qt.Key.Key_7:
			self.msg_keyboard_camera_command.roll = -100
		elif key.key() == Qt.Key.Key_2:
			self.msg_keyboard_camera_command.pitch = 100
		elif key.key() == Qt.Key.Key_8:
			self.msg_keyboard_camera_command.pitch = -100
		elif key.key() == Qt.Key.Key_6:
			self.msg_keyboard_camera_command.yaw = 100
		elif key.key() == Qt.Key.Key_4:
			self.msg_keyboard_camera_command.yaw = -100
		elif key.key() == Qt.Key.Key_Plus:
			self.msg_keyboard_camera_command.zoom = 100
		elif key.key() == Qt.Key.Key_Minus:
			self.msg_keyboard_camera_command.zoom = -100
		elif key.key() == Qt.Key.Key_Enter:
			self.msg_keyboard_camera_command.action = 1
		elif key.key() == Qt.Key.Key_0:
			self.msg_keyboard_camera_command.action = 2
		elif key.key() == Qt.Key.Key_Period:
			self.msg_keyboard_camera_command.action = 3
		elif key.key() == Qt.Key.Key_Slash:
			self.msg_keyboard_camera_command.action = 4

		# Keyboard/action
		elif key.key() == Qt.Key.Key_Insert:
			self.msg_keyboard_action.data = 1
		elif key.key() == Qt.Key.Key_T:
			self.msg_keyboard_action.data = 2
		elif key.key() == Qt.Key.Key_Space:
			self.msg_keyboard_action.data = 3			       
		elif key.key() == Qt.Key.Key_L:
			self.msg_keyboard_action.data = 4
		elif key.key() == Qt.Key.Key_Escape:
			self.msg_keyboard_action.data = 5			
		elif key.key() == Qt.Key.Key_R:
			self.msg_keyboard_action.data = 6
		elif key.key() == Qt.Key.Key_B:
			self.msg_keyboard_action.data = 7
		elif key.key() == Qt.Key.Key_F1:
			self.msg_keyboard_action.data = 101
		elif key.key() == Qt.Key.Key_F2:
			self.msg_keyboard_action.data = 102			
		elif key.key() == Qt.Key.Key_F4:
			self.msg_keyboard_action.data = 110
		elif key.key() == Qt.Key.Key_F5:
			self.msg_keyboard_action.data = 11
		elif key.key() == Qt.Key.Key_F6:
			self.msg_keyboard_action.data = 12
		elif key.key() == Qt.Key.Key_F7:
			self.msg_keyboard_action.data = 13
		elif key.key() == Qt.Key.Key_F12:
			self.msg_keyboard_action.data = 111	
		
		self.node.get_logger().info('Publishing topic keyboard/action: "%s"' % self.msg_keyboard_action.data)
		self.pub_keyboard_action.publish(self.msg_keyboard_action)

		self.node.get_logger().info('Publishing topic keyboard/camera_command: "%s"' % self.msg_keyboard_action.data)
		self.pub_keyboard_camera_command.publish(self.msg_keyboard_camera_command)
		
		self.node.get_logger().info('Publishing topic keyboard/action: "%s"' % self.msg_keyboard_action.data)
		self.pub_keyboard_drone_command.publish(self.msg_keyboard_drone_command)
		
		
			
	
			

	def image_callback(self, msg):
		pass
	
	def camera_info_callback(self, msg):
		pass
	
	def time_callback(self, msg):
		pass
	
	def attitude_callback(self, msg):
		pass
	
	def altitude_callback(self, msg):
		pass
	
	def position_callback(self, msg):
		pass
	
	def local_position_callback(self, msg):
		pass
	
	def speed_callback(self, msg):
		pass
	
	def goodput_callback(self, msg):
		pass
	
	def quality_callback(self, msg):
		pass
	
	def rssi_callback(self, msg):
		pass

	def percentage_callback(self, msg):
		self.ui.battery_info.setValue(msg.data)

	def state_callback(self, msg):
		self.ui.status_info.setValue(msg.data)
		

	def rpy_callback(self, msg):
		pass

	def exposure_time_callback(self, msg):
		pass

	def iso_gain_callback(self, msg):
		pass

	def awb_r_gain_callback(self, msg):
		pass

	def awb_b_gain_callback(self, msg):
		pass
	
	def hfov_callback(self, msg):
		pass

	def vfov_callback(self, msg):
		pass

	def gps_fix_callback(self, msg):
		self.ui.gps_fix_info.setValue(msg.data)

	def steady_callback(self, msg):
		pass

	def health_callback(self, msg):
		pass

	def gps_satellite_callback(self,msg):
		self.ui.gps_satellite_info.setValue(msg.data)

	def gps_location_callback(self,msg):
		self.ui.gps_location_info.setValue("[%.1f, %.1f, %.1f]" % (msg.latitude, msg.longitude, msg.altitude))

	def zoom_callback(self,msg):
		self.ui.zoom_info.setValue(msg.data)

	def camera_altitude_callback(self,msg):
			self.ui.camera_altitude_info.setValue(msg.vector.z)

class UI(object):
	def __init__(self, mainwindow: QtWidgets.QMainWindow, size: QSize, node: Node):
		self.node = node
		
		default_height = size.height()
		default_width = size.width()

		self.centralwidget = QtWidgets.QWidget()
		self.scroll = QtWidgets.QScrollArea()
		self.scroll.setWidget(self.centralwidget)
		self.scroll.setWidgetResizable(True)
		self.scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
		self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
		mainwindow.setCentralWidget(self.scroll)
		# video stream and status area setup
		self.vstream_status_hlw = QtWidgets.QWidget()
		# self.vstream_status_hlw.setGeometry(QtCore.QRect(45, 20, 1350, 450))
		self.vstream_status_hlw.setMinimumSize(default_width - 50, int(default_height / 2) - 50)
		self.vstream_status_hlw.setMaximumSize(default_width - 50, int(default_height / 2) - 50)

		self.videostream_status_hl = QtWidgets.QHBoxLayout(self.vstream_status_hlw)
		self.videostream_status_hl.setContentsMargins(5, 5, 5, 5)
		# video stream area
		self.buildVStreamSection(self.vstream_status_hlw)
		# status area
		self.buildStatusSection(self.vstream_status_hlw)

		# commands and parameters area setup
		self.commands_parameters_hlw = QtWidgets.QWidget()
		# self.commands_parameters_hlw.setGeometry(QtCore.QRect(45, 500, 1350, 350))
		self.commands_parameters_hlw.setMinimumSize(default_width - 50, int(default_height / 2) - 50)
		self.commands_parameters_hlw.setMaximumSize(default_width - 50, int(default_height / 2) - 50)
		self.commands_parameters_hl = QtWidgets.QHBoxLayout(self.commands_parameters_hlw)
		self.commands_parameters_hl.setContentsMargins(5, 5, 5, 5)
		# commands area
		self.buildCommandsSection(self.commands_parameters_hlw)
		# parameters area
		self.buildParametersSection(self.commands_parameters_hlw)

		self.overalllayout = QtWidgets.QVBoxLayout(self.centralwidget)
		self.overalllayout.addWidget(self.vstream_status_hlw)
		self.overalllayout.addWidget(self.commands_parameters_hlw)


		# mainwindow.setCentralWidget(self.centralwidget)
		mainwindow.setWindowTitle("Drone GUI")
		self.menubar = QtWidgets.QMenuBar(mainwindow)
		self.menubar.setGeometry(QtCore.QRect(0, 0, 1440, 20))
		mainwindow.setMenuBar(self.menubar)
		self.statusbar = QtWidgets.QStatusBar(mainwindow)
		mainwindow.setStatusBar(self.statusbar)
		QtCore.QMetaObject.connectSlotsByName(mainwindow)

	def buildStatusSection(self, parent: QtWidgets.QWidget):
		self.statusmain_gb = QtWidgets.QGroupBox(parent)
		self.statusmain_gb.setProperty("class", "smaingb")
		self.videostream_status_hl.addWidget(self.statusmain_gb)
		self.status_vl = QtWidgets.QVBoxLayout(self.statusmain_gb)

		self.status_title_label = TitleLabel(layout=self.status_vl, text="Status", parent=self.statusmain_gb)

		# status area r1
		self.status_r1_hl = QtWidgets.QHBoxLayout()
		# status
		self.status_info = StatusGroupBox(self.statusmain_gb, "Status: ", "<status>")
		self.status_info.setup(self.status_r1_hl)

		# model
		self.model_info = StatusGroupBox(self.statusmain_gb, "Model: ", "<model>")
		self.model_info.setup(self.status_r1_hl)

		# ip
		self.ip_info = StatusGroupBox(self.statusmain_gb, "IP: ", "<ip>")
		self.ip_info.setup(self.status_r1_hl)

		self.status_vl.addLayout(self.status_r1_hl)

		# status r2
		self.status_r2_hl = QtWidgets.QHBoxLayout()

		# control
		self.control_info = StatusGroupBox(self.statusmain_gb, "Control: ", "<<off./man.>>")
		self.control_info.setup(self.status_r2_hl)

		# battery
		self.battery_info = StatusGroupBox(self.statusmain_gb, "Battery: ", "<battery>")
		self.battery_info.setup(self.status_r2_hl)

		# link
		self.link_info = StatusGroupBox(self.statusmain_gb, "Link: ", "<l.q.>")
		self.link_info.setup(self.status_r2_hl)

		self.status_vl.addLayout(self.status_r2_hl)

		# status r3
		self.status_r3_hl = QtWidgets.QHBoxLayout()

		# attitude
		self.attitude_info = StatusGroupBox(self.statusmain_gb, "Attitude: ", "<[<r.>, <p.>, <y.>]>")
		self.attitude_info.setup(self.status_r3_hl)

		# altitude
		self.altitude_info = StatusGroupBox(self.statusmain_gb, "Altitude: ", "<altitude>")
		self.altitude_info.setup(self.status_r3_hl)

		# speed
		self.speed_info = StatusGroupBox(self.statusmain_gb, "Speed: ", "[<vx>, <vy>, <vz>]")
		self.speed_info.setup(self.status_r3_hl)

		self.status_vl.addLayout(self.status_r3_hl)

		# status r4
		self.status_r4_hl = QtWidgets.QHBoxLayout()

		# gps location
		self.gps_location_info = StatusGroupBox(self.statusmain_gb, "GPS Location: ", "[<lat.>, <lon.>, <alt.>]")
		self.gps_location_info.setup(self.status_r4_hl)

		# gps fix
		self.gps_fix_info = StatusGroupBox(self.statusmain_gb, "GPS Fix: ", "<GPS f.>")
		self.gps_fix_info.setup(self.status_r4_hl)

		# gps satellites
		self.gps_satellite_info = StatusGroupBox(self.statusmain_gb, "GPS Satellites: ", "<GPS #s>")
		self.gps_satellite_info.setup(self.status_r4_hl)

		self.status_vl.addLayout(self.status_r4_hl)

		# status r5
		self.status_r5_hl = QtWidgets.QHBoxLayout()

		# camera altitude
		self.camera_altitude_info = StatusGroupBox(self.statusmain_gb, "Camera Altitude: ", "<cam. alt.>")
		self.camera_altitude_info.setup(self.status_r5_hl)

		# zoom
		self.zoom_info = StatusGroupBox(self.statusmain_gb, "Zoom: ", "<zoom>")
		self.zoom_info.setup(self.status_r5_hl)

		# storage
		self.storage_info = StatusGroupBox(self.statusmain_gb, "Storage: ", "<storage>")
		self.storage_info.setup(self.status_r5_hl)

		self.status_vl.addLayout(self.status_r5_hl)

		# status r6
		self.status_r6_hl = QtWidgets.QHBoxLayout()

		# drone command
		self.drone_command_info = StatusGroupBox(self.statusmain_gb, "Drone Command: ", "[<r.>, <p.>, <g.>, <y.>]")
		self.drone_command_info.setup(self.status_r6_hl)

		# move by
		self.move_by_info = StatusGroupBox(self.statusmain_gb, "Move By: ", "[<dx>, <dy>, <dz>, <dyaw>]")
		self.move_by_info.setup(self.status_r6_hl)
		
		# move to
		self.move_to_info = StatusGroupBox(self.statusmain_gb, "Move To: ", "[<lat.>, <lon.>, <alt.>, <head.>]")
		self.move_to_info.setup(self.status_r6_hl)
		# end r6

		# end status
		self.status_vl.addLayout(self.status_r6_hl)
		
	def buildVStreamSection(self, parent: QtWidgets.QWidget):
		self.vstream_gb = QtWidgets.QGroupBox(parent)
		self.vstream_gb.setProperty("class", "vstreamgb")
		self.videostream_status_hl.addWidget(self.vstream_gb)
		self.video_stream_vl = QtWidgets.QVBoxLayout(self.vstream_gb)

		# video stream
		self.media_player = QMediaPlayer()
		self.video_widget = QVideoWidget(parent)
		self.video_widget.setMinimumSize(625, 350)
		self.video_widget.setMaximumSize(625, 350)
		# self.media_player.setSource()) # TODO: set video source
		self.media_player.setVideoOutput(self.video_widget)

		self.video_stream_vl.addWidget(self.video_widget, 1,
									   QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignVCenter)

		self.video_stream_hl = QtWidgets.QHBoxLayout()
		self.video_stream_hl.setContentsMargins(5, 5, 5, 5)
		vstream_button_s1 = QtWidgets.QSpacerItem(15, 10, QtWidgets.QSizePolicy.Policy.Minimum,
												  QtWidgets.QSizePolicy.Policy.Minimum)
		self.video_stream_hl.addItem(vstream_button_s1)

		self.rgb_button = EnhancedButton(parent=self.vstream_gb, label="RGB", fixed_size=(120, 50),
										 layout=self.video_stream_hl, property_class="vsbutton")

		self.thermal_button = EnhancedButton(parent=self.vstream_gb, label="Thermal", fixed_size=(120, 50),
											 layout=self.video_stream_hl, property_class="vsbutton")

		self.blended_button = EnhancedButton(parent=self.vstream_gb, label="Blended", fixed_size=(120, 50),
											 layout=self.video_stream_hl, property_class="vsbutton")

		self.disparity_button = EnhancedButton(parent=self.vstream_gb, label="Disparity", fixed_size=(120, 50),
											   layout=self.video_stream_hl, property_class="vsbutton")

		# vstream buttons end
		self.video_stream_vl.addLayout(self.video_stream_hl)

	def buildCommandsSection(self, parent:QtWidgets.QWidget):
		self.commands_tabs = QtWidgets.QTabWidget(parent)
		self.commands_tabs.setProperty("class", "commands")
		self.commands_maintab = QtWidgets.QWidget()
		self.commands_tabs.addTab(self.commands_maintab, "Commands")
		self.commands_tabs.tabBar().setProperty("class", "commandstab")

		self.commands_parameters_hl.addWidget(self.commands_tabs)
		self.commands_vl = QtWidgets.QVBoxLayout(self.commands_maintab)

		commands_r1_vspacer = QtWidgets.QSpacerItem(10, 10, QtWidgets.QSizePolicy.Policy.Minimum,
													QtWidgets.QSizePolicy.Policy.Minimum)
		self.commands_vl.addItem(commands_r1_vspacer)

		self.commands_title_label = TitleLabel(layout=self.commands_vl, parent=self.commands_maintab, text="Commands")

		# commands r1
		commands_r1_vspacer = QtWidgets.QSpacerItem(10, 10, QtWidgets.QSizePolicy.Policy.Minimum,
													QtWidgets.QSizePolicy.Policy.Minimum)
		self.commands_vl.addItem(commands_r1_vspacer)

		self.commands_r1_hl = QtWidgets.QHBoxLayout()
		self.commands_r1_hl.setContentsMargins(5, 5, 5, 5)

		self.arm_disarm_button = EnhancedButton(parent=self.commands_maintab, checkable=True, labels=("Arm", "Disarm"),
												fixed_size=(150, 90), layout=self.commands_r1_hl,
												property_class="commandsbutton", callback_function=self.arm_disarm_callback, ui=self)

		self.takeoff_land_button = EnhancedButton(parent=self.commands_maintab, checkable=True,
												  labels=("Take-Off", "Land"), property_class="commandsbutton",
												  fixed_size=(150, 90), layout=self.commands_r1_hl, callback_function=self.takeoff_land_callback, ui=self)

		self.halt_button = EnhancedButton(parent=self.commands_maintab, label="Halt", fixed_size=(150, 90),
										  layout=self.commands_r1_hl, property_class="commandsbutton", callback_function=self.halt_callback, ui=self)

		self.commands_vl.addLayout(self.commands_r1_hl)

		# commands r2
		commands_r2_vspacer = QtWidgets.QSpacerItem(10, 55, QtWidgets.QSizePolicy.Policy.Minimum,
													QtWidgets.QSizePolicy.Policy.MinimumExpanding)
		self.commands_vl.addItem(commands_r2_vspacer)

		self.commands_r2_hl = QtWidgets.QHBoxLayout()
		self.commands_r2_hl.setContentsMargins(5, 5, 5, 5)

		self.photo_startstop_button = EnhancedButton(parent=self.commands_maintab, checkable=True,
													 labels=("Start Photo", "Stop Photo"), fixed_size=(150, 60),
													 layout=self.commands_r2_hl, property_class="commandsbutton", callback_function=self.photo_start_stop_callback, ui=self)
		self.video_startstop_button = EnhancedButton(parent=self.commands_maintab, checkable=True,
													 labels=("Start Video", "Stop Video"), fixed_size=(150, 60),
													 layout=self.commands_r2_hl, property_class="commandsbutton", callback_function=self.video_start_stop_callback, ui=self)

		self.download_button = EnhancedButton(parent=self.commands_maintab, label="Download", fixed_size=(150, 60),
											  layout=self.commands_r2_hl, property_class="commandsbutton", callback_function=self.download_callback, ui=self)
		self.commands_vl.addLayout(self.commands_r2_hl)

		# commands r3
		commands_r3_vspacer = QtWidgets.QSpacerItem(10, 10, QtWidgets.QSizePolicy.Policy.Minimum,
													QtWidgets.QSizePolicy.Policy.MinimumExpanding)
		self.commands_vl.addItem(commands_r3_vspacer)
		self.commands_r3_hl = QtWidgets.QHBoxLayout()
		self.calibrate_button = EnhancedButton(parent=self.commands_maintab, label="Calibrate", fixed_size=(150, 30),
											   layout=self.commands_r3_hl, property_class="commandsbutton", callback_function=self.calibrate_callback, ui=self)
		self.reboot_button = EnhancedButton(parent=self.commands_maintab, label="Reboot", fixed_size=(150, 30),
											layout=self.commands_r3_hl, property_class="commandsbutton", callback_function=self.reboot_callback, ui=self)
		self.rth_button = EnhancedButton(parent=self.commands_maintab, label="RTH", fixed_size=(150, 30),
										 layout=self.commands_r3_hl, property_class="commandsbutton", callback_function=self.rth_callback, ui=self)

		commands_r4_vspacer = QtWidgets.QSpacerItem(10, 10, QtWidgets.QSizePolicy.Policy.Minimum,
													QtWidgets.QSizePolicy.Policy.Minimum)
		self.commands_vl.addItem(commands_r4_vspacer)
		self.commands_vl.addLayout(self.commands_r3_hl)

		self.commands_secondarytab = QtWidgets.QWidget()
		self.commands_tabs.addTab(self.commands_secondarytab, "Secondary Commands")

		self.secondary_commands_vl = QtWidgets.QVBoxLayout(self.commands_secondarytab)
		self.secondary_commands_vl.setContentsMargins(5, 5, 5, 5)

		self.secondary_commands_title_label = TitleLabel(layout=self.secondary_commands_vl, text="Secondary Commands",
														 parent=self.commands_secondarytab)

		self.secondary_commands_r1_hl = QtWidgets.QHBoxLayout()
		self.secondary_commands_r1_hl.setContentsMargins(5, 5, 5, 5)

		self.camera_reset_button = EnhancedButton(parent=self.commands_secondarytab, label="Reset Camera",
												  fixed_size=(150, 80), layout=self.secondary_commands_r1_hl,
												  property_class="commandsbutton")

		self.gimbal_calibrate_button = EnhancedButton(parent=self.commands_secondarytab, label="Calibrate Gimbal",
													  fixed_size=(150, 80), layout=self.secondary_commands_r1_hl,
													  property_class="commandsbutton")

		self.gimbal_reset_button = EnhancedButton(parent=self.commands_secondarytab, label="Reset Gimbal",
												  fixed_size=(150, 80), layout=self.secondary_commands_r1_hl,
												  property_class="commandsbutton")

		self.secondary_commands_vl.addLayout(self.secondary_commands_r1_hl)

		self.secondary_commands_r2_hl = QtWidgets.QHBoxLayout()
		self.secondary_commands_r2_hl.setContentsMargins(5, 5, 5, 5)

		self.skycontroller_offboard_button = EnhancedButton(parent=self.commands_secondarytab, label="Offboard",
															fixed_size=(150, 40), layout=self.secondary_commands_r2_hl,
															property_class="commandsbutton")

		self.storage_format_button = EnhancedButton(parent=self.commands_secondarytab, label="Storage Format",
													fixed_size=(150, 40), layout=self.secondary_commands_r2_hl,
													property_class="commandsbutton")

		self.secondary_commands_vl.addLayout(self.secondary_commands_r2_hl)

	def buildParametersSection(self, parent:QtWidgets.QWidget):


		# parameters tab 1

		self.parameters_tabs = QtWidgets.QTabWidget(parent)
		self.parameters_tabs.setMovable(True)
		self.parameters_tabs.setProperty("class", "parameterspane")
		self.parameters_tabs.tabBar().setProperty("class", "parameterstab")

		self.parametersmain_tabw = QtWidgets.QWidget()

		self.parameters_tabs.addTab(self.parametersmain_tabw, "Parameters")

		self.commands_parameters_hl.addWidget(self.parameters_tabs)
		self.parameters_vl_maintab = QtWidgets.QVBoxLayout(self.parametersmain_tabw)

		self.parameters_title_label = TitleLabel(layout=self.parameters_vl_maintab, text="Parameters",
												 parent=self.parametersmain_tabw)

		# parameters r1
		self.parameters_r1_hl = QtWidgets.QHBoxLayout()

		self.parameter_hdr = CheckBoxGroupBox(label="HDR", parent=self.parametersmain_tabw, is_checked=True)
		self.parameter_hdr.setup(self.parameters_r1_hl)

		self.parameter_autorecord = CheckBoxGroupBox(label="Auto Record", parent=self.parametersmain_tabw)
		self.parameter_autorecord.setup(self.parameters_r1_hl)

		self.parameter_bankedturn = CheckBoxGroupBox(label="Banked Turn", parent=self.parametersmain_tabw)
		self.parameter_bankedturn.setup(self.parameters_r1_hl)

		self.parameter_rthautotrigger = CheckBoxGroupBox(label="RTH Auto Trigger", parent=self.parametersmain_tabw)
		self.parameter_rthautotrigger.setup(self.parameters_r1_hl)

		self.parameters_vl_maintab.addLayout(self.parameters_r1_hl)
		self.parameter_cameraoperated = CheckBoxGroupBox(label="Camera Operated", parent=self.parametersmain_tabw)
		self.parameter_cameraoperated.setup(self.parameters_r1_hl)
		self.parameters_r2_hl = QtWidgets.QHBoxLayout()
		self.parameters_r2_hl.setObjectName("parameters_r2_hl")

		# camera mode
		self.camera_mode = ComboBoxGroupBox(layout=self.parameters_r2_hl, label="Camera Mode",
											choices=['Recording', 'Photo'],
											parent=self.parametersmain_tabw, property_class="parametergb")

		self.ev_compensation = ComboBoxGroupBox(layout=self.parameters_r2_hl, label="EV Compensation",
													   choices=['-3.00', '-2.00', '-1.00', '0.00', '1.00', '2.00',
																'3.00'],
													   parent=self.parametersmain_tabw, default_index=3,
													   property_class="parametergb")

		# image style
		self.image_style = ComboBoxGroupBox(layout=self.parameters_r2_hl, label="Image Style",
											property_class="parametergb",
											choices=['Natural Look', 'Flat and Desaturated', 'Intense', 'Pastel'],
											parent=self.parametersmain_tabw)

		self.parameters_vl_maintab.addLayout(self.parameters_r2_hl)
		# end r2

		# r3
		self.parameters_r3_hl = QtWidgets.QHBoxLayout()

		# max speed
		self.max_speed = SliderGroupBox(property_class="parametergb", parent=self.commands_parameters_hlw,
										label="Max Speed", min_value=0, max_value=100, value=0, suffix="m/s",
										layout=self.parameters_r3_hl, minimum_slider_width=200)

		# max tilt
		self.max_tilt = SliderGroupBox(property_class="parametergb", parent=self.commands_parameters_hlw,
									   label="Max Tilt", min_value=0, max_value=100, value=22, suffix="°",
									   layout=self.parameters_r3_hl, minimum_slider_width=200)

		self.parameters_vl_maintab.addLayout(self.parameters_r3_hl)

		# downloads row
		self.download_folder_gb = QtWidgets.QGroupBox(self.commands_parameters_hlw)
		self.download_folder_gb.setProperty("class", "parametergb")
		self.download_folder_hl = QtWidgets.QHBoxLayout(self.download_folder_gb)
		self.download_folder_hl.setContentsMargins(5, 5, 5, 5)
		self.download_folder_label = QtWidgets.QLabel(self.download_folder_gb)
		self.download_folder_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
		self.download_folder_label.setText("Download Folder")
		self.download_folder_label.setWordWrap(True)
		self.download_folder_label.setProperty("class", "parameterlabel")
		self.download_folder_hl.addWidget(self.download_folder_label, 0,
										  QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignVCenter)
		self.path_label = QtWidgets.QLabel(self.download_folder_gb)
		self.path_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
		self.path_label.setWordWrap(True)
		self.path_label.setText("<path>")
		self.path_label.setProperty("class", "ilabel")
		self.download_folder_hl.addWidget(self.path_label, 0,
										  QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignVCenter)
		self.parameters_vl_maintab.addWidget(self.download_folder_gb)

		#  parameters camera tab 2
		self.camera_tabw = QtWidgets.QWidget()
		self.parameters_tabs.addTab(self.camera_tabw, "Camera Parameters")
		self.camera_tab_vl = QtWidgets.QVBoxLayout(self.camera_tabw)

		self.camera_parameters_title_label = TitleLabel(parent=self.camera_tabw, text="Camera Parameters",
														layout=self.camera_tab_vl)

		# camera r1
		self.camera_parameters_r1_hl = QtWidgets.QHBoxLayout()

		# hdr
		self.camera_hdr = CheckBoxGroupBox(parent=self.camera_tabw, label="HDR", is_checked=True)
		self.camera_hdr.setup(self.camera_parameters_r1_hl)

		# autorecord
		# self.camera_autorecord_gb = QtWidgets.QGroupBox(self.camera_tabw)
		# self.camera_autorecord_gb.setProperty("class", "parametergb")
		# self.camera_autorecord_hl = QtWidgets.QHBoxLayout(self.camera_autorecord_gb)
		# self.camera_autorecord_hl.setContentsMargins(5, 5, 5, 5)
		# self.camera_autorecord_label = QtWidgets.QLabel(self.camera_autorecord_gb)
		# self.camera_autorecord_label.setText("Autorecord: ")
		# self.camera_autorecord_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
		# self.camera_autorecord_label.setWordWrap(True)
		# self.camera_autorecord_label.setProperty("class", "parameterlabel")
		# self.camera_autorecord_hl.addWidget(self.camera_autorecord_label, 0,
		#                                     QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignVCenter)
		# self.camera_autorecord_rbutton = QtWidgets.QRadioButton(self.camera_autorecord_gb)
		# self.camera_autorecord_hl.addWidget(self.camera_autorecord_rbutton, 0,
		#                                     QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignVCenter)
		# self.camera_parameters_r1_hl.addWidget(self.camera_autorecord_gb)
		self.camera_autorecord = CheckBoxGroupBox(parent=self.camera_tabw, label="Autorecord")
		self.camera_autorecord.setup(self.camera_parameters_r1_hl)

		# disparity image
		# self.camera_disparity_image_gb = QtWidgets.QGroupBox(self.camera_tabw)
		# self.camera_disparity_image_gb.setProperty("class", "parametergb")
		# self.camera_disparity_image_hl = QtWidgets.QHBoxLayout(self.camera_disparity_image_gb)
		# self.camera_disparity_image_hl.setContentsMargins(5, 5, 5, 5)
		# self.camera_disparity_image_label = QtWidgets.QLabel(self.camera_disparity_image_gb)
		# self.camera_disparity_image_label.setText("Disparity\nImage:")
		# self.camera_disparity_image_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
		# self.camera_disparity_image_label.setWordWrap(True)
		# self.camera_disparity_image_label.setProperty("class", "parameterlabel")
		# self.camera_disparity_image_hl.addWidget(self.camera_disparity_image_label, 0,
		#                                          QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignVCenter)
		# self.camera_disparity_image_rbutton = QtWidgets.QRadioButton(self.camera_disparity_image_gb)
		# self.camera_disparity_image_hl.addWidget(self.camera_disparity_image_rbutton, 0,
		#                                          QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignVCenter)
		# self.camera_parameters_r1_hl.addWidget(self.camera_disparity_image_gb)
		self.camera_disparity_image = CheckBoxGroupBox(parent=self.camera_tabw, label="Disparity Image")
		self.camera_disparity_image.setup(self.camera_parameters_r1_hl)

		# relative
		# self.camera_relative_gb = QtWidgets.QGroupBox(self.camera_tabw)
		# self.camera_relative_gb.setProperty("class", "parametergb")
		# self.camera_relative_hl = QtWidgets.QHBoxLayout(self.camera_relative_gb)
		# self.camera_relative_hl.setContentsMargins(5, 5, 5, 5)
		# self.camera_relative_label = QtWidgets.QLabel(self.camera_relative_gb)
		# self.camera_relative_label.setText("Relative:")
		# self.camera_relative_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
		# self.camera_relative_label.setWordWrap(True)
		# self.camera_relative_label.setProperty("class", "parameterlabel")
		# self.camera_relative_hl.addWidget(self.camera_relative_label, 0,
		#                                   QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignVCenter)
		# self.camera_relative_rbutton = QtWidgets.QRadioButton(self.camera_relative_gb)
		# self.camera_relative_hl.addWidget(self.camera_relative_rbutton, 0,
		#                                   QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignVCenter)
		# self.camera_parameters_r1_hl.addWidget(self.camera_relative_gb)
		self.camera_relative = CheckBoxGroupBox(parent=self.camera_tabw, label="Relative")
		self.camera_relative.setup(self.camera_parameters_r1_hl)
		self.camera_tab_vl.addLayout(self.camera_parameters_r1_hl)
		# end camera r1

		# camera r2
		self.camera_parameters_r2_hl = QtWidgets.QHBoxLayout()

		# camera mode
		self.camera_camera_mode = ComboBoxGroupBox(parent=self.camera_tabw, label="Camera Mode",
											choices=['Recording', 'Photo'],
											property_class="parametergb", layout=self.camera_parameters_r2_hl)

		# ev compensation
		self.camera_ev_compensation = ComboBoxGroupBox(parent=self.camera_tabw, label="EV Compensation",
													   choices=['-3.00', '-2.00', '-1.00', '0.00', '1.00', '2.00',
																'3.00'],
													   property_class="parametergb",
													   layout=self.camera_parameters_r2_hl)

		# image style
		self.camera_image_style = ComboBoxGroupBox(parent=self.camera_tabw, label="Image Style",
												   property_class="parametergb",
												   choices=['Natural Look', 'Flat and Desaturated', 'Intense',
															'Pastel'],
												   layout=self.camera_parameters_r2_hl)

		self.camera_tab_vl.addLayout(self.camera_parameters_r2_hl)
		# end camera r2

		# camera r3
		self.camera_parameters_r3_hl = QtWidgets.QHBoxLayout()

		# streaming
		self.camera_streaming = ComboBoxGroupBox(parent=self.camera_tabw, label="Streaming",
												 property_class="parametergb",
												 choices=['Minimise Latency with Average Reliability',
														  'Maximise Reliability with Average Latency',
														  'Maximise Reliability using Frame Rate Decimation'],
												 layout=self.camera_parameters_r3_hl)

		# rendering
		self.camera_rendering = ComboBoxGroupBox(parent=self.camera_tabw, label="Rendering",
												 property_class="parametergb",
												 choices=['Visible', 'Thermal', 'Blended'],
												 layout=self.camera_parameters_r3_hl)

		self.camera_tab_vl.addLayout(self.camera_parameters_r3_hl)
		# end r3

		# camera r4
		self.camera_parameters_r4_hl = QtWidgets.QHBoxLayout()

		# max zoom speed

		self.max_zoom_speed = SliderGroupBox(property_class="parametergb", parent=self.camera_tabw,
											 margin=(10, 0, 10, 0),
											 label="Maximum Zoom Speed", min_value=0.1, max_value=10.0, step=0.1,
											 value=10.0, suffix="°/s", layout=self.camera_parameters_r4_hl,
											 minimum_slider_width=300, minimum_height=60)
		self.camera_tab_vl.addLayout(self.camera_parameters_r4_hl)

		# end r4

		self.drone_tab1w = QtWidgets.QWidget()
		self.parameters_tabs.addTab(self.drone_tab1w, "Drone Parameters 1")
		self.drone_tab1_vl = QtWidgets.QVBoxLayout(self.drone_tab1w)

		self.drone_parameters1_title_label = TitleLabel(layout=self.drone_tab1_vl, text="Drone Parameters 1",
														parent=self.drone_tab1w)

		vs = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Expanding)
		self.drone_tab1_vl.addItem(vs)

		# drone r1
		self.drone_parameters_r1_hl = QtWidgets.QHBoxLayout()

		self.drone_bankedturn = CheckBoxGroupBox(parent=self.drone_tab1w, label="Banked Turn", h_expand=0, v_expand=0,
												 expand_policy=(QtWidgets.QSizePolicy.Policy.Minimum,
																QtWidgets.QSizePolicy.Policy.Minimum))
		self.drone_bankedturn.setup(self.drone_parameters_r1_hl)

		# max yaw rate
		self.max_yaw_rate = SliderGroupBox(property_class="droneparametergb", parent=self.drone_tab1w,
										   label="Maximum Yaw Rate", min_value=3.0, max_value=200.0, step=0.1,
										   value=180.0, suffix="°/s", minimum_slider_width=200,
										   layout=self.drone_parameters_r1_hl)

		self.drone_tab1_vl.addLayout(self.drone_parameters_r1_hl)

		# end r1

		vs = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Expanding)
		self.drone_tab1_vl.addItem(vs)

		self.drone_parameters_r2_hl = QtWidgets.QHBoxLayout()

		# max altitude
		self.drone_max_altitude = SliderGroupBox(property_class="droneparametergb", parent=self.drone_tab1w,
												 label="Maximum Altitude", min_value=0.5, max_value=4000.0, step=0.1,
												 value=2.0, suffix="m", minimum_slider_width=200,
												 layout=self.drone_parameters_r2_hl)

		# max distance
		self.max_distance = SliderGroupBox(property_class="droneparametergb", parent=self.drone_tab1w,
										   label="Maximum Distance", min_value=10.0, max_value=4000.0, step=0.1,
										   value=10.0, suffix="m", minimum_slider_width=200,
										   layout=self.drone_parameters_r2_hl)

		self.drone_tab1_vl.addLayout(self.drone_parameters_r2_hl)

		vs = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Expanding)
		self.drone_tab1_vl.addItem(vs)

		# end r2

		self.drone_tab2w = QtWidgets.QWidget()
		self.parameters_tabs.addTab(self.drone_tab2w, "Drone Parameters 2")
		self.drone_tab2_vl = QtWidgets.QVBoxLayout(self.drone_tab2w)
		self.drone_parameters2_title_label = TitleLabel(layout=self.drone_tab2_vl, text="Drone Parameters 2",
														parent=self.drone_tab2w)

		vs = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Expanding)
		self.drone_tab2_vl.addItem(vs)

		# drone2 r1
		self.drone_parameters2_r1_hl = QtWidgets.QHBoxLayout()

		# max horizontal speed
		self.max_horizontal_speed = SliderGroupBox(property_class="droneparametergb", parent=self.drone_tab2w,
												   label="Maximum Horizontal Speed", min_value=0.1, max_value=15.0,
												   step=0.1, value=1.0, suffix="m/s", minimum_slider_width=200,
												   layout=self.drone_parameters2_r1_hl)

		# max pitch roll
		self.max_pitch_roll = SliderGroupBox(property_class="droneparametergb", parent=self.drone_tab2w,
											 label="Maximum Pitch Roll", min_value=1, max_value=40, value=10,
											 suffix="°", layout=self.drone_parameters2_r1_hl, minimum_slider_width=200)

		self.drone_tab2_vl.addLayout(self.drone_parameters2_r1_hl)
		# end r1
		vs = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Expanding)
		self.drone_tab2_vl.addItem(vs)
		# drone2 r2
		self.drone_parameters2_r2_hl = QtWidgets.QHBoxLayout()

		# max vertical speed

		self.max_vertical_speed = SliderGroupBox(property_class="droneparametergb", parent=self.drone_tab2w,
												 label="Maximum Vertical Speed", min_value=0, max_value=4, step=1,
												 value=1, suffix="m/s", layout=self.drone_parameters2_r2_hl,
												 minimum_slider_width=200)
		# max pitch roll rate
		self.max_pitch_roll_rate = SliderGroupBox(property_class="droneparametergb", parent=self.drone_tab2w,
												  label="Maximum Pitch Roll Rate", min_value=40.0, max_value=300.0,
												  step=0.1, value=200.0, suffix="°/s", minimum_slider_width=200,
												  layout=self.drone_parameters2_r2_hl)

		self.drone_tab2_vl.addLayout(self.drone_parameters2_r2_hl)
		vs = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Expanding)
		self.drone_tab2_vl.addItem(vs)
		# end r2


		## pair setup
		self.camera_hdr.setPairs([self.parameter_hdr])
		self.parameter_hdr.setPairs([self.camera_hdr])

		self.camera_autorecord.setPairs([self.parameter_autorecord])
		self.parameter_autorecord.setPairs([self.camera_autorecord])

		self.drone_bankedturn.setPairs([self.parameter_bankedturn])
		self.parameter_bankedturn.setPairs([self.drone_bankedturn])

		self.camera_mode.setPairs([self.camera_camera_mode])
		self.camera_camera_mode.setPairs([self.camera_mode])

		self.camera_ev_compensation.setPairs([self.ev_compensation])
		self.ev_compensation.setPairs([self.camera_ev_compensation])

		self.image_style.setPairs([self.camera_image_style])
		self.camera_image_style.setPairs([self.image_style])


	# Primary Commands
	def arm_disarm_callback(self):
		if self.arm_disarm_button.isChecked():
			self.node.msg_keyboard_action.data = 1 # arm
		else:
			self.node.msg_keyboard_action.data = 5 # disarm
		
		self.node.pub_keyboard_action.publish(self.node.msg_keyboard_action)

	def takeoff_land_callback(self):
		if self.takeoff_land_button.isChecked():
			self.node.msg_keyboard_action.data = 2 # takeoff
		else:
			self.node.msg_keyboard_action.data = 4 # land
		self.node.pub_keyboard_action.publish(self.node.msg_keyboard_action)
	
	def halt_callback(self):
		self.node.msg_keyboard_action.data = 3
		self.node.pub_keyboard_action.publish(self.node.msg_keyboard_action)

	def photo_start_stop_callback(self):
		self.node.msg_keyboard_camera_command = KeyboardCameraCommand()
		if self.photo_startstop_button.isChecked():
			self.node.msg_keyboard_camera_command.action = 1 # start photo
		else:
			self.node.msg_keyboard_camera_command.action = 0  # TODO: input correct data for stop photo
		self.node.pub_keyboard_camera_command.publish(self.node.msg_keyboard_camera_command)

	def video_start_stop_callback(self):
		self.node.msg_keyboard_camera_command = KeyboardCameraCommand()
		if self.video_startstop_button.isChecked():
			self.node.msg_keyboard_camera_command.action = 2 # start recording
		else:
			self.node.msg_keyboard_camera_command.action = 3 # stop recording
		self.node.pub_keyboard_camera_command.publish(self.node.msg_keyboard_camera_command)


	def download_callback(self):
		self.node.msg_keyboard_camera_command = KeyboardCameraCommand()
		self.node.msg_keyboard_camera_command.action = 4
		self.node.pub_keyboard_camera_command.publish(self.node.msg_keyboard_camera_command)
	

	def calibrate_callback(self):
		self.node.msg_keyboard_action.data = 111
		self.node.pub_keyboard_action.publish(self.node.msg_keyboard_action)
	
	def reboot_callback(self):
		self.node.msg_keyboard_action.data = 110
		self.node.pub_keyboard_action.publish(self.node.msg_keyboard_action)
	
	def rth_callback(self):
		self.node.msg_keyboard_action.data = 7
		self.node.pub_keyboard_action.publish(self.node.msg_keyboard_action) 



class MainWindow(QtWidgets.QMainWindow):

	def __init__(self, node) -> None:
		super().__init__()
		self.node = node

	def keyPressEvent(self, a0: QtGui.QKeyEvent) -> None:
		self.node.keyboard_event(a0)
		return super().keyPressEvent(a0)

def main(args=None):
	import sys
	rclpy.init(args=sys.argv)
	
	anafi_gui = AnafiGUI()

	app = QtWidgets.QApplication(sys.argv)
	screen = app.primaryScreen()
	stylesheet = open("/home/eee/ros2_ws/src/anafi_gui/anafi_gui/GUI/stylesheet.qss", "r").read()
	app.setStyleSheet(stylesheet)
	main_window = MainWindow(node=anafi_gui)
	ui = UI(main_window, screen.size(), anafi_gui)

	anafi_gui.set_ui(ui)

	# MainWindow.showFullScreen()
	main_window.show()

	#rclpy.spin(anafi_gui.node)

	# ui.media_player.play()
	sys.exit(app.exec())
	# help(node.keyboard_event())

if __name__ == '__main__':
	main()
