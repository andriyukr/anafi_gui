import sys
import typing
import time
from PyQt6 import QtCore, QtGui
from PyQt6.QtMultimedia import *
from PyQt6.QtMultimediaWidgets import *
from PyQt6.QtCore import *
from PyQt6.QtWidgets import *
from anafi_gui.custom_widgets import *
from anafi_gui.ui import *
import rclpy
from threading import Thread
from std_msgs.msg import String

from timeit import default_timer as timer
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters, GetParameters
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
		self.node = rclpy.create_node("anafi_gui")
		self.node.get_logger().info("ANAFI GUI is running...")
		
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
		self.node.create_subscription(QuaternionStamped, 'drone/attitude', self.attitude_callback, qos_profile_sensor_data)
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


		# Services
		self.get_anafi_parameters_client = self.node.create_client(GetParameters, 'anafi/get_parameters')
		#while not self.get_anafi_parameters_client.wait_for_service(timeout_sec=1.0):
		#	self.node.get_logger().info('No connection to anafi')
		self.set_anafi_parameters_client = self.node.create_client(SetParameters, 'anafi/set_parameters')
		#while not self.set_anafi_parameters_client.wait_for_service(timeout_sec=1.0):
		#	self.node.get_logger().info('No connection to anafi')

		self.get_safe_anafi_parameters_client = self.node.create_client(GetParameters, 'safe_anafi/get_parameters')
		#while not self.get_safe_anafi_parameters_client.wait_for_service(timeout_sec=1.0):
		#	self.node.get_logger().info('No connection to safe_anafi')
		self.set_safe_anafi_parameters_client = self.node.create_client(SetParameters, 'safe_anafi/set_parameters')
		#while not self.set_safe_anafi_parameters_client.wait_for_service(timeout_sec=1.0):
		#	self.node.get_logger().info('No connection to safe_anafi')
		
		# Messages
		# self.msg_camera_command = CameraCommand()
		# self.msg_drone_command = PilotingCommand()
		self.msg_moveto_command = MoveToCommand()
		self.msg_moveby_command = MoveByCommand()
		self.msg_gimbal_command = GimbalCommand()
		self.msg_keyboard_drone_command = KeyboardDroneCommand()
		self.msg_keyboard_camera_command = KeyboardCameraCommand()
		self.msg_keyboard_action = UInt8()

		self.timer_thread = Thread(target=self.parameters_callback)
		self.timer_thread.start()

	def parameters_callback(self):
		while rclpy.ok():
			req = GetParameters.Request()
			req.names = ['drone/model']
			future = self.get_anafi_parameters_client.call_async(req)
			while not future.done():
				time.sleep(0.1)
				if not rclpy.ok():
					return
			drone_model = future.result().values[0].string_value
			self.ui.model_info.setValue(drone_model)

			req = GetParameters.Request()
			req.names = ['drone/max_horizontal_speed']
			future = self.get_anafi_parameters_client.call_async(req)
			while not future.done():
				time.sleep(0.1)
				if not rclpy.ok():
					return
			max_speed = future.result().values[0].double_value
			self.ui.max_speed.slider.valueLabel.setValue(max_speed)

			req = GetParameters.Request()
			req.names = ['armed']
			future = self.get_safe_anafi_parameters_client.call_async(req)
			while not future.done():
				time.sleep(0.1)
				if not rclpy.ok():
					return
			armed = future.result().values[0].bool_value
			self.ui.arm_disarm_button.setText("Disarm" if armed else "Arm")

			time.sleep(0.01)
		
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
		self.ui.altitude_info.setValue(msg.data)
	
	def position_callback(self, msg):
		pass
	
	def local_position_callback(self, msg):
		pass
	
	def speed_callback(self, msg):
		self.ui.speed_info.setValue("[%.1f, %.1f, %.1f]" % (msg.vector.x, msg.vector.y, msg.vector.z))
	
	def goodput_callback(self, msg):
		pass
	
	def quality_callback(self, msg):
		self.ui.link_quality_info.setValue(msg.data)
	
	def rssi_callback(self, msg):
		pass

	def percentage_callback(self, msg):
		self.ui.battery_info.setValue(msg.data)

	def state_callback(self, msg):
		self.ui.status_info.setValue(msg.data)
		
	def rpy_callback(self, msg):
		self.ui.attitude_info.setValue("[%.1f, %.1f, %.1f]" % (msg.vector.x, msg.vector.y, msg.vector.z))

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
		pass

	def steady_callback(self, msg):
		pass

	def health_callback(self, msg):
		pass

	# Parameter Callbacks
	def max_speed_callback(self):
		max_speed = self.ui.max_speed.slider.valueLabel.value()

		req = SetParameters.Request()
		req.parameters = [
			Parameter(name='drone/max_horizontal_speed', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=max_speed)),
			Parameter(name='drone/max_vertical_speed', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=max(max_speed, 4.0)))]
		future = self.set_anafi_parameters_client.call_async(req)
		# while not future.done():
		# 	time.sleep(0.1)

class MainWindow(QtWidgets.QMainWindow):

	def __init__(self, node) -> None:
		super().__init__()
		self.node = node

	def closeEvent(self, event):
		self.node.node.get_logger().info("ANAFI GUI is stopping...")
		self.node.node.destroy_node()
		rclpy.shutdown()
		#self.node.timer_thread.join()

	def keyPressEvent(self, a0: QtGui.QKeyEvent) -> None:
		self.node.keyboard_event(a0)
		return super().keyPressEvent(a0)

def main(args=None):
	import sys
	rclpy.init(args=sys.argv)
	
	anafi_gui = AnafiGUI()

	app = QtWidgets.QApplication(sys.argv)
	screen = app.primaryScreen()
	stylesheet = open(get_package_share_directory('anafi_gui') + "/stylesheet.qss", "r").read()
	app.setStyleSheet(stylesheet)
	main_window = MainWindow(node=anafi_gui)
	#ui = UI(main_window, screen.size(), anafi_gui)
	window_size = QSize(1600, 900)
	ui = UI(main_window, window_size, anafi_gui)

	anafi_gui.set_ui(ui)

	# MainWindow.showFullScreen()
	main_window.show()

	spin_thread = Thread(target=rclpy.spin, args=(anafi_gui.node,))
	spin_thread.start()

	# ui.media_player.play()
	results = app.exec()

	spin_thread.join()
	anafi_gui.timer_thread.join()

	sys.exit(results)
	# help(node.keyboard_event())

if __name__ == '__main__':
	main()
