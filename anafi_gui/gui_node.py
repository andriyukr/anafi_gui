import sys
import typing
import time
import numpy as np
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
#from anafi_autonomy.msg import KeyboardCameraCommand, KeyboardDroneCommand
from anafi_autonomy.msg import KeyboardCommand

class Worker1(QObject):
	#Creating PyQt Signal to emit Pic
	ImageUpdate=pyqtSignal(QImage)
	
	def __init__(self,node):
		super().__init__()
		self.node=node

	def run(self):
		#print(self.node.Img)
		ConvertToQtFormat = QImage(self.node.Img.data, self.node.Img.shape[1], self.node.Img.shape[0], QImage.Format.Format_RGB888) #Convert Open CV image into a QImage
		Pic=ConvertToQtFormat.scaled(740,500,Qt.AspectRatioMode.KeepAspectRatioByExpanding)
		self.ImageUpdate.emit(Pic) 
		#print('Emit')

class Worker2(QObject):
	ModelUpdate=pyqtSignal(str)
	def __init__(self,node):
		super().__init__()
		self.node=node

	def run(self):

			drone_model_text=self.node.drone_model
			self.ModelUpdate.emit(drone_model_text) 
			

class AnafiGUI(Node):
	def __init__(self):
		self.node = rclpy.create_node("anafi_gui")
		self.node.get_logger().info("ANAFI GUI is running...")
		self.ui = None
		self.Img=np.zeros((100,100,3), dtype=np.uint8) #Initializing Empty Image 
		self.drone_model=None

		
	# 	# publishers
		self.pub_camera_command = self.node.create_publisher(CameraCommand, 'camera/command', qos_profile_system_default)
		self.pub_drone_command = self.node.create_publisher(PilotingCommand, 'drone/command', qos_profile_system_default)
		self.pub_moveto_command = self.node.create_publisher(MoveToCommand, 'drone/moveto', qos_profile_system_default)
		self.pub_moveby_command = self.node.create_publisher(MoveByCommand, 'drone/moveby', qos_profile_system_default)
		self.pub_gimbal_command = self.node.create_publisher(GimbalCommand, 'gimbal/command', qos_profile_system_default)
		#self.pub_keyboard_action = self.node.create_publisher(UInt8, 'keyboard/action', qos_profile_system_default)
		#self.pub_keyboard_camera_command = self.node.create_publisher(KeyboardCameraCommand, 'keyboard/camera_command', qos_profile_system_default)
		#self.pub_keyboard_drone_command = self.node.create_publisher(KeyboardDroneCommand, 'keyboard/drone_command', qos_profile_system_default)
		self.pub_keyboard_command = self.node.create_publisher(KeyboardCommand, 'keyboard/command', qos_profile_system_default)

		
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

		self.node.create_subscription(UInt8, 'drone/gps/satellites', self.gps_satellite_callback, qos_profile_system_default)
		self.node.create_subscription(Float32, 'camera/zoom', self.zoom_callback, qos_profile_system_default)
		self.node.create_subscription(NavSatFix, 'drone/gps/location', self.gps_location_callback, qos_profile_system_default)
		self.node.create_subscription(Vector3Stamped, 'gimbal/attitude/absolute', self.gimbal_attitude_callback, qos_profile_system_default)
		self.node.create_subscription(PilotingCommand , 'drone/command',self.drone_command_callback ,qos_profile_system_default)
		self.node.create_subscription(MoveByCommand, 'drone/moveby',self.move_by_callback, qos_profile_system_default)
		self.node.create_subscription(MoveToCommand, 'drone/moveto', self.move_to_callback,qos_profile_system_default)
		self.node.create_subscription(UInt64, 'storage/available', self.storage_available_callback,qos_profile_system_default)
		
		self.br=CvBridge()
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
		#self.msg_keyboard_command = KeyboardDroneCommand()
		#self.msg_keyboard_command= KeyboardCameraCommand()
		self.msg_keyboard_command = KeyboardCommand()
		#self.msg_keyboard_command = UInt8()

		self.timer_thread = Thread(target=self.parameters_callback)
		self.timer_thread.start()

		self.timer_thread2 = Thread(target=self.set_parameters_callback)
		self.timer_thread2.start()

	def parameters_callback(self):
		while rclpy.ok():
			req = GetParameters.Request()
			req.names = ['drone/model']
			future = self.get_anafi_parameters_client.call_async(req)
			while not future.done():
				time.sleep(0.1)
				if not rclpy.ok():
					return
			self.drone_model = future.result().values[0].string_value
			self.ui.model_info.setValue(self.drone_model)

			req = GetParameters.Request()
			req.names = ['device/ip']
			future = self.get_anafi_parameters_client.call_async(req)
			while not future.done():
				time.sleep(0.1)
				if not rclpy.ok():
					return
			drone_ip = future.result().values[0].string_value
			self.ui.ip_info.setValue(drone_ip)

			req = GetParameters.Request()
			req.names = ['drone/offboard']
			future = self.get_anafi_parameters_client.call_async(req)
			while not future.done():
				time.sleep(0.1)
				if not rclpy.ok():
					return
			control_var = future.result().values[0].bool_value
			control_text_val="offboard" if control_var else "Manual"
			self.ui.control_info.setValue(control_text_val)

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
			self.node.get_logger()
			self.ui.arm_disarm_button.setText("Disarm" if armed else "Arm") #sets the Text
			self.interate_more_than_once=True
			#print(self.ui.camera_autorecord.isChecked())
			time.sleep(0.01)

	def set_parameters_callback(self):
		while rclpy.ok():
			print(self.ui.parameter_autorecord.checkBox.toggled.connect(self.ui.parameter_autorecord.checkBox.isChecked))
			#print("hi")
		time.sleep(100)
		
	def set_ui(self, ui):
		self.ui = ui


	def keyboard_event(self, key: QtGui.QKeyEvent):
		self.msg_keyboard_command = KeyboardCommand()
		
		# Keyboard/drone_command
		if key.key() == Qt.Key.Key_A:
			self.msg_keyboard_command.drone_yaw = 1
		elif key.key() == Qt.Key.Key_D:
			self.msg_keyboard_command.drone_yaw = -1
		elif key.key() == Qt.Key.Key_W:
			self.msg_keyboard_command.drone_z = 1
		elif key.key() == Qt.Key.Key_S:
			self.msg_keyboard_command.drone_z = -1 
		elif key.key() == Qt.Key.Key_Right:
			self.msg_keyboard_command.drone_y = -1 # move right
		elif key.key() == Qt.Key.Key_Left:
			self.msg_keyboard_command.drone_y = 1 # move left
		elif key.key() == Qt.Key.Key_Up:
			self.msg_keyboard_command.drone_x = 1
		elif key.key() == Qt.Key.Key_Down:
			self.msg_keyboard_command.drone_x = -1
		
		# Keyboard/Camera Command
		elif key.key() == Qt.Key.Key_5:
			self.msg_keyboard_command.camera_action = 11
		elif key.key() == Qt.Key.Key_9:
			self.msg_keyboard_command.gimbal_roll = 1
		elif key.key() == Qt.Key.Key_7:
			self.msg_keyboard_command.gimbal_roll = -1
		elif key.key() == Qt.Key.Key_2:
			self.msg_keyboard_command.gimbal_pitch = 1
		elif key.key() == Qt.Key.Key_8:
			self.msg_keyboard_command.gimbal_pitch = -1
		elif key.key() == Qt.Key.Key_6:
			self.msg_keyboard_command.gimbal_yaw = 1
		elif key.key() == Qt.Key.Key_4:
			self.msg_keyboard_command.gimbal_yaw = -1
		elif key.key() == Qt.Key.Key_Plus:
			self.msg_keyboard_command.zoom = 1
		elif key.key() == Qt.Key.Key_Minus:
			self.msg_keyboard_command.zoom = -1
		elif key.key() == Qt.Key.Key_Enter:
			self.msg_keyboard_command.camera_action = 1
		elif key.key() == Qt.Key.Key_0:
			self.msg_keyboard_command.camera_action = 2
		elif key.key() == Qt.Key.Key_Period:
			self.msg_keyboard_command.camera_action = 3
		elif key.key() == Qt.Key.Key_Slash:
			self.msg_keyboard_command.camera_action = 4
		elif key.key() == Qt.Key.Key_F12:
			self.msg_keyboard_command.camera_action = 111

		# Keyboard/action
		elif key.key() == Qt.Key.Key_Insert:
			self.msg_keyboard_command.drone_action = 1
		elif key.key() == Qt.Key.Key_T:
			self.msg_keyboard_command.drone_action = 2
		elif key.key() == Qt.Key.Key_Space:
			self.msg_keyboard_command.drone_action = 3				   
		elif key.key() == Qt.Key.Key_L:
			self.msg_keyboard_command.drone_action = 4
		elif key.key() == Qt.Key.Key_Escape:
			self.msg_keyboard_command.drone_action = 5			
		elif key.key() == Qt.Key.Key_R:
			self.msg_keyboard_command.drone_action = 6
		elif key.key() == Qt.Key.Key_B:
			self.msg_keyboard_command.drone_action = 7
		elif key.key() == Qt.Key.Key_F1:
			self.msg_keyboard_command.drone_action = 101
		elif key.key() == Qt.Key.Key_F2:
			self.msg_keyboard_command.drone_action = 102			
		elif key.key() == Qt.Key.Key_F4:
			self.msg_keyboard_command.drone_action = 110
		elif key.key() == Qt.Key.Key_F5:
			self.msg_keyboard_command.drone_action = 11
		elif key.key() == Qt.Key.Key_F6:
			self.msg_keyboard_command.drone_action = 12
		elif key.key() == Qt.Key.Key_F7:
			self.msg_keyboard_command.drone_action = 13
		elif key.key() == Qt.Key.Key_F9:
			self.msg_keyboard_command.drone_action = 21	
		elif key.key() == Qt.Key.Key_F11:
			self.msg_keyboard_command.drone_action = 111	
		
		self.node.get_logger().info('Publishing topic keyboard/action: "%s"' % self.msg_keyboard_command.drone_action)
		self.pub_keyboard_command.publish(self.msg_keyboard_command)

		
	def image_callback(self, msg):
		self.Img=self.br.imgmsg_to_cv2(msg, desired_encoding='rgb8')
	
	
	def camera_info_callback(self, msg):
		pass
	
	def time_callback(self, msg):
		pass
	
	def attitude_callback(self, msg):
		pass
	
	def altitude_callback(self, msg):
		self.ui.altitude_info.setValue(round(msg.data,2))
	
	def position_callback(self, msg):
		pass
	
	def local_position_callback(self, msg):
		pass
	
	def speed_callback(self, msg):
		self.ui.speed_info.setValue("[%.1f, %.1f, %.1f]" % (msg.vector.x, msg.vector.y, msg.vector.z))
	
	def goodput_callback(self, msg):
		pass
	
	def quality_callback(self, msg):
		self.ui.link_info.setValue(msg.data)
	
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

	def gimbal_attitude_callback(self,msg):
		self.ui.gimbal_attitude_info.setValue("[%.1f, %.1f, %.1f]" % (msg.vector.x, msg.vector.y, msg.vector.z))
	
	def drone_command_callback(self,msg):
		self.ui.drone_command_info.setValue("[%.1f, %.1f, %.1f, %.1f]" % (msg.roll,msg.pitch,msg.yaw,msg.gaz))
	
	def move_by_callback(self,msg):
		self.ui.move_by_info.setValue("[%.1f, %.1f, %.1f, %.1f]" % (msg.dx,msg.dy,msg.dz,msg.dyaw))
	
	def move_to_callback(self,msg):
		self.ui.move_to_info.setValue("[%.1f, %.1f, %.1f, %.1f, %d]" % (msg.latitude,msg.longitude,msg.altitude,msg.heading,msg.orientation_mode))

	def storage_available_callback(self,msg):
		self.ui.storage_info.setValue(msg.data)
	
	
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

		#Creating Worker Instance and passing the node object
		self.node.worker1=Worker1(self.node)
		self.node.worker2=Worker2(self.node)

		#Creating the thread
		self.worker_thread=QThread()
		self.worker2_thread=QThread()

		
		self.node.worker1.ImageUpdate.connect(self.ImageUpdateSlot)
		self.node.worker1.moveToThread(self.worker_thread)

		self.node.worker2.ModelUpdate.connect(self.ModelUpdateSlot)
		self.node.worker2.moveToThread(self.worker2_thread)

		#Starting the thread
		self.worker_thread.started.connect(self.node.worker1.run)
		self.worker_thread.start()

		self.worker2_thread.started.connect(self.node.worker2.run)
		self.worker2_thread.start()

		#Creating time to update image in the Worker class
		self.timer=QTimer()
		self.timer.start()
		self.timer.setInterval(33)
		self.timer.timeout.connect(self.node.worker1.run)

		self.timer2=QTimer()
		self.timer2.start()
		self.timer2.setInterval(1000)
		self.timer2.timeout.connect(self.node.worker2.run)
		
		#print(self.worker_thread.isRunning())
	

	#Slot to set the Image in the GUI 
	def ImageUpdateSlot(self, Image):
		self.node.ui.video_widget.setPixmap(QPixmap(Image))
		#self.node.node.get_logger().info('Receiving video frame')

	def ModelUpdateSlot(self):
		if self.node.drone_model=="ai":
			self.ButtonList=["RGB","Disparity"]
			self.CallbackDict={"RGB":self.rgb_callback,"Disparity":self.disparity_callback}
		elif (self.node.drone_model=="thermal") or (self.node.drone_model=="usa"):
			self.CallbackDict={"RGB":self.rgb_callback,"Thermal":self.thermal_callback,"Blended":self.blended_callback}
			self.ButtonList=["RGB","Thermal","Blended"]
		else:
			self.ButtonList=[]

		if len(self.ButtonList)>0:
			#generate buttons
			for i in self.ButtonList:
				EnhancedButton(parent=self.node.ui.vstream_gb, label=i, fixed_size=(60, 25), layout=self.node.ui.video_stream_hl, property_class="vsbutton",callback_function=self.CallbackDict[i])
	
			self.node.ui.video_buttons.addStretch()
			self.node.ui.video_stream_hl.addStretch()
			self.node.ui.video_buttons.addLayout(self.node.ui.video_stream_hl)
			self.node.ui.video_widget.setLayout(self.node.ui.video_buttons)
		if self.node.drone_model != None:
			self.timer2.stop()
			print(self.node.drone_model)


	def closeEvent(self, event):
		self.node.node.get_logger().info("ANAFI GUI is stopping...")
		self.node.node.destroy_node()
		rclpy.shutdown()
		#self.node.timer_thread.join()

	def keyPressEvent(self, a0: QtGui.QKeyEvent) -> None:
		self.node.keyboard_event(a0)
		return super().keyPressEvent(a0)

	def rgb_callback(self):
		req = SetParameters.Request()
		if (self.node.drone_model=="usa") or (self.node.drone_model=="thermal"):
			req.parameters = [Parameter(name='camera/thermal/rendering', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=0))]
		elif (self.node.drone_model=="ai"):
			req.parameters = [Parameter(name='camera/stereo/disparity_map', value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False))]
		future = self.node.set_anafi_parameters_client.call_async(req)
		print("RGB")

	def disparity_callback(self):
		req = SetParameters.Request()
		req.parameters = [Parameter(name='camera/stereo/disparity_map', value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True))]
		future = self.node.set_anafi_parameters_client.call_async(req)
		print("Disparity")

	def thermal_callback(self):
		req = SetParameters.Request()
		req.parameters = [Parameter(name='camera/thermal/rendering', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=1))]
		future = self.node.set_anafi_parameters_client.call_async(req)
		print("Thermal")

	def blended_callback(self):
		req = SetParameters.Request()
		req.parameters = [Parameter(name='camera/thermal/rendering', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=2))]
		future = self.node.set_anafi_parameters_client.call_async(req)
		print("Blended")
	


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

	#main_window.button_display()

	# ui.media_player.play()
	results = app.exec()

	spin_thread.join()
	anafi_gui.timer_thread.join()

	sys.exit(results)
	# help(node.keyboard_event())

if __name__ == '__main__':
	main()
