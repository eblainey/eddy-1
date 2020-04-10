from PyQt5 import QtWidgets, uic, QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtWebKit import *
from PyQt5.QtWebKitWidgets import *
from PyQt5.QtWebKitWidgets import QWebView
from PyQt5.QtGui import QPixmap, QMovie
import sys
# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import CompressedImage, ChannelFloat32, BatteryState
import cv2
import numpy as np
from std_msgs.msg import Float64, Bool, String

'''Progress:
    1. Import stylesheet
    2. Maximized window
    3. Camera input
    4. Close subscription on tab
Learning running loop
'''

class OwnImageWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(OwnImageWidget, self).__init__(parent)
        self.image = None
        self.setWindowTitle("DragonBall Control Window")

    def setImage(self, image):
        self.image = image
        sz = image.size()
        self.setMinimumSize(sz)
        self.update()

    def paintEvent(self, event):
        qp = QtGui.QPainter()
        qp.begin(self)
        if self.image:
            qp.drawImage(QtCore.QPoint(0, 0), self.image)
        qp.end()

class Ui(QtWidgets.QMainWindow):
    def __init__(self):
        super(Ui, self).__init__() # Call the inherited classes __init__ method
        uic.loadUi('mainwindow.ui', self)  # Load the .ui file
        #self.showMaximized() # Show full screen

        # Stylesheet import
        # sshFile = "MaterialDark.stylesheet"
        # with open(sshFile, "r") as fh:
        #     QtWidgets.qApp.setStyleSheet(fh.read())

        self.drive_progressBar.setValue(100)
        self.cpu_progressBar.setValue(100)

        # UI variables
        self.drv_battery_percent = 0
        self.cpu_battery_percent = 0
        self.pwr = 0
        self.speed = 0
        self.angle = 0

        self.view = self.webView
        self.view.load(QtCore.QUrl("http://localhost:5006/mapBokehEddy5"))

        # For window resizing.
        self.window_width = self.ImgWidget.frameSize().width()
        self.window_height = self.ImgWidget.frameSize().height()
        self.ImgWidget = OwnImageWidget(self.ImgWidget)
        self.html_width = self.view.frameSize().width()
        self.html_height = self.view.frameSize().height()
        print (self.html_width, self.html_height)


        self.camera_sub = rospy.Subscriber("eddy/camera/image_raw/compressed", CompressedImage, self.callback, queue_size=1)
        self.batteryDrive = rospy.Subscriber("/drive/voltage", Float64, self.voltsDrive, queue_size=10)
        self.batteryCPU = rospy.Subscriber("/mavros/battery", BatteryState, self.voltsCPU, queue_size=10)
        self.torquePwr = rospy.Subscriber("/torque/encoder", ChannelFloat32, self.flywheel, queue_size=10)
        self.odometer = rospy.Subscriber("/quadratureData", ChannelFloat32, self.odometry, queue_size=3)
        self.recorder = rospy.Subscriber("/rosbag/recording", Bool, self.recording_status, queue_size=3)
        self.message = rospy.Subscriber("/UI/notifications", String, self.ui_notification, queue_size=3)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(10)

    def callback(self, ros_data):
        np_arr = np.fromstring(ros_data.data, np.uint8)
        self.frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if self.frame is None:
            return
        self.video()

    def update_frame(self):
        if self.drv_battery_percent is not None:
            self.drive_progressBar.setValue(self.drv_battery_percent)
        if self.cpu_battery_percent is not None:
            self.cpu_progressBar.setValue(self.cpu_battery_percent)
        self.label_3.show()
        self.label_4.show()
        self.lcdNumber_3.display(self.pwr)
        self.lcdNumber_2.display(self.speed)

    def voltsDrive(self, battData):
        # For drive battery max is 25.2 and min 20.4 (@3.4 cells), therefore
        # 25.2 - 20.4 = 4.8 is 100%
        self.drv_battery_percent = (battData.data - 20.4)*100/4.8

    def voltsCPU(self, battData):
        # For drive battery max is 16.8 and min 13.6 (@3.4 cells), therefore
        # 16.8 - 13.6 = 3.2 is 100%
        self.cpu_battery_percent = (battData.voltage - 13.6) * 100 / 3.2

    def flywheel(self, encData):
        self.pwr = (120-abs(encData.values[1]))*100/120

    def odometry(self, Data):
        self.speed = Data.values[1]

    def tilt_angle(self, angleData):
        self.angle = angleData.data

    def recording_status(self, ros_data):
        if ros_data.data:
            self.label_4.setText("Recording in progress.")
            self.label_4.setStyleSheet('color: red')
        else:
            self.label_4.setText("Recording Stopped.")
            self.label_4.setStyleSheet('color: green')

    def ui_notification(self, ros_data):
        self.label_3.setText(ros_data.data)
        self.label_3.setStyleSheet('color: red')

    def video(self):
        if self.frame is None:
            return
        img = self.frame
        img_height, img_width, img_colors = img.shape
        scale_w = float(self.window_width) / float(img_width)
        scale_h = float(self.window_height) / float(img_height)
        scale = min([scale_w, scale_h])
        img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        height, width, bpc = img.shape
        bpl = bpc * width
        image = QtGui.QImage(img.data, width, height, bpl, QtGui.QImage.Format_RGB888)
        self.ImgWidget.setImage(image)

    def closeEvent(self, event):
        rospy.signal_shutdown("User has clicked the red x on the main window.")
        event.accept()

if __name__ == '__main__':
    rospy.init_node("eddy_GUI", anonymous=False)
    '''Initializes and cleanup ros node'''
    app = QtWidgets.QApplication(sys.argv)  # Create an instance of QtWidgets.QApplication
    window = Ui()  # Create an instance of our class
    window.setWindowTitle('Eddy GUI')
    window.show()
    #app.setStyle('Fusion')
    app.exec_()  # Start the application
    cv2.destroyAllWindows()
