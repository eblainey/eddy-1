# Eddy

Eddy is a semi-autonomous surface vessel designed to facilitate unmanned sampling of biomatter in natural
bodies of waters. Eddy is outfitted with; a single beam echosounder for estimating biomass in the water column,
a downward facing low-light HD camera for imaging biomatter, a GPS sensor for localization and navigation, 
an IMU for attitude measurement and localization correction and a high powered LED to illuminate underwater 
features in extreme conditions.

## ROS Topics

* /eddy/gps/fix - sensor_msgs/NatSatFix
* /eddy/gps/time_reference - sensor_msgs/TimeReference
* /eddy/gps/vel - geometry_msgs/TwistStamped
* /eddy/imu/accel - geometry_msgs/Vector3Stamped
* /eddy/imu/gravity - geometry_msgs/Vector3Stamped
* /eddy/imu/imu - sensor_msgs/Imu
* /eddy/imu/magnetic_field - sensor_msgs/MagneticField
* /eddy/camera/image_raw - sensor_msgs/Image
* /eddy/camera/camera_info - sensor_msgs/CameraInfo
* /eddy/camera/compressed - sensor_msgs/CompressedImage
* /eddy/sonar/history - sensor_msgs/Image
* /eddy/sonar/profile - std_msgs/UInt8MultiArray
* /eddy/sonar/distance - sensor_msgs/Range
