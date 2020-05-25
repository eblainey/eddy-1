# Eddy

Eddy is a semi-autonomous surface vessel designed to facilitate unmanned sampling of biomatter in natural
bodies of waters. Eddy is outfitted with; a single beam echosounder for estimating biomass in the water column,
a downward facing low-light HD camera for imaging biomatter, a GPS sensor for localization and navigation, 
an IMU for attitude measurement and localization correction and a high powered LED to illuminate underwater 
features in extreme conditions.

## ROS Topics

/eddy/gps/fix
/eddy/gps/time_reference
/eddy/gps/vel

/eddy/imu/accel
/eddy/imu/gravity
/eddy/imu/imu
/eddy/imu/magnetic_field

/eddy/camera/image_raw
/eddy/camera/camera_info
/eddy/camera/compressed

/eddy/sonar/history
/eddy/sonar/profile
/eddy/sonar/distance
