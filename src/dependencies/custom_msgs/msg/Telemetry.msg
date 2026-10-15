# Header frame_id is the Pixhawk body frame: base_link_frd.
# IMU and magnetic-field vectors are FRD; attitude and heading are NED-referenced.
std_msgs/Header header
std_msgs/Bool armed

sensor_msgs/BatteryState battery
sensor_msgs/Imu imu
sensor_msgs/MagneticField magnetic_field
sensor_msgs/FluidPressure external_pressure
sensor_msgs/FluidPressure internal_pressure

# Roll, pitch, yaw in radians, referenced to NED.
geometry_msgs/Vector3 attitude_rpy_ned
# Degrees clockwise from north in the NED frame.
std_msgs/Float32 heading_ned_degrees
std_msgs/Float32MultiArray thruster_pwms
