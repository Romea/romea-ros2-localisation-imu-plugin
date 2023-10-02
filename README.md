# 1 Overview #

This package provides imu plugin for robot localisation. It takes data coming from an imu in order to compute unbiased angular speed and vehicle attitude (roll and pitch). The angular speed bias (angular speed average) is estimated when the robot does not move . A zero velocity dectection algorithm based on accelerometers and odometry data is used to assert if robot does not move or not.

# 2 Node #

### 2.1 Subscribed Topics ###

- vehicle_controller/odom (nav_msgs::msg::msg::Odometry)

  This topic is pusblished by standard vehicle controllers (diff_drive_controller,ackermann_steering_controller, four_wheel_steering_controller) and provides a lot informations like linear angular and speeds and dead reckoning.

- imu/data (sensor_msgs::msg::Imu)

  This topic is published by imu sensor and provides linear accelerations, angular speeds and attitude angles

### 2.2 Published Topics ###

- angular_speed (romea_localisation_msgs::msg::ObservationAngularSpeedStamped)

  Unbiased angular speed and its variance 

- attitude (romea_localisation_msgs::msg::ObservationAttitudeStamped)

  Roll and pitch angles and their covariance

### 2.3 Parameters ###

- imu.acceleration_noise_density (double)
 
  Noise density of acceleration data in m/s^2/√Hz
   
- imu.acceleration_bias_stability_std (double)

  Standard deviation of acceleration data  biases in m/s^2

- imu.acceleration_range (double)

  Range of acceleration data in m/s^2

- imu.angular_speed_noise_density (double)

  Noise density of angular speed data in rad/s/√Hz

- imu.angular_speed_bias_stability_std (double)

  Standard deviation of angular_speed data biases in rad/s

- imu.angular_speed_range(double)

  Range of angular speed data in rad/s
  
- imu.magnetic_noise_density(double)

  Noise density of magnetic data in T/√Hz

- imu.magnetic_bias_stability_std(double)

  Standard deviation of magnetic data biases in T

- imu.magnetic_range(double)

  Range of magnetic data in T

- imu.heading_std(double)

  Standard deviation of heading angle in degrees

- imu.xyz

  Imu position in localisation body reference frame (usually base_foot_print_link) in meters

- imu.rpy

  Imu orientation in localisation body reference frame (usually base_foot_print_link) in degrees

- restamping (bool, default: false)

  If this parameter is set to true stamp of angular speed and attitude messages is equal to computer current time else this stamp is equal imu message stamp.  This paremeter will be used when imu data are coming from a remote master.

- debug (bool, default: false)

  Enable or not debug logs

