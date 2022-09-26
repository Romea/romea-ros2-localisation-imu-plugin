# 1 Overview #

This package provides imu plugin for vehicle localisation. It takes data coming from an imu in order to compute unbiased angular speed and vehicle attitude (roll and pitch). The angular speed bias (angular speed average) is estimated when the vehicle does not move . A zero velocity dectection algorithm based on accelerometers and odometry data is used to assert if vehicle does not move or not.

# 2 Node #

### 2.1 Subscribed Topics ###

- vehicle_controller/odom (nav_msgs::Odometry)

  This topic is pusblished by standard vehicle controllers (diff_drive_controller,ackermann_steering_controller, four_wheel_steering_controller) and provides a lot informations like linear angular and speeds and dead reckoning.

- imu/data (sensor_msgs::Imu)

  This topic is published by imu sensor and provides linear accelerations, angular speeds and attitude angles

### 2.2 Published Topics ###

- twist (romea_localisation_msgs::ObservationTwist2DStamped)

### 2.3 Parameters ###

- ~imu/acceleration_noise_density (double)

- ~imu/acceleration_bias_stability_std (double)

- ~imu/acceleration_range (double)

- ~imu/angular_speed_noise_density (double)

- ~imu/angular_speed_bias_stability_std (double)

- ~imu/angular_speed_range(double)

- ~imu/magnetic_noise_density(double)

- ~imu/magnetic_bias_stability_std(double)

- ~imu/magnetic_range(double)

- ~imu/heading_std(double)

- ~restamping (bool, default: false)

    If this parameter is set to true stamp of angular speed and attitude messages is equal to computer current time else this stamp is equal imu message stamp.  This paremeter will be used when imu data are coming from a remote master.

- ~debug (bool, default: false)

    Enable or not debug logs

