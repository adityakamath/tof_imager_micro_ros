# teensy_pcl_publisher

  * Sketch for a Teensy 4.1 (MicroMod) and the MicroMod Carrier Board with [this Sparkfun Qwiik ToF Imager](https://www.sparkfun.com/products/18642) which is based on the VL53L5CX 8x8 ToF sensor, with a maximum range of 4m.
  * Uses the Sparkfun VL53L5CX Library to read raw sensor data over I2C, and calculates the position of every detected point (x, y, z) w.r.t the sensor.
  * Uses the micro-ROS utilities to sync the time with the micro-ROS agent, and populates a PointCloud2 message, along with the calculated positions.
  * Publishes this PointCloud2 message over Serial.

This sketch was uploaded from Windows 10 (using the Arduino IDE with micro_ros_arduino v2.0.5-galactic) and tested using ROS2 Galactic.
