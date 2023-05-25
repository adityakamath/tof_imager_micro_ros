/*
==========
Arduino Portenta code for Spakrfun Qwiic VL53L5CX ToF Imager
This code implements the following functionality:
  > Reads sensor measurements over I2C, synchronizes time with the ROS2 host PC, and populates the PointCloud2 message (publisher)
  > micro-ROS communication with Host PC using several transports (Raspberry Pi 4 with ROS2 Galactic)
Author: Aditya Kamath
adityakamath.github.io
github.com/adityakamath
==========
*/

#include <micro_ros_arduino.h>
#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/point_cloud2.h>
#include <sensor_msgs/msg/point_field.h>

//other includes
#include <math.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <stdio.h>
#include <TimeLib.h>
#include <Wire.h>

#if !defined(TARGET_PORTENTA_H7_M7)
#error This example is only avaible for Arduino Portenta H7 (M7 Core)
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define ROS_DOMAIN_ID 2
#define PROCESSING    true //flag for the Processing App to visualize sensor readings
#define HWSERIAL      Serial //change to Serial1 if using serial or wifi OR set the Processing flag to false
#define OFFSET        4
#define NR_FIELDS     3
#define RESOLUTION    8
#define FOV           PI/2
#define LED_PIN       LEDR

//micro-ROS publisher entities
rclc_support_t     support;
rcl_node_t         node;
rcl_publisher_t    publisher;
rcl_allocator_t    allocator;
sensor_msgs__msg__PointCloud2 msg;
bool uros_initialized = false;

//VL53L5CX ToF sensor data
SparkFun_VL53L5CX    myImager;
VL53L5CX_ResultsData measurementData; //result data class structure, 1356 byes of RAM

typedef union
{
  float   number;
  uint8_t bytes[4];
} FLOAT2UINT8_T;
FLOAT2UINT8_T x, y, z;

unsigned long long time_offset = 0;

void error_loop()
{
  if(uros_initialized)
  {
    teardown_uros();
  }
  while(1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void teardown_uros()
{
  if(uros_initialized)
  {
    rcl_ret_t rc = rcl_publisher_fini(&publisher, &node);
    rc += rcl_node_fini(&node);
    rc += rclc_support_fini(&support);
    if(rc == RCL_RET_OK){ uros_initialized = false; }
  }
}

void setup()
{
  byte arduino_mac[] = {0x01, 0xAB, 0x23, 0xCD, 0x45, 0xEF}; //user assigned MAC address
  IPAddress arduino_ip(192, 168, 1, 109);                    //user assigned IP address
  IPAddress agent_ip(192, 168, 1, 100);                      //static IP defined on the ROS2 agent side
  set_microros_native_ethernet_udp_transports(arduino_mac, arduino_ip, agent_ip, 8888);

  bootM4(); //boot M4 core for dual core processing
  digitalWrite(LED_PIN, LOW);
  delay(2000);

  //initialize I2C and serial communication
  if(PROCESSING){ HWSERIAL.begin(115200); }
  Wire.begin(); //this resets I2C bus to 100kHz
  Wire.setClock(1000000); //sensor has max I2C freq of 1MHz

  //VL53L5CX: Configure and start the sensor
  myImager.setWireMaxPacketSize(128); //increase default from 32 to 128 bytes
  if (myImager.begin() == false){ while (1); }
  myImager.setResolution(RESOLUTION * RESOLUTION); //enable all 64 pads
  myImager.setRangingFrequency(15); //using 8x8, min frequency is 1Hz and max is 15Hz
  myImager.startRanging();

  //micro-ROS: define and configure allocator, init_options, node and publisher
  allocator    = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, (size_t)ROS_DOMAIN_ID));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_vl53l5cx_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, PointCloud2),
    "micro_ros_vl53l5cx_pcl_publisher")
  );

  //initialize message memory
  if(!micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, PointCloud2),
    &msg,
    (micro_ros_utilities_memory_conf_t) {})
    )
  {
    error_loop();
  }

  uros_initialized = true;

  //populate the static fields of the message
  msg.header.frame_id = micro_ros_string_utilities_set(msg.header.frame_id, "tof_frame");
  msg.height          = RESOLUTION;
  msg.width           = RESOLUTION;

  msg.fields.size  = 3;
  msg.fields.data[0].name = micro_ros_string_utilities_set(msg.fields.data[0].name, "x");
  msg.fields.data[1].name = micro_ros_string_utilities_set(msg.fields.data[1].name, "y");
  msg.fields.data[2].name = micro_ros_string_utilities_set(msg.fields.data[2].name, "z");

  for(int i=0; i<msg.fields.size; i++)
  {
    msg.fields.data[i].offset   = i*OFFSET;
    msg.fields.data[i].datatype = 7;
    msg.fields.data[i].count    = 1;
  }

  msg.is_bigendian = false;
  msg.point_step   = NR_FIELDS * OFFSET;
  msg.row_step     = msg.point_step * msg.width;
  msg.data.size    = msg.row_step * msg.height;
  msg.is_dense     = false;

  //synchronize time
  RCCHECK(rmw_uros_sync_session(1000));
  calculateOffset();
}

void loop()
{
  struct timespec time_stamp = getTime();
  msg.header.stamp.sec = time_stamp.tv_sec;
  msg.header.stamp.nanosec = time_stamp.tv_nsec;

  //poll sensor for new data
  int data_count = 0;
  if (myImager.isDataReady() == true)
  {
    if (myImager.getRangingData(&measurementData))
    {
      //loop over width and height of the depth image
      for(int w = 0; w < RESOLUTION; w++)
      {
        for(int h = 0; h < RESOLUTION; h++)
        {
          int depth = measurementData.distance_mm[w + h*RESOLUTION]; //depth in mm
          int depth_mm = (depth < 0) ? 0 : depth;                    //set invalid measurements to 0

          x.number = (cos(w * (FOV/RESOLUTION) - FOV/2.0 - PI/2) * depth_mm)/1000.0; //x in m
          y.number = (sin(h * (FOV/RESOLUTION) - FOV/2.0) * depth_mm)/1000.0;        //y in m
          z.number = (depth_mm)/1000.0;                                              //z in m

          //decompose Float32 into uint8 bytes and populate msg.data[]
          for(int i=0; i<OFFSET; i++){
            msg.data.data[data_count + i]            = uint8_t(x.bytes[i]);
            msg.data.data[data_count + i + OFFSET]   = uint8_t(y.bytes[i]);
            msg.data.data[data_count + i + OFFSET*2] = uint8_t(z.bytes[i]);
          }
          data_count += OFFSET*NR_FIELDS;

          //print row data as CSV to serial
          if(PROCESSING)
          {
            HWSERIAL.print(depth_mm);
            HWSERIAL.print(",");
          }
        }
      }
      if(PROCESSING){ HWSERIAL.println(""); } //print new line after each row
    }
  }
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  delay(20);
}

void calculateOffset()
{
  unsigned long now = millis();
  unsigned long long ros_time_ms = rmw_uros_epoch_millis();
  time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
  struct timespec tp = {0};
  // add time difference between uC time and ROS time to
  // synchronize time with ROS
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;

  return tp;
}
