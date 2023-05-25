/*
==========
Teensy code for Spakrfun Qwiic VL53L5CX ToF Imager
This code implements the following functionality:
  > Reads sensor measurements over I2C, synchronizes time with the ROS2 host PC, and populates the PointCloud2 message (publisher)
  > micro-ROS communication with Host PC over Serial (Raspberry Pi 4 with ROS 2 Humble)
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

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){errorLoop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

#define ROS_DOMAIN_ID 0
#define OFFSET        4
#define NR_FIELDS     3
#define RESOLUTION    8
#define FOV           PI/2
#define LED_PIN       13
#define FREQUENCY     15

//micro-ROS publisher entities
rclc_support_t     support;
rcl_node_t         node;
rcl_publisher_t    publisher;
rcl_allocator_t    allocator;
rcl_init_options_t init_options;
sensor_msgs__msg__PointCloud2 msg;

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

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state = WAITING_AGENT;

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  
  Wire.begin(); 
  Wire.setClock(1000000);

  if (myImager.begin() == false){ while (1); }
  myImager.setResolution(RESOLUTION * RESOLUTION); //enable all 64 pads
  myImager.setRangingFrequency(FREQUENCY); //using 8x8, min frequency is 1Hz and max is 15Hz
  myImager.startRanging();

  set_microros_transports();
}

void loop() 
{
  switch (state)
  {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT){ destroyEntities(); }
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED){ publishPointCloud(); }
      break;
    case AGENT_DISCONNECTED:
      destroyEntities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  state == AGENT_CONNECTED ? digitalWrite(LED_PIN, LOW) : digitalWrite(LED_PIN, HIGH);
}

bool createEntities()
{
  allocator    = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, (size_t)ROS_DOMAIN_ID));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "tof_imager_node", "", &support));

  RCSOFTCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, PointCloud2),
    "pointcloud")
  );

  //initialize message memory
  micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, PointCloud2),
    &msg,
    (micro_ros_utilities_memory_conf_t) {});

  // populate the static fields of the message
  msg.height       = RESOLUTION;
  msg.width        = RESOLUTION;
  msg.is_bigendian = false;
  msg.point_step   = NR_FIELDS * OFFSET;
  msg.row_step     = msg.point_step * msg.width;
  msg.data.size    = msg.row_step * msg.height;
  msg.is_dense     = false;
  msg.fields.size  = 3;

  for(int i=0; i<(int)msg.fields.size; i++)
  {
    msg.fields.data[i].offset   = i*OFFSET;
    msg.fields.data[i].datatype = 7;
    msg.fields.data[i].count    = 1;
  }

  msg.header.frame_id = micro_ros_string_utilities_set(msg.header.frame_id, "tof_frame");
  msg.fields.data[0].name = micro_ros_string_utilities_set(msg.fields.data[0].name, "x");
  msg.fields.data[1].name = micro_ros_string_utilities_set(msg.fields.data[1].name, "y");
  msg.fields.data[2].name = micro_ros_string_utilities_set(msg.fields.data[2].name, "z");

  // synchronize time with agent
  RCCHECK(rmw_uros_sync_session(10));
  calculateOffset();

  return true;
}

bool destroyEntities()
{  
  rcl_ret_t rc = rcl_publisher_fini(&publisher, &node);
  rc += rcl_node_fini(&node);
  rc += rclc_support_fini(&support);
  rc += rcl_init_options_fini(&init_options);

  return (rc != RCL_RET_OK) ? false : true;
}

void publishPointCloud()
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
        }
      }
    }
  }
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  delay(1000/FREQUENCY);
}

void errorLoop()
{
  // flash LED twice till the microcontroller is reset
  while(1)
  {
    flashLED(2);
  }
}

void flashLED(int n_times)
{
  // flash LED a given number of times
  for(int i=0; i<n_times; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(150);
    digitalWrite(LED_PIN, LOW);
    delay(150);
  }
  delay(1000);
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