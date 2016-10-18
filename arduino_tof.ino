/* This example shows how to use continuous mode to take
  range measurements with the VL6180X. It is based on
  vl53l0x_ContinuousRanging_Example.c from the VL53L0X API.

  The range readings are in units of mm. */

#include <Wire.h>
#include <Servo.h>
#include "vl53l0x-arduino/VL53L0X.h"

#define NUM_SENSORS 6
VL53L0X range_sensors[NUM_SENSORS]; // clockwise starting with looking forward

// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

#define LONG_RANGE

// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

//#define HIGH_SPEED
//#define HIGH_ACCURACY

#if defined(HIGH_SPEED) && defined(HIGH_ACCURACY)
#error "Can't define both HIGH_SPEED and HIGH_ACCURACY"
#endif

uint8_t sensor_pins[NUM_SENSORS];
uint16_t sensor_data[NUM_SENSORS];  // clockwise starting with looking forward

Servo handServo;
uint8_t handPwmPin = 5;

#define HAND_OPEN 0
#define HAND_CLOSED 140

#define STEREO_BUF_SIZE 384
uint8_t incoming[STEREO_BUF_SIZE];
uint8_t msg_buf[STEREO_BUF_SIZE];

struct uint8array {
  uint8_t len;
  uint8_t height;
  uint8_t *data;
  uint8_t data_new;
};

struct MsgProperties {
  uint16_t positionImageStart;
  uint8_t width;
  uint8_t height;
} ;

struct uint8array incoming_msg; // buffer used to contain image without line endings
uint16_t insert_loc, extract_loc, msg_start;   // place holders for buffer read and write

/**
   Increment circular buffer counter by i
*/
uint16_t stereoprot_add(uint16_t counter, uint16_t i, uint16_t buffer_size)
{
  return (counter + i) % buffer_size;
}


/**
   Decrement circular buffer counter by i
*/
uint16_t stereoprot_diff(uint16_t counter, uint16_t i, uint16_t buffer_size)
{
  return (counter - i + buffer_size) % buffer_size;
}

/**
   Checks if the sequence in the array is equal to 255-0-0-171,
   as this means that this is the end of an image
*/
uint8_t stereoprot_isEndOfMsg(uint8_t *stack, uint16_t i, uint16_t buffer_size)
{

  if (stack[i] == 255 && (stack[stereoprot_add(i, 1, buffer_size)] == 0) && (stack[stereoprot_add(i, 2, buffer_size)] == 0) && stack[stereoprot_add(i, 3, buffer_size)] == 171) {
    return 1;
  }
  return 0;
}

/**
   Checks if the sequence in the array is equal to 255-0-0-171,
   as this means a new image is starting from here
*/
uint8_t stereoprot_isStartOfMsg(uint8_t *stack, uint16_t i, uint16_t buffer_size)
{
  //printf("Checking start: %d %d %d %d \n",stack[i],stack[stereoprot_add(i, 1,buffer_size)],stack[stereoprot_add(i, 2,buffer_size)],stack[stereoprot_add(i, 3,buffer_size)]);
  if (stack[i] == 255 && (stack[stereoprot_add(i, 1, buffer_size)] == 0) && (stack[stereoprot_add(i, 2, buffer_size)] == 0) && stack[stereoprot_add(i, 3, buffer_size)] == 175) {
    return 1;
  }
  return 0;
}

/**
   Get all available data from stereo com link and decode any complete messages.
   Returns as soon as a complete message is found. Messages placed in msg_buf
*/
uint8_t handleStereoPackage(uint8_t newByte, uint16_t buffer_size, uint16_t *insert_loc, uint16_t *extract_loc, uint16_t *msg_start, uint8_t *msg_buf, uint8_t *ser_read_buf, uint8_t *stereocam_datadata_new, uint8_t *stereocam_datalen, uint8_t *stereocam_dataheight)
{
  MsgProperties msgProperties;
  // read all data from the stereo com link, check that don't overtake extract
  if ( stereoprot_add(*insert_loc, 1, buffer_size) != *extract_loc) {
    ser_read_buf[*insert_loc] = newByte;
    *insert_loc = stereoprot_add(*insert_loc, 1, buffer_size);
  }

  // search for complete message in buffer, if found increments read location and returns immediately

  //while (stereoprot_diff(*insert_loc, stereoprot_add(*extract_loc,3,buffer_size),buffer_size) > 0) {
  while (stereoprot_diff(*insert_loc, *extract_loc, buffer_size) > 3) {
    if (stereoprot_isStartOfMsg(ser_read_buf, *extract_loc, buffer_size)) {
      *msg_start = *extract_loc;
    } else if (stereoprot_isEndOfMsg(ser_read_buf, *extract_loc, buffer_size)) { // process msg
      // Find the properties of the image by iterating over the complete image
      stereoprot_get_msg_properties(ser_read_buf, &msgProperties, *msg_start, buffer_size);
      // Copy array to circular buffer and remove all bytes that are indications of start and stop lines
      uint16_t i = stereoprot_add(*msg_start, 8, buffer_size), j = 0, k = 0, index = 0;
      for (k = 0; k < msgProperties.height; k++) {
        for (j = 0; j < msgProperties.width; j++) {
          msg_buf[index++] = ser_read_buf[i];
          i = stereoprot_add(i, 1, buffer_size);
        }
        i = stereoprot_add(i, 8, buffer_size);   // step over EOL and SOL
      } // continue search for new line
      *stereocam_datalen = msgProperties.width * msgProperties.height;
      *stereocam_dataheight = msgProperties.height;
      *stereocam_datadata_new = 1;
      *extract_loc = stereoprot_add(*extract_loc, 4, buffer_size);     // step over EOM string

      return 1;
    }
    *extract_loc = stereoprot_add(*extract_loc, 1, buffer_size);
  }
  return 0;
}

/**
   Retrieve size of image from message
*/
void stereoprot_get_msg_properties(uint8_t *raw, MsgProperties *properties, uint16_t start, uint16_t buffer_size)
{
  *properties = (MsgProperties) {
    start, 0, 0
  };
  uint16_t i = start, startOfLine = start;
  while (1) {
    // Check the first 3 bytes for the pattern 255-0-0, then check what special byte is encoded next
    if ((raw[i] == 255) && (raw[stereoprot_add(i, 1, buffer_size)] == 0) && (raw[stereoprot_add(i, 2, buffer_size)] == 0)) {
      if (raw[stereoprot_add(i, 3, buffer_size)] == 171) { // End of image
        break;
      }
      if (raw[stereoprot_add(i, 3, buffer_size)] == 128) { // Start of line
        startOfLine = i;
      }
      if (raw[stereoprot_add(i, 3, buffer_size)] == 218) { // End of line
        properties->height++;
        properties->width = stereoprot_diff(i, startOfLine + 4, buffer_size); // removed 4 for the indication bits at the end of line
      }
    }
    i = stereoprot_add(i, 1, buffer_size);
  }
}

void SendArray(uint8_t *b, uint8_t array_width, uint8_t array_height)
{
  uint8_t code[4];
  code[0] = 0xff;
  code[1] = 0x00;
  code[2] = 0x00;
  code[3] = 0xAF; // 175
  Serial.write(code, 4);

  uint16_t horizontalLine = 0;
  for (horizontalLine = 0; horizontalLine < array_height; horizontalLine++) {
    code[3] = 0x80;//128
    Serial.write(code, 4);

    Serial.write(b + array_width * horizontalLine, array_width);

    code[3] = 0xDA;//218
    Serial.write(code, 4);
  }

  code[3] = 0xAB;
  Serial.write(code, 4);
}

// coutners
uint8_t i = 0;
void setup()
{
  // setup communication protocol
  Serial.begin(115200);
  insert_loc = 0; extract_loc = 0; msg_start = 0;
  incoming_msg.len = 0; incoming_msg.data = msg_buf; incoming_msg.data_new = 0; incoming_msg.height = 0;

  // setup hand servo control
  pinMode(handPwmPin, OUTPUT);
  handServo.attach(handPwmPin);
  handServo.write(HAND_OPEN);

  // setup i2c VL53LOX range finder sensor sampling
  Wire.begin();

  sensor_pins[0] = 17;
  sensor_pins[1] = 9;
  sensor_pins[2] = 10;
  sensor_pins[3] = 12;
  sensor_pins[4] = 13;
  sensor_pins[5] = 14;

  // disable all sensors
  for (i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensor_pins[i], OUTPUT);
  }

  // set address front sensor
  for (i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensor_pins[i], INPUT);
    delay(5);
    
    range_sensors[i].init();
    range_sensors[i].setAddress(10);
    range_sensors[i].setTimeout(500);

#if defined LONG_RANGE
    // lower the return signal rate limit (default is 0.25 MCPS)
    range_sensors[i].setSignalRateLimit(0.1);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    range_sensors[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    range_sensors[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif
    
#if defined HIGH_SPEED
    // reduce timing budget to 20 ms (default is about 33 ms)
    range_sensors[i].setMeasurementTimingBudget(20000);
    #elif defined HIGH_ACCURACY
    // increase timing budget to 200 ms
    range_sensors[i].setMeasurementTimingBudget(200000);
#endif
    delay(5);
  }
  
  // initialise data
  memset(sensor_data, 0, sizeof(sensor_data));

  // start sensor sampling
  for (i = 0; i < NUM_SENSORS; i++) {
    range_sensors[i].startContinuous(); delay(1);
  }
}

void loop()
{
  // read incoming data // Read from other device with the stereo communication protocol.
  while (Serial.available() > 0 && stereoprot_add(insert_loc, 1, STEREO_BUF_SIZE) != extract_loc) {
    if (handleStereoPackage(Serial.read(), STEREO_BUF_SIZE, &insert_loc, &extract_loc, &msg_start, incoming_msg.data, incoming,
                            &incoming_msg.data_new, &incoming_msg.len, &incoming_msg.height)) {
      // hand open/close command length 1
      if (incoming_msg.len == 1) {
        if (incoming_msg.data[0]) {
          handServo.write(HAND_CLOSED);
        } else if (!incoming_msg.data[0]) {
        }
        incoming_msg.len = 0;
      }
    }
  }

  // sensor read are blocking calls so will auto regulate loop timing at ~10Hz
  for (i = 0; i < NUM_SENSORS; i++) {
    sensor_data[i] = range_sensors[i].readRangeContinuousMillimeters();
  }

  SendArray((uint8_t*)sensor_data, NUM_SENSORS * sizeof(uint16_t), 1);
}

