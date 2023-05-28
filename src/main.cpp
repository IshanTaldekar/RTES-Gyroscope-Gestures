#include "mbed.h"
#include "AccurateWaiter.h"
#include "LCD_DISCO_F429ZI.h"
#include "stm32f429i_discovery_ts.h"

#include <iostream>
#include <string>

/* Globals below this point @author James Huang: */
DigitalIn button(USER_BUTTON); // Button input

/* Status LED outputs: */
DigitalOut led1(LED1); 
DigitalOut led2(LED2);

/* Globals below this point @author Jack Shkifati */

SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel); // mosi, miso, sclk, cs

InterruptIn int2(PA_2, PullDown);

#define CTRL_REG1 0x20  // register fields(bits): data_rate(2),Bandwidth(2),Power_down(1),Zen(1),Yen(1),Xen(1)

/*
  Data sheet table 22
  ODR : The rate at which sensor provides data
  Cutoff: Filter out high frequency
  configuration: 200Hz ODR,50Hz cutoff, Power on, Z on, Y on, X on
*/
#define CTRL_REG1_CONFIG 0b11'10'1'1'1'1

#define CTRL_REG4 0x23  // register fields(bits): reserved(1), endian-ness(1),Full scale sel(2), reserved(1),self-test(2), SPI mode(1)

/*
  Controls how fast you want to measure degrees per second : (00: 245 dps, 01: 500 dps, 10: 2000dps, 11: 2000 dps)
  Configuration: reserved,little endian,500 dps,reserved,disabled,4-wire mode
*/
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0

/*
  READY Inturrput configuration
  register fields(bits): I1_Int1 (1), I1_Boot(1), H_Lactive(1), PP_OD(1), I2_DRDY(1), I2_WTM(1), I2_ORun(1), I2_Empty(1)
*/
#define CTRL_REG3 0x22

#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000  // configuration: Int1 disabled, Boot status disabled, active high interrupts, push-pull, enable Int2 data ready, disable fifo interrupts

#define OUT_X_L 0x28  // By setting the MSB to 1 we can auto increment through the output data from 0x28 to 0x2D (X: Z)

#define SPI_FLAG 1  // SPI flag when complete

#define DATA_READY_FLAG 2  // Data ready flag when next data is values of data are ready to be read.

uint8_t write_buf[32];
uint8_t read_buf[32];

EventFlags flags;

/* Global variables below this point: @author Ishan Taldekar */

int16_t user_profile_ui_squares_y_positions[5] = {245, 198, 151, 103, 55};

enum DeviceMode {

  RECORD = 1,
  UNLOCK

};

enum UIBackgroundColor { 

    DARK_CYAN = 1,
    GREEN, 
    RED

};

UIBackgroundColor ui_background_color = UIBackgroundColor::DARK_CYAN;

std::string ui_ready_statement = "ready";
std::string ui_recording_statement = "record mode";
std::string ui_recording_successful_statement = "successful record";
std::string ui_unlocking_statement = "unlock mode";
std::string ui_unlocking_successful_statement = "successful unlock";
std::string ui_unlocking_failed_statement = "failed unlock";

std::string ui_state_descriptor = ui_ready_statement;

bool user_profile_locked_flag = false;  // when true, user profile cannot be changed

const int32_t SENSING_DURATION = 100;  // number of seconds that gyroscope will sense gesture: 100 -> 5 seconds, 200 -> 10 seconds, ...
const int32_t STABILITY_DURATION = 5;  // timeframe after which stability of measurement in any axis is gauged + compared about these values
const float STABILITY_THRESHOLD = 5;  // upper bound on the change in degree a value under which is considered to be holding stable
const int32_t COMPARE_THRESHOLD = 30;  // the difference in degrees around the actual value of measurement that still register as a match
const float DELTA_THRESHOLD = 10;  // amount of change in angle required for an major change along an axis is registered
const float SENSOR_DRIFT_FILTER_THRESHOLD = 0.15;  // upper-bound measurement value which is attributed to sensor drift

struct Data {

  float delta_x = 0.0f;  // current change in x axis since last time the direction flipped or angle held stable
  float delta_y = 0.0f;  // current change in y axis since last time the direction flipped or angle held stable
  float delta_z = 0.0f;  // current change in z axis since last time the direction flipped or angle held stable

  bool positive_delta_x[SENSING_DURATION];  // direction of change in x axis currently in the positive direction?
  bool positive_delta_y[SENSING_DURATION];  // direction of change in y axis currently in the positive direction?
  bool positive_delta_z[SENSING_DURATION];  // direction of change in z axis currently in the positive direction?
  
  float angles_x[SENSING_DURATION];
  float angles_y[SENSING_DURATION];
  float angles_z[SENSING_DURATION];

  int32_t angles_index = 0;
  int32_t positive_delta_index = -1;

} data;

struct NewMeasurement {

  float x = 0.0f;  // angle measurement in x direction
  float y = 0.0f;  // angle measurement in y direction
  float z = 0.0f;  // angle measurement in z direction

} new_measurement;

enum MovementAxis {  // used for labelling the axes

  X = 1,
  Y,
  Z

};

struct Gesture {  // container for recording gesture as a sequence of movements defined by axis of major change and amount of change in degrees

  MovementAxis axis[SENSING_DURATION];  // axis of major change
  float angle_change[SENSING_DURATION];  // amount of change

  int32_t next_index = 0;  // index where next measurement goes
  int32_t compare_index = 0;  // measurement to be comapred next

};

int8_t current_user_profile = 4;  // the current user profile selected using the UI: 4 --> user profile 1, 0 --> user profile 5

Gesture recorded_gestures[5];  // gestures stored with the index corresponding to the user profile

LCD_DISCO_F429ZI lcd;
TS_StateTypeDef ts_state;

/**
 * spi callback function:
 * The spi transfer function requires that the callback provided to it takes an int parameter
 * 
 * @author Jack Shkifati
*/
void spi_cb(int event) {

  flags.set(SPI_FLAG);

};

/**
 * Data callback function
 * 
 * @author Jack Shkifati
*/
void data_cb() {
  
  flags.set(DATA_READY_FLAG);

};

/**
 * Determine whether or not a gesture might have been completed by checking if the direction of rotation along an axis changed or
 * the angle has held steady for 200 ms (configurable using STABILITY_DURATION).
 * 
 * @param angle_delta the current angle change recorded 
 * @param positive_delta the direction of angle change 
 * @param angles the current rotation angle value
 * 
 * @return true if this axis needs to be compared with other axes values
 * 
 * @author Ishan Taldekar
 */
bool compareAxisChange(float angle_delta, bool positive_delta[SENSING_DURATION], float angles[SENSING_DURATION]) {

  return abs(angle_delta) > DELTA_THRESHOLD && 
         ((data.positive_delta_index > 0 && positive_delta[data.positive_delta_index] != positive_delta[data.positive_delta_index - 1]) || 
         (data.angles_index >= STABILITY_DURATION && abs(angles[data.angles_index - STABILITY_DURATION] - angles[data.angles_index]) <= STABILITY_THRESHOLD) ||
         (data.positive_delta_index == SENSING_DURATION - 1));  

}

/**
 * Either record a new gesture segment for the current user profile, or compare the new gesture segment to the recorded segment in the 
 * movement sequence for the old profile depending on the current device mode. 
 * 
 * @param current_mode the current mode (RECORD or UNLOCK)
 * @param movement_axis the axis of rotation (X, Y, or Z)
 * @param delta the magnitude of angle change recorded
 * @param gesture_match updated if in UNLOCK mode and the gesture segment does not match the expected
 * 
 * @author Ishan Taldekar
 */
void registerGestureSegment(DeviceMode current_mode, MovementAxis movement_axis, float delta, bool &gesture_match) {

  if (current_mode == DeviceMode::RECORD) {  // record the current gesture segment (axis and magnitude of change of rotation)

    recorded_gestures[current_user_profile].axis[recorded_gestures[current_user_profile].next_index] = movement_axis;
    recorded_gestures[current_user_profile].angle_change[recorded_gestures[current_user_profile].next_index] = delta;
    ++recorded_gestures[current_user_profile].next_index;

  } else {  // compare the current gesture segment (axis and magnitude of change of rotation)

    if (!(recorded_gestures[current_user_profile].axis[recorded_gestures[current_user_profile].compare_index] == movement_axis
        && abs(recorded_gestures[current_user_profile].angle_change[recorded_gestures[current_user_profile].compare_index] - delta) <= COMPARE_THRESHOLD)) {

      gesture_match = false;

    }

    ++recorded_gestures[current_user_profile].compare_index;

  }

  data.delta_x = 0;
  data.delta_y = 0;
  data.delta_z = 0;

}

/**
 * Check if the new measurement triggers a gesture segment registeration, and compare or record the new gesture segment.
 * 
 * @param current_mode the current mode (RECORD or UNLOCK)
 * @param gesture_match updated if in unlock mode and the gesture does not match the expected values
 * 
 * @author Ishan Taldekar
 */
void processNewMeasurements(DeviceMode current_mode, bool &gesture_match) {

  if (current_mode == DeviceMode::UNLOCK && recorded_gestures[current_user_profile].next_index == 0) {  // return false if in unlock mode and a gesture has not previously been recorded

    gesture_match = false;
    return;

  }

  if (!gesture_match) return;

  if (compareAxisChange(data.delta_x, data.positive_delta_x, data.angles_x)) {

    if (abs(data.delta_x) >= abs(data.delta_y) && abs(data.delta_x) >= abs(data.delta_z)) {  // is the change in the current axis larger than the change in the other two axes?
      
      registerGestureSegment(current_mode, MovementAxis::X, data.delta_x, gesture_match);

    }

  }

  if (compareAxisChange(data.delta_y, data.positive_delta_y, data.angles_y)) {

    if (abs(data.delta_y) >= abs(data.delta_x) && abs(data.delta_y) >= abs(data.delta_z)) {  // is the change in the current axis larger than the change in the other two axes?

      registerGestureSegment(current_mode, MovementAxis::Y, data.delta_y, gesture_match);
      
    }

  }

  if (compareAxisChange(data.delta_z, data.positive_delta_z, data.angles_z)) {

    if (abs(data.delta_z) >= abs(data.delta_x) && abs(data.delta_z) >= abs(data.delta_y)) {  // is the change in the current axis larger than the change in the other two axes?

      registerGestureSegment(current_mode, MovementAxis::Z, data.delta_z, gesture_match);
      
    }

  }

}

/**
 * Logs whether movement along each axis is in the positive or negative direction.
 * 
 * @author Ishan Taldekar
 */
void logMovementDirections() {

  ++data.positive_delta_index;

  data.positive_delta_x[data.positive_delta_index] = new_measurement.x > 0;  // is the movement along x axis in the positive direction?
  data.positive_delta_y[data.positive_delta_index] = new_measurement.y > 0;  // is the movement along y axis in the positive direction?
  data.positive_delta_z[data.positive_delta_index] = new_measurement.z > 0;  // is the movement along z axis in the positive direction?

}

/**
 * Logs movement in a direction, if its above a threshold. And updates the current total angle measurement.
 * 
 * @param current_delta the total change in angle for measured for the current axis in the same direction
 * @param angles_list the list of angle of rotation with each new gyroscope measurement
 * @param current_measurement the current measurement for change in angle of rotation for the current axis
 * 
 * @author Ishan Taldekar
 */
void logMovementAngles(float &current_delta, float angles_list[SENSING_DURATION], float &current_measurement) {

  if (abs(current_measurement) > SENSOR_DRIFT_FILTER_THRESHOLD) {  // filter out sensor drift, if measurement less than sensor drift threshold
    
    current_delta += current_measurement; 

    // Update the current angle that the device has turned in along this axis from the base
    angles_list[data.angles_index] = data.angles_index > 0 ? angles_list[data.angles_index - 1] + current_measurement: current_measurement;
  
  } else {  // No update if the there is no measured rotation

    angles_list[data.angles_index] = data.angles_index > 0 ? angles_list[data.angles_index - 1]: 0;

  }

}

/**
 * Logs movement in three axes: x, y, z.
 * 
 * @author Ishan Taldekar
 */
void logMovementAngles() {

  ++data.angles_index;

  logMovementAngles(data.delta_x, data.angles_x, new_measurement.x);
  logMovementAngles(data.delta_y, data.angles_y, new_measurement.y);
  logMovementAngles(data.delta_z, data.angles_z, new_measurement.z);

}

/**
 * Records new gyroscope measurements 
 * 
 * @param current_mode the current mode (RECORD or UNLOCK)
 * 
 * @author Ishan Taldekar, Jack Skhifati
 */
bool senseAndProcessGesture(DeviceMode current_mode) {

  while (button.read()) {  // Wait for the button to be released

    thread_sleep_for(1);

  }

  ui_state_descriptor = current_mode == DeviceMode::RECORD ? ui_recording_statement : ui_unlocking_statement;  // display appropriate status (record mode or unlock mode) at the bottom of screen

  /* Turn on both LEDs to indicate that the device is reading from the sensor: */
  led1 = 1;
  led2 = 1;

  int16_t raw_gx, raw_gy, raw_gz;  // raw gyroscope readings along the three axes

  /* reset values to base in preparation for a new run: */

  bool gesture_match = true;

  data.delta_x = 0;
  data.delta_y = 0;
  data.delta_z = 0;

  data.positive_delta_index = -1;
  data.angles_index = -1;
  
  if (current_mode == DeviceMode::UNLOCK) recorded_gestures[current_user_profile].compare_index = 0;

  if (current_mode == DeviceMode::RECORD) recorded_gestures[current_user_profile].next_index = 0;

  for (int16_t i = 0; i < SENSING_DURATION; ++i) {  // New run, sense gesture for predefined duration

    // wait until new sample is ready
    flags.wait_all(DATA_READY_FLAG);

    //               data  | read | increment
    write_buf[0] = OUT_X_L | 0x80 | 0x40;

    // start sequential sample reading  7= (read 6 registers and write to 1)
    spi.transfer(write_buf, 7, read_buf, 7, spi_cb, SPI_EVENT_COMPLETE);
    flags.wait_all(SPI_FLAG);

    // read_buf after transfer: garbage byte, gx_low,gx_high,gy_low,gy_high,gz_low,gz_high
    // Put the high and low bytes in the correct order lowB,Highb -> HighB,LowB
    raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
    raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
    raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

    new_measurement.x = ((float)raw_gx) * 0.001;  // change in angle measured in x direction
    new_measurement.y = ((float)raw_gy) * 0.001;  // change in angle measured in y direction
    new_measurement.z = ((float)raw_gz) * 0.001;  // change in angle measured in z direction

    logMovementDirections();
    logMovementAngles();
    
    processNewMeasurements(current_mode, gesture_match);
    
    // thread_sleep_for(50);

    std::cout << "CURRENT ANGLE: " << data.angles_x[data.angles_index] << ", " << data.angles_y[data.angles_index] << ", " << data.angles_z[data.angles_index] << std::endl;
    
  }

  if (gesture_match[recorded_gestures].next_index != recorded_gestures[current_user_profile].compare_index) gesture_match = false;

  return gesture_match;

}

/**
 * Communicates whether the record or unlock process carried out by the user was successful using the LCD background colors
 * and the red and green LEDs on the device.
 * 
 * @param current_mode the current mode (RECORD or UNLOCK)
 * @param result was the event a success or failed
 * 
 * @author Ishan Taldekar
 */ 
void displayResult(DeviceMode current_mode, bool result) {

  led1 = 0;
  led2 = 0;

  thread_sleep_for(500);

  if (result) {  // if successful blink green 5 times both using the green led and the LCD screen

    ui_state_descriptor = current_mode == DeviceMode::RECORD ? ui_recording_successful_statement : ui_unlocking_successful_statement;  // print out appropriate status message (record successful or unlock successful) at the bottom of the screen

    for (int8_t i = 0; i < 5; ++i) { 

      led1 = 1;
      ui_background_color = UIBackgroundColor::GREEN;

      thread_sleep_for(500);

      led1 = 0;
      ui_background_color = UIBackgroundColor::DARK_CYAN;

      thread_sleep_for(500); 

    }

  } else {  // if failed blink red 5 times both using the red led and the LCD screen.

    ui_state_descriptor = ui_unlocking_failed_statement;  // print out unlock failed, recording cannot fail

    for (int8_t i = 0; i < 5; ++i) {

      led2 = 1;
      ui_background_color = UIBackgroundColor::RED;

      thread_sleep_for(500);

      led2 = 0;
      ui_background_color = UIBackgroundColor::DARK_CYAN;

      thread_sleep_for(500); 

    }

  }

  led1 = 0;
  led2 = 0;

  ui_state_descriptor = ui_ready_statement;  // after displaying the result the device is ready again

}

/**
 * Use onboard LEDs to indicate which mode we're entering (blinking red LED means unlock mode and steady red LED means record mode).
 * 
 * @param current_mode the current mode (RECORD or UNLOCK)
 * 
 * @author Ishan Taldekar
 */
void indicateMode(DeviceMode current_mode) {

  if (current_mode == DeviceMode::UNLOCK) {

    for (int8_t i = 0; i < 3; i++) {

      led2 = 1;
      thread_sleep_for(500);

      led2 = 0;
      thread_sleep_for(500);

    }

  } else if (current_mode == DeviceMode::RECORD) {
    
    led2 = 1;

    thread_sleep_for(3000);

    led2 = 0;

  }

}

/**
 * UI display loop that renders all the UI components to the LCD.
 * 
 * @author Ishan Taldekar
 */
void uiDisplayLoop() {

  std::string header = "User Profile";

  while (true) {
      
    if (ui_background_color == UIBackgroundColor::DARK_CYAN) {
        
      lcd.Clear(LCD_COLOR_DARKCYAN);
      lcd.SetBackColor(LCD_COLOR_DARKCYAN);

    } else if (ui_background_color == UIBackgroundColor::GREEN) {
        
      lcd.Clear(LCD_COLOR_GREEN);
      lcd.SetBackColor(LCD_COLOR_GREEN);
    
    } else {
        
      lcd.Clear(LCD_COLOR_RED);
      lcd.SetBackColor(LCD_COLOR_RED);

    }

    lcd.SetTextColor(LCD_COLOR_WHITE);

    lcd.DisplayStringAt(0, LINE(1), (uint8_t *)header.c_str(), CENTER_MODE);

    char user_profiles_labels[2] = {'5', NULL};

    for (int8_t i = 5; i >= 1; --i) {  // Display numbers for the 5 user profiles available

      lcd.DisplayStringAt(0, LINE((i * 3) + 1), (uint8_t *) user_profiles_labels, CENTER_MODE);
      --user_profiles_labels[0];

    }
    
    lcd.DrawHLine(0, 0, lcd.GetXSize());
    lcd.DrawHLine(0, lcd.GetYSize() - 1, lcd.GetXSize());
    lcd.DrawHLine(0, 40, lcd.GetXSize());
    lcd.DrawHLine(0, 290, lcd.GetXSize());

    lcd.SetTextColor(LCD_COLOR_ORANGE);
    lcd.DisplayStringAt(10, LINE(18.5), (uint8_t *) ui_state_descriptor.c_str(), LEFT_MODE);  // status description

    lcd.SetTextColor(LCD_COLOR_BLACK);
    lcd.DrawRect(100, user_profile_ui_squares_y_positions[current_user_profile], 30, 30);  // draw the rectangle to indicate the current user profile

    thread_sleep_for(100);

  }

}

/**
 * Return the user profile code associated with a touch in a particular region. (0 --> user profile 1, 4 --> user profile 5)
 * 
 * @param y the y position of the touch event
 * 
 * @author Ishan Taldekar
 */
int8_t getUserProfileNumber(int32_t y) {

  if (y < 260 && y >= 210) return 4;
  else if (y < 210 && y >= 165) return 3;
  else if (y < 165 && y >= 135) return 2;
  else if (y < 135 && y >= 80) return 1;
  else if (y < 80 && y >= 30) return 0;
  else return -1;

}

/**
 * Detect touch events on the LCD and use it to set the current profile for recording or unlocking.
 * 
 * @authors Inderpreet Walia, Ishan Taldekar
 */
void uiEventsLoop() {

  BSP_TS_Init(lcd.GetXSize(), lcd.GetYSize());

  TS_StateTypeDef ts_state;

  while (true) {

    BSP_TS_GetState(&ts_state);

    if (ts_state.TouchDetected && !user_profile_locked_flag) {

      int8_t new_user_profile = getUserProfileNumber(ts_state.Y);
      current_user_profile = new_user_profile == -1 ? current_user_profile : new_user_profile;
      thread_sleep_for(100);

    }

  }

}

/**
 * Setup device and threads, and interpret button presses.
 * 
 * @authors Jack Shkifati, James Huang, Ishan Taldekar
 */ 
int main() {

  // Setup the spi for 8 bit data, high steady state clock,
  // second edge capture, with a 1MHz clock rate
  spi.format(8, 3);
  spi.frequency(1'000'000);

  // Configure the registers
  write_buf[0] = CTRL_REG1;
  write_buf[1] = CTRL_REG1_CONFIG;
  spi.transfer(write_buf, 2, read_buf, 2, spi_cb, SPI_EVENT_COMPLETE);
  flags.wait_all(SPI_FLAG);

  // Configure the output data register
  write_buf[0] = CTRL_REG4;
  write_buf[1] = CTRL_REG4_CONFIG;
  spi.transfer(write_buf, 2, read_buf, 2, spi_cb, SPI_EVENT_COMPLETE);
  flags.wait_all(SPI_FLAG);

  // configure the interrupt to call our function
  // when the pin becomes high
  int2.rise(&data_cb);
  write_buf[0] = CTRL_REG3;
  write_buf[1] = CTRL_REG3_CONFIG;
  spi.transfer(write_buf, 2, read_buf, 2, spi_cb, SPI_EVENT_COMPLETE);
  flags.wait_all(SPI_FLAG);

  // The gyroscope sensor keeps its configuration between power cycles.
  // This means that the gyroscope will already have it's data-ready interrupt
  // configured when we turn the board on the second time. This can lead to
  // the pin level rising before we have configured our interrupt handler.
  // To account for this, we manually check the signal and set the flag
  // for the first sample.
  if (!(flags.get() & DATA_READY_FLAG) && (int2.read() == 1)) {

    flags.set(DATA_READY_FLAG);
  
  }

  Thread ui_display_thread;
  ui_display_thread.start(uiDisplayLoop);

  Thread ui_events_thread;
  ui_events_thread.start(uiEventsLoop);

  bool button_pressed = false;  // button state
  int button_hold_time = 0;  // button time

  while (true) {
    
    bool current_state = button.read();
    
    /* 
      Press and hold the button for 2s, red light will turn on for 3 seconds after which device will get into record mode.
      Press and release within 2s, red light will blink, and device will go into unlock mode on (function needed)
    */
    if (current_state && !button_pressed) {   // Button was just pressed

      button_pressed = true;
      button_hold_time = 0;
    
    } else if (current_state && button_pressed) {  // Button is being held down

      button_hold_time++;

      if (button_hold_time >= 500) {  // if button held down for 4 seconds then enter record mode
              
        user_profile_locked_flag = true;  // user cannot change the profile after this point  
        
        indicateMode(DeviceMode::RECORD);  // show that user is currently in record mode
        senseAndProcessGesture(DeviceMode::RECORD);  // start recording new gesture
        displayResult(DeviceMode::RECORD, true);  // show that recording was successful
        
        user_profile_locked_flag = false;  // user can change the profile now
      
      }

    } else if (!current_state && button_pressed) {  // Button was just released

      if (button_hold_time < 500) {  // button released under 4 seconds so enter unlock mode
          
        user_profile_locked_flag = true;  // user cannot change the profile 

        indicateMode(DeviceMode::UNLOCK);  // show that user is currently in unlock mode
        bool result = senseAndProcessGesture(DeviceMode::UNLOCK);  // record new gesture and compare with the existing gesture on the go
        displayResult(DeviceMode::UNLOCK, result);  // display the result

        user_profile_locked_flag = false;  // user can change the profile beyond this point

      }

      button_pressed = false;
      button_hold_time = 0;

    }

    thread_sleep_for(1); 

  } 
  
}
