#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif
#define LED_PIN       5
#define NUMPIXELS     6


// Initizlize Adafruit_Neopixel LEDs object

// Declare global static variables for different modes: 
// autonomous: red, manual: blue, disabled: yellow, goal_reached: flashing green, led_off: black, 


// Hint: use a global state variable to track mode


void on_state(const std_msgs::String &msg) {
// Define callback function for everytime you receive a new state message

}


// Define your ROS subscriber to listen to state messages
// rostopic: "/state"
// datatype: std_msgs::String

void setup() {

    // Setup neopixel code and pin

// Setup ROS
}

void loop() {

// Implement newpixel logic 
// Hint: make sure to call spinOnce here!
}
