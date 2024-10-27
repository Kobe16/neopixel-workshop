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
// Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Setup ROS handle (in setup() function too!)
// ros::NodeHandle nh;


// Declare global static variables for different modes: 
// autonomous: red, manual: blue, disabled: yellow, goal_reached: green, led_off: black
// static uint32_t RED = pixels.color(150, 0, 0); 
// static uint32_t GREEN = pixels.color(0, 150, 0); 
// static uint32_t BLUE = pixels.color(0, 0, 150);
// static uint32_t YELLOW = pixels.color(200, 200, 0);
// static uint32_t BLACK = pixels.color(0, 0, 0);

// static uint32_t autonomous = RED;
// static uint32_t manual = BLUE;
// static uint32_t disabled = YELLOW; 
// static uint32_t goal_reached = GREEN;
// static uint32_t led_off = BLACK;


// Hint: use a global state variable to track mode
// static uint32_t state = disabled;  // start out disabled

// void refresh_color() {
//   for (int i = 0; i < NUMPIXELS; i++) pixels.setPixelColor(i, state);  // add neopixel code
// }

void on_state(const std_msgs::String &msg) {
// Define callback function for everytime you receive a new state message

//   if (strcmp(msg.data, "manual") == 0) state = manual;
//   else if (strcmp(msg.data, "autonomous") == 0) state = autonomous;
//   else if (strcmp(msg.data, "off") == 0) state = disabled;
//   else if (strcmp(msg.data, "goal_reached") == 0) {
//     if (millis() % 1000 < 500) state = goal_reached;
//     else state = led_off;  // flash green per competition
//   }
//   refresh_color();
}


// Define your ROS subscriber to listen to state messages
// rostopic: "/state"
// datatype: std_msgs::String
// ros::Subscriber<std_msgs::String> state_sub("/state", &on_state);

void setup() {

    // Setup neopixel code and pin

//   #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
//   clock_prescale_set(clock_div_1);
//   #endif
//   pixels.begin();
//   pinMode(LED_PIN, OUTPUT);

// Setup ROS
//   nh.initNode();
//   nh.subscribe(state_sub);
//   nh.advertise(battery_pub);
}

void loop() {

// Implement newpixel logic 
// Hint: make sure to call spinOnce here!
//   refresh_color();
//   pixels.show();
//   nh.spinOnce(); // must be called for rosserial to work
//   delay(10);
}
