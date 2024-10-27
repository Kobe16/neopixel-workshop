#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif
#define LED_PIN       5
#define NUMPIXELS     6

Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Cell sensing pins
#define B1_C1 A1
#define B1_C2 A2
#define B1_C3 A3
#define B2_C1 A4
#define B2_C2 A5
#define B2_C3 A6

#define NUM_CELLS 6
#define CELLS_PER_BATT 3
#define CELL_READ_WINDOW 25
#define BATTERY_READ_PERIOD 1000

// Cell sensing calibration constants (calibrated using reference sweep + LOBF)
#define C1_A 0.0043845
#define C1_B -0.0119552

#define C2_A 0.00427625
#define C2_B -0.012834

#define C3_A C2_A // very close to the same (due to same differential measurement circuit)
#define C3_B C2_B

ros::NodeHandle nh;

static uint32_t RED = pixels.color(150, 0, 0); 
static uint32_t GREEN = pixels.color(0, 150, 0); 
static uint32_t BLUE = pixels.color(0, 0, 150);
static uint32_t YELLOW = pixels.color(200, 200, 0);
static uint32_t BLACK = pixels.color(0, 0, 0);

static uint32_t autonomous = RED;
static uint32_t manual = BLUE;
static uint32_t disabled = YELLOW; 
static uint32_t goal_reached = GREEN;
static uint32_t led_off = BLACK;

int battery_pins[6] = {B1_C1, B1_C2, B1_C3, B2_C1, B2_C2, B2_C3};
float Aconstants[3] = {C1_A, C2_A, C3_A};
float Bconstants[3] = {C1_B, C2_B, C3_B};
unsigned long last_battery_reading = 0;

static uint32_t state = disabled;  // start out disabled

void refresh_color() {
  for (int i = 0; i < NUMPIXELS; i++) pixels.setPixelColor(i, state);  // add neopixel code
}

void on_state(const std_msgs::String &msg) {
  if (strcmp(msg.data, "manual") == 0) state = manual;
  else if (strcmp(msg.data, "autonomous") == 0) state = autonomous;
  else if (strcmp(msg.data, "off") == 0) state = disabled;
  else if (strcmp(msg.data, "goal_reached") == 0) {
    if (millis() % 1000 < 500) state = goal_reached;
    else state = led_off;  // flash green per competition
  }
  refresh_color();
}

ros::Subscriber<std_msgs::String> state_sub("/state", &on_state);
std_msgs::Float64MultiArray battery_levels;
ros::Publisher battery_pub("/battery_level", &battery_levels);

void setup() {
  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
  #endif

  pixels.begin();
  battery_levels.data_length = NUM_CELLS;
    battery_levels.data = new float[NUM_CELLS];
    for (int i = 0; i < NUM_CELLS; i++)
      battery_levels.data[i] = 0;

  pinMode(LED_PIN, OUTPUT);
  nh.initNode();
  nh.subscribe(state_sub);
  nh.advertise(battery_pub);
}

void loop() {
  if (millis() - last_battery_reading > BATTERY_READ_PERIOD) {
    last_battery_reading = millis();
    handle_batteries();
  }
  if (should_panic()) {
    led_panic();
  }
  else {
    refresh_color();
  }
  pixels.show();
  nh.spinOnce(); // must be called for rosserial to work
  delay(10);
}

void led_panic()
{
  unsigned int time = millis();
  if ((time & 1 << i) == 0) {
    switch(i) {
      case 0: 
        for (int i = 0; i < NUMPIXELS; i++) pixels.setPixelColor(i, RED);
        break;
      case 1: 
        for (int i = 0; i < NUMPIXELS; i++) pixels.setPixelColor(i, GREEN);
        break;
      case 2: 
        for (int i = 0; i < NUMPIXELS; i++) pixels.setPixelColor(i, BLUE);
        break;
    }
  } else {
    for (int i = 0; i < NUMPIXELS; i++) pixels.setPixelColor(i, BLACK);
  }

}

bool should_panic() {
  bool should_panic = false;
  for (int cell = 0; cell < NUM_CELLS; cell++) {
    should_panic = should_panic || cell_should_panic(battery_levels.data[cell]);
  }
  return should_panic;
}

bool cell_should_panic(float reading) {
  return reading < 3.7 && reading > 2;
}

float num_read = 0;
void handle_batteries() {
  for (int cell = 0; cell < NUM_CELLS; cell++) {
    float read_val = linearRemap(analogRead(battery_pins[cell]), Aconstants[cell % CELLS_PER_BATT], Bconstants[cell % CELLS_PER_BATT]);
    if (read_val < 2) {
      continue;
    }
    battery_levels.data[cell] *= num_read / (num_read + 1);
    battery_levels.data[cell] += read_val / (num_read + 1);
  }
  battery_pub.publish(&battery_levels);
  num_read++;
  num_read = min(num_read, CELL_READ_WINDOW);
}

float linearRemap(float value, float calA, float calB) {
  return calA * value + calB;
}
