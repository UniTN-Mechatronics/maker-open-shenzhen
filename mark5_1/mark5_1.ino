#include <Makeblock.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <FiniteStateMachine.h>
#include <clockwork.h>
#include "low_pass.h"
#include "encoder_motor_mod.h"


#define PROMPT Serial.println(">");

#define SLAVE_ADDRESS 0x03
byte g_eeprom_addr = 0x10;

#define LOOP_TIME 10 // milliseconds

#define CAL_SX -1
#define CAL_DX  1

#define MAX_RPM 300
#define RPM_TO_FORWARD 0.5 // forward = max * rpm_to_forward; yaw = max * (1-rpm_to_forward)

#define ARM_PIN 3
#define SERIAL_ENABLED_PIN 9

#define TIME_TO_TOGGLE 2000
#define TIME_TO_GOAWAY 1000

MeEncoderMotorMod motor_sx(0x09, SLOT2);
MeEncoderMotorMod motor_dx(0x09, SLOT1);

#define SPEED_UPDOWN 150
#define UPDOWN_CAL 1
MeDCMotor motor_updown(M1);
MeLimitSwitch limitSwitch_updown_up(PORT_3, SLOT1);
MeLimitSwitch limitSwitch_updown_down(PORT_3, SLOT2);

MeRGBLed led(PORT_6);

Clockwork loop_syncro(LOOP_TIME, NULL);

MeBluetooth bluetooth(PORT_5);

MeUltrasonicSensor range_finder(PORT_4);

MeLightSensor lightSensor_togglestate(PORT_7);
MeLightSensor lightSensor_goaway(PORT_8);

double light_togglestate;
double light_goaway;
double light_difference;
double light_difference_lowp;
double light_difference_lowp2;
double light_difference_hp;
Low_pass light_togglestate_lp(5);
Low_pass light_goaway_lp(5);
// Low_pass light_difference_lp(0.01, 100.0);
Low_pass light_difference_lp(0.01);

// laser follow params
#define LASER_KP_X 1.0
#define LASER_KP_C 1.0
Low_pass laser_error_x_lp(1.0);
Low_pass laser_error_y_lp(1.0);

//////////////////////////// STATE
long angle_sx = 0;
long angle_dx = 0;
struct {
  double forward_vel;
  double yaw_rate;
  double rpm_to_forward;
  int led_color;
  bool armed;

  bool remote;

  double laser_error_x; double laser_error_x_filt;
  double laser_error_y; double laser_error_y_filt;

  unsigned long time_last_transition;
} state;

// callback for received data via i2c
void receiveData(int byteCount) {
  static char cmd = 0;
  int val;
  while (Wire.available()) {
    val = Wire.read();
    if (cmd == 0) {
      cmd = (char)val;
    }
    else {
      command_parsing(cmd, val);
      cmd = 0;
    }
  }
}

//////////////////////////// STATE MACHINE
enum {
  PREINIT = 0,
  IDLE,
  ACTIVE,
  MANUAL,
  GOAWAY,
};

State PreInit     = State(PREINIT, wait_for_arming);
State Idle        = State(IDLE, godown, check_goaway, NULL); 
State Active      = State(ACTIVE, goup, laser_follower, NULL);
State Manual      = State(MANUAL, goup, manual, NULL);
State GoAway      = State(GOAWAY, goup, goaway, NULL);

FSM stateMachine = FSM(PreInit);     //initialize state machine

////////////////////////////// led color
enum {
  BLACK = 0,
  WHITE,
  GREEN,
  RED,
  BLUE,
  ORANGE,
  YELLOW
};

///////////////////////////////////////////////////////////////////////////////

void setup() {
  pinMode(SERIAL_ENABLED_PIN, OUTPUT);
  state.armed = FALSE;
  Serial.begin(115200);
  bluetooth.begin(115200);

  byte i2c_address = SLAVE_ADDRESS;

  Wire.begin(i2c_address);

  led.setNumber(24);

  pinMode(ARM_PIN, INPUT);

  state.time_last_transition = millis();

  Serial.println("init to reset");

  reset_state();

  // buzzerOn();
  // delay(300);
  // buzzerOff();

  Serial.flush();

  light_difference_lp.set_y0(double(lightSensor_togglestate.read() - lightSensor_goaway.read()) );

  Serial.println("end setup");
  digitalWrite(SERIAL_ENABLED_PIN, HIGH);
}

///////////////////////////////////////////////////////////////////////////////

void loop() {
  loop_syncro.start();

  static unsigned long buf = 0;
  static char esc_sequence;

  char ch;
  if(state.armed == FALSE) {delay(100); goto MINIMAL; };
  if (Serial.available()) {
    ch = Serial.read();

    // Serial command parsing:
    switch (ch) {
      case '0'...'9': // Accumulates values
        buf = buf * 10 + ch - '0';
        break;
      case 0x1B:
        Serial.read();
        esc_sequence = Serial.read();
        switch (esc_sequence) {
          case 0x44:
            // angle_sx
            stateMachine.transitionTo(Manual);
            break;
          case 0x41:
            stateMachine.transitionTo(Manual);
            break;
          case 0x42:
            stateMachine.transitionTo(Manual);
            break;
          case 0x43:
            stateMachine.transitionTo(Manual);
            break;
        }
        buf = 0;
        PROMPT;
        break;
      case 'V':
        state.forward_vel = buf/1000.0;
        buf = 0;
        stateMachine.transitionTo(Manual);
        PROMPT;
        break;
      case 'v':
        state.forward_vel = -(buf/1000.0);
        buf = 0;
        stateMachine.transitionTo(Manual);
        PROMPT;
        break;
      case 'Y':
        state.yaw_rate = buf/1000.0;
        buf = 0;
        stateMachine.transitionTo(Manual);
        PROMPT;
        break;
      case 'y':
        state.yaw_rate = -(buf/1000.0);
        buf = 0;
        stateMachine.transitionTo(Manual);
        PROMPT;
        break;
      case '/':
        buf = 0;
        stateMachine.transitionTo(Idle);
        PROMPT;
        break;
      case 'A':
        buf = 0;
        PROMPT;
        stateMachine.transitionTo(Active);
        break;
      case '?':
        buf = 0;
        print_state();
        PROMPT;
        break;
      default:
      buf = command_parsing(ch, buf);
        // buf = 0;
    }
  }


  motion();
  check_transition();
  
MINIMAL:
  update_led();
  stateMachine.update();
  
  loop_syncro.stop();

}

///////////////////////////////////////////////////////////////////////////////

void update_led() {
  int r = 0;
  int g = 0;
  int b = 0;
  switch (state.led_color) {
    case BLACK:
        r = 0; g = 0; b = 0;
        break;
      case WHITE:
        r = 100; g = 100; b = 100;
        break;
      case GREEN:
        r = 0; g = 100; b = 0;
        break;
      case RED:
        r = 100; g = 0; b = 0;
        break;
      case BLUE:
        r = 0; g = 0; b = 100;
        break;
      case ORANGE:
        r = 100; g = 44; b = 0;
      case YELLOW:
        r = 100; g = 100; b = 0;
        break;
  }
  for(int t=0; t<24; t++){
    led.setColorAt(t, r, g, b); // parameter description: led number, red, green, blue, flash mode
  }
  led.show();
}

///////////////////////////////////////////////////////////////////////////////

void motion() {
  state.rpm_to_forward = rpm_to_forward(state.yaw_rate, 0.3, 0.7);
  double rpm_sx = (state.forward_vel*state.rpm_to_forward - state.yaw_rate*(1-state.rpm_to_forward))*MAX_RPM;
  double rpm_dx = (state.forward_vel*state.rpm_to_forward + state.yaw_rate*(1-state.rpm_to_forward))*MAX_RPM;

  motor_sx.RunSpeed(CAL_SX*rpm_sx);
  motor_dx.RunSpeed(CAL_DX*rpm_dx);

  // if(state.forward_vel<0 && (millis()/400)%2 == 0) {
  //   buzzerOn();
  // }
  // else {
  //   buzzerOff();
  // }
}

///////////////////////////////////////////////////////////////////////////////

void reset_state() {
  // Serial.flush();

  // buzzerOff();

  state.forward_vel = 0;
  state.yaw_rate = 0;
  state.led_color = WHITE;

  state.laser_error_x = 0;
  state.laser_error_x_filt = 0;
  state.laser_error_y = 0;
  state.laser_error_y_filt = 0;

  state.remote = false;

  state.rpm_to_forward = 0.5;

  motor_sx.RunSpeed(0);
  motor_dx.RunSpeed(0);
  // motor_updown.run(0);

  Serial.println("state reseted");

}

///////////////////////////////////////////////////////////////////////////////

void wait_for_arming() {
  state.led_color = BLACK;
  if(digitalRead(ARM_PIN)){
    state.armed = TRUE;
    state.led_color = WHITE;
    Serial.flush();
    stateMachine.transitionTo(Idle);
  }
}

///////////////////////////////////////////////////////////////////////////////

void print_state() {
  Serial.print("state: ");
  Serial.print(stateMachine.getCurrentState().id());
  Serial.print(" forward_vel: ");
  Serial.print(state.forward_vel);
  Serial.print(" yaw_rate: ");
  Serial.print(state.yaw_rate);
  Serial.print(" led color: ");
  Serial.print(state.led_color);

}

///////////////////////////////////////////////////////////////////////////////

void manual() {
  state.led_color = BLUE;
 
}

///////////////////////////////////////////////////////////////////////////////

// void activate_i2c() {
//   Wire.onReceive(receiveData);
// }

// void deactivate_i2c() {
//   Wire.onReceive(NULL);
// }

void laser_follower() {
  state.led_color = GREEN;

  state.laser_error_x_filt = laser_error_x_lp.update(state.laser_error_x, LOOP_TIME/1000.0);
  state.laser_error_y_filt = laser_error_y_lp.update(state.laser_error_y, LOOP_TIME/1000.0);

  state.forward_vel = (state.laser_error_x_filt) * LASER_KP_X;
  state.yaw_rate    = (state.laser_error_y_filt) * LASER_KP_C;

  Serial.print(state.laser_error_y);
  Serial.print("\t");
  Serial.print(state.laser_error_y_filt);
  Serial.print("\t");
  Serial.println(state.yaw_rate);
}


int command_parsing(char ch, int v) {
  switch (ch) {
    case 'X':
      state.laser_error_x = v/100.0;
      v = 0;
      PROMPT;
      break;
     case 'x':
      state.laser_error_x = -v/100.0;
      v = 0;
      PROMPT;
      break;
    case 'C':
      state.laser_error_y = v/100.0;
      v = 0;
      PROMPT;
      break;
     case 'c':
      state.laser_error_y = -v/100.0;
      v = 0;
      PROMPT;
      break;
    default:
      v = 0;
      PROMPT;
  }
  return v;
}

///////////////////////////////////////////////////////////////////////////////

void check_transition() {
  light_togglestate = light_togglestate_lp.update(double(lightSensor_togglestate.read()), LOOP_TIME/1000.0);
  light_goaway = light_goaway_lp.update(double(lightSensor_goaway.read()), LOOP_TIME/1000.0);

  light_difference = (light_togglestate - light_goaway);

  light_difference_lowp = light_difference_lp.update(light_difference, LOOP_TIME/1000.0);

  light_difference_hp = light_difference - light_difference_lowp;

  // Serial.print(light_togglestate);
  // Serial.print("\t");
  // Serial.print(light_goaway);
  // Serial.print("\t");
  // Serial.println(light_difference_hp);


  if(millis() - state.time_last_transition > TIME_TO_TOGGLE && light_difference_hp > 100.0) {
    state.time_last_transition = millis();
    if(stateMachine.getCurrentState().id() == IDLE) {
      stateMachine.transitionTo(Active);
    }
    if(stateMachine.getCurrentState().id() == ACTIVE) {
      stateMachine.transitionTo(Idle);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////

void check_goaway() {
  static int counter = 0;
  if(light_difference_hp < -100.0) {
    counter++;
  }
  if(counter*LOOP_TIME > TIME_TO_GOAWAY) {
    stateMachine.transitionTo(GoAway);
  }
}

///////////////////////////////////////////////////////////////////////////////

void goaway() {
  state.led_color = RED;
  int distance = range_finder.distanceCm();
  if (distance < 50) {
    state.forward_vel = 0;
    state.yaw_rate = 0.6;
  } else {
    state.forward_vel = 0.5;
    state.yaw_rate = 0;
  }
}

///////////////////////////////////////////////////////////////////////////////

void goup() {
  while(!limitSwitch_updown_up.touched()){
    motor_updown.run(UPDOWN_CAL * SPEED_UPDOWN);
  }
    motor_updown.run(0);
}

///////////////////////////////////////////////////////////////////////////////

void godown() {
  reset_state();
  while(!limitSwitch_updown_down.touched()){
    motor_updown.run( - UPDOWN_CAL * SPEED_UPDOWN);
  }
    motor_updown.run(0);
}

///////////////////////////////////////////////////////////////////////// SPARE FUNCTIONS

int sgn(int x) {
  int y;
  x>0 ? y = 1 : y = -1;
  return y;
}

double rpm_to_forward(double yaw_rate, double min, double max) {
  return -abs(yaw_rate)*(max - min) + max;
}
