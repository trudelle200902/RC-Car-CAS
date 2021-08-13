/*
   CollisionAvoidanceSystem

   This program is a collison avoidance system for an RC car. This
   program read the signal(PWM) sent by the receiver that is supposed
   to be received by the ESC(Electronic Speed Controller/Motor controller) and
   return them to the ESC using the servo library to create pwm signal but if
   the Ultrasonic sensor detect the RC car will hit a wall send the pwm value
   associated with breaking to the ESC instead of the value sent by receiver
   to prevent collision.

   Component:
    - Arduino Board used: Arduino UNO
    - UDS: Ultrasonic Distance Sensor(HC-SR04) https://www.elecfreaks.com/blog/post/hc-sr04-ultrasonic-module-user-guide.html https://www.sparkfun.com/products/15569
    - BD-LLC: Bi-Direcrional Logic Level Converter(BOB-12009) https://learn.sparkfun.com/tutorials/bi-directional-logic-level-converter-hookup-guide/all
    - SMPS: 5V Step-Up/Step-Down Voltage Regulator(S9V11F5) https://www.pololu.com/product/2836

    - Rc Car used: Latrax Rally
        - ESC(built in): Traxxas 3045r LaTrax Waterproof Electronic Speed Control
        - Rx(built in): Traxxas 3046 LaTrax 3-channel 2.4GHz Micro Receiver
        - Motor(built in): Traxxas 7575X Brushed DC Motor 370 (28-Turn)
        - 3 capacitor of 100000pF(built in) on the Dc Motor(C1, C2, C3)
        - Servo Motor(built in): Traxxas 2065 Sub-Micro Waterproof Servo
        - Transmitter: Traxxas 3047 LaTrax 2.4GHz, 2-channel (transmitter only)
        - Battery: 6.0v NiMh 1200mAh 5 cell


   Circuit(schematic and complete circuit the RC on link bellow (not yet)):
    - ESC Vout to rx Ch2 Vin, SMPS Vin, BD-LLC HV(Input)
    - ESC Gnd to Arduino Gnd(In), SMPS Gnd, rx Ch2 Gnd, BD-LLC Gnd for Hv
    - Arduino Vin to SMPS Vout
    - Arduino 5v(Out) to BD-LLC LV(Input), UDS Vcc(In)
    - Arduino Gnd to BD-LLC Gnd for Lv, UDS Gnd
    - Arduino Pin 2(Input, Interrupt) to BD-LLC Lv Ch1 (Bidirectional)
    - Arduino Pin 3(Input, Interrupt) to UDS ECHO_PIN Pin(Output)
    - Arduino Pin 4(Output) to UDS Trig Pin(Input)
    - Arduino Pin 8(Output) to ESC Pwm (Input)
    - rx CH2 Pwm (Output) to BD-LLC Hv Ch1 (Bidirectional)

    //not sure i should put rest of the circuit
    - Battery+ to ESC Vin
    - Battery- to ESC Gnd(in)
    - ESC Vout to Motor+, C1, C3
    - ESC Gnd to Motor-, C2, C3
    - Gnd to C1, C2
    - rx CH1 Vout to Servo Vin
    - rx CH1 Gnd to Servo Gnd
    - rx CH1 Pmw (Output) to Servo Pwm(Input)


   Width of PWM signal in MicroSecond associated to action:
    aproximately 1495 to 2000 or more foward(2000 is full throtle)
    aproximately 1445 to aproximately 1495 = neutral
    1000 to aproximately 1445 = reverse or break if Rc is going foward smaller num = more breaking power

   Project Start: 2 August 2021
    By Mathieu Trudelle
   Last Modified: 2 August 2021
    By Mathieu Trudelle
*/
///////////////////////////////////////////////////////////
/*
   TODO
    if rc in neutral and near a wall dont go on reverse
    if rc is stopped wait for user to go on neutral then give back control to user but in safeMode
    if in safe mode and user get away from wall disable safe mode
    if user go foward in safeMode go 10% of speed
    if user is in safeMode and get really close to a wall user enter danger mode
    if user enter dangerMode stop car until user go to neutral and allow only reverse and neutral contrl
    if user is in dangerMode, it is enabled until user isn't near wall

    try fixing problem that when starting rc cant automatically take controll or make sure that if user
    start car infront of obstacle it at least wait until the connection is well made before taking control
*/
#include <Servo.h>//to help sending pwm signal to ESC 

#define PWM_IN_PIN 2//interrupt pin intercepting signal sent by receiver to ESC
#define PWM_OUT_PIN 8//output pin sending back intercepted signal to the ESC
#define ECHO_PIN 3//interrupt pin receiving the signal sent back by the UDS representing the time to receive signal after sending it in us
#define TRIGGER_PIN 4//output pin sending a signal of 10 us to the HC-Sr04 to activate i

#define REVERSE 0
#define NEUTRAL 1
#define DRIVE 2
#define BACKWARD 0
#define IMMOBILE 1//change bame
#define FOWARD 2

//#define DELAY_TIMER_TRIGGER 200000
#define VAL_BREAK 1000//width in Micro seconds of PWM signal associated to breaking
#define VAL_NEUTRAL 1470

#define BREAK_DISTANCE 1500//in mm

Servo my_transmitter;
bool is_breaking = false;
//bool is_user_restricted = true;
bool is_neutral = false;
long timer_trigger;
byte car_state;//0 moving back 1 not moving 2 moving foward

volatile float wall_distance;
volatile bool is_accurate_distance;

//volatile bool sendNewTrig = true, immobile;
volatile unsigned long rx_pulse_length;
//volatile unsigned long pulseStartTime, pulseWidth, lastPulse, ECHO_PINStart, ECHO_PINDuration, wallDistance;

/////////////////testing/debuging variable
#define BLUE_PIN 10
#define RED_PIN 11
#define YELLOW_PIN 12
#define GREEN_PIN 13
volatile bool is_using_serial_debug = false;


/*
   method called at the start of program. Set
   the pinMode, asign ISR to interrups pin
   and set pin that return new PWM signal
*/
void setup() {
  Serial.begin(9600);

  pinMode(PWM_IN_PIN, INPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  my_transmitter.attach(PWM_OUT_PIN);

  attachInterrupt(digitalPinToInterrupt(PWM_IN_PIN), handle_rx_signal_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), handle_uds_echo_isr, CHANGE);

  digitalWrite(TRIGGER_PIN, LOW);//not sure is useful

  pinMode(BLUE_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
}

/*
   Main loop
   - transmit signal to the ESC depending on value of breaking
*/
void loop() {
  long my_pulse_length;

  if (timer_trigger <= micros()) {
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);

    car_state = read_car_state();
    timer_trigger = micros() + 200000/*DELAY_TIMER_TRIGGER*/;//set t
  }

  //  if (sendNewTrig) {
  //    sendNewTrig = false;
  //    digitalWrite(TRIGGER_PIN, HIGH);
  //    delayMicroseconds(10);
  //    digitalWrite(TRIGGER_PIN, LOW);
  //  }

  if (wall_distance <= BREAK_DISTANCE && get_controller_state() == DRIVE && car_state == FOWARD && !(is_neutral || is_breaking)) {
    is_breaking = true;
  }else if (is_breaking && get_controller_state() == NEUTRAL && car_state != FOWARD) {
    is_neutral = true;
    is_breaking = false;
  } else if (is_neutral && wall_distance > BREAK_DISTANCE/* || get_controller_state() == NEUTRAL*/) {
    is_neutral = false;
  } /*else if (wall_distance <= BREAK_DISTANCE) {
    is_breaking = true;
  }*/

  if (is_breaking/* && get_controller_state() == DRIVE && car_state == FOWARD*/) {
    my_pulse_length = VAL_BREAK;
    digitalWrite(BLUE_PIN, HIGH);
  } else if(is_neutral && get_controller_state() != REVERSE){
    my_pulse_length = VAL_NEUTRAL;
    digitalWrite(BLUE_PIN, LOW);
  } else {
    my_pulse_length = rx_pulse_length;
    digitalWrite(BLUE_PIN, LOW);
  }
  my_transmitter.writeMicroseconds(my_pulse_length);

  //////////DEBUG//////////
  if (is_using_serial_debug) {
    write_serial_debug(my_pulse_length);
  }
  write_debug_led_state();
}

/*
   Interrupt method called when the tension change
   on the receiver pin.It is used to calculate the value
   of the PWM signal sent by the receiver.
*/
void handle_rx_signal_isr() {
  static long time_start_pulse;
  if (digitalRead(PWM_IN_PIN) == HIGH) {
    time_start_pulse = micros();
  } else {
    rx_pulse_length = micros() - time_start_pulse;
  }
}

void handle_uds_echo_isr() {
  static long time_start_echo;

  if (digitalRead(ECHO_PIN) == HIGH) {
    time_start_echo = micros();
  } else {
    float echo_length = micros() - time_start_echo;
    //wall-distance = (ECHO_PINDuration / 2) * 0.34;//distance en mm
    float temp_wall_distance = (echo_length / 2) * 0.34;//distance en mm
    if (temp_wall_distance != 0 && temp_wall_distance < 4000) {
      wall_distance = temp_wall_distance;
      is_accurate_distance = true;
    } else {
      is_accurate_distance = false;
    }

  }
}

byte get_controller_state() {
  if (1445 > rx_pulse_length) {
    return  0;
  } else if (1495 > rx_pulse_length) {
    return 1;
  } else {
    return 2;
  }
}

byte read_car_state() {
  if (is_accurate_distance) {
    static float last_position;
    byte state;
    if (last_position - wall_distance < -5) {
      state = 0;
    } else if (last_position - wall_distance < 5) {
      state = 1;
    } else {
      state = 2;
    }
    last_position = wall_distance;
    return state;
  } else {
    return car_state;
  }
}

//////////////////////////////////////////DEBUG FUNCTION/////////////////////////////////////////////
void write_debug_led_state() {
  //digitalWrite(BLUE_PIN, is_breaking && get_controller_state() == DRIVE);
  //digitalWrite(GREEN_PIN, car_mode == MODE_NORMAL);
  //digitalWrite(YELLOW_PIN, car_mode == MODE_SAFE);
  //digitalWrite(RED_PIN, car_mode == MODE_DANGER);
  digitalWrite(GREEN_PIN, !(is_breaking || is_neutral));
  digitalWrite(YELLOW_PIN, is_neutral);
  digitalWrite(RED_PIN, is_breaking);
}

/*
   when debugging called to manually break
   car when an input is sent on Serial monitor
*/
void serialEvent() {
  if (!is_using_serial_debug) {

    is_using_serial_debug = true;
    Serial.println("DEBUG");
  }
  if (Serial.available() > 0) {
    rx_pulse_length = Serial.readString().toInt();
  }
}

void write_serial_debug(long my_pulse_length) {
  static long test_timer;
  if (test_timer <= micros()) {
    Serial.println(wall_distance);
    Serial.println(my_pulse_length);
    Serial.print(is_breaking); Serial.println(is_neutral);
    test_timer = micros() + 1000000;
  }
}
