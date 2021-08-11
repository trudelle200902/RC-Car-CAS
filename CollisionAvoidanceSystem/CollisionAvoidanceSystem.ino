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
    - Arduino Pin 3(Input, Interrupt) to UDS Echo Pin(Output)
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
    if rc is stopped wait for user to go on neutral then give back control to user but in MODE_SAFE
    if in safe mode and user get away from wall disable safe mode
    if user go foward in MODE_SAFE go 10% of speed
    if user is in MODE_SAFE and get really close to a wall user enter danger mode
    if user enter MODE_DANGER stop car until user go to neutral and allow only reverse and neutral contrl
    if user is in MODE_DANGER, it is enabled until user isn't near wall

    try fixing problem that when starting rc cant automatically take controll or make sure that if user
    start car infront of obstacle it at least wait until the connection is well made before taking control
*/
#include <Servo.h>//to help sending pwm signal to ESC 

#define PWM_IN_PIN 2//interrupt pin intercepting signal sent by receiver to ESC
#define PWM_OUT_PIN 8//output pin sending back intercepted signal to the ESC
#define ECHO_PIN 3//interrupt pin receiving the signal sent back by the UDS representing the time to receive signal after sending it in us
#define TRIGGER_PIN 4//output pin sending a signal of 10 us to the HC-Sr04 to activate it

#define MODE_NORMAL 0
#define MODE_SAFE 1
#define MODE_DANGER 2
#define REVERSE 0
#define NEUTRAL 1
#define DRIVE 2
#define BACKWARD 0
#define IMMOBILE 1//change bame
#define FOWARD 2

#define VAL_BREAK 1000//width in Micro seconds of PWM signal associated to breaking
#define VAL_NEUTRAL 1470
//#define TIMERBREAKDELAY 200000
#define DELAY_TIMER_TRIGGER 200000
#define DELAY_TIMER_BREAK_OVERRIDE 2000000

const int BREAK_DISTANCES[] = {1700, 700, 300};//in mm

Servo my_transmitter;
//use enum instead
byte car_mode;//0 = normal(user have complete control), 1 = safe mode(10% foward speed), 2 = danger mode()
byte car_state;//0 moving back 1 not moving 2 moving foward
//byte controllerState =1;//0 reverse 1 neutral 2 foward

volatile unsigned long rx_pulse_length;
volatile float wall_distance;
volatile bool is_accurate_distance;

////////testing/debuging variable////////
#define BLUE_PIN 10
#define RED_PIN 11
#define YELLOW_PIN 12
#define GREEN_PIN 13
volatile bool is_using_serial_debug = false;
bool prob = false;


//////////////////////////////////////CODE START//////////////////////////////////////

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
  static bool is_breaking = false;
  static bool is_user_controlling = true;
  static long timer_break_override;
  static long timer_trigger;

  if (timer_trigger <= micros()) {
    send_pulse_uds();
    car_state = read_car_state();//just put it in  send_pulse_uds()
    timer_trigger = micros() + DELAY_TIMER_TRIGGER;
  }

  if (!is_user_controlling) {
    if (car_state != FOWARD && is_breaking) {
      timer_break_override = micros() + DELAY_TIMER_BREAK_OVERRIDE;
      is_breaking = false;
    }else if(car_state == FOWARD && !is_breaking){
      is_breaking = true;
    }
    //check if car is immobile if the break timer has ended
    if (((timer_break_override <= micros() && car_mode == MODE_SAFE) || get_controller_state() == NEUTRAL) && !is_breaking) {
      //is_breaking = false;
      is_user_controlling = true;
    }
    if (wall_distance <= BREAK_DISTANCES[car_mode]) {
      car_mode++;
      if (car_mode > MODE_DANGER) {
        car_mode = MODE_DANGER;
      }
      if (car_state == FOWARD) {
        is_breaking = true;
        is_user_controlling = false;
      }
    }
  } else if (wall_distance <= BREAK_DISTANCES[car_mode]) {
    car_mode++;
    if (car_mode > MODE_DANGER) {
      car_mode = MODE_DANGER;
    }
    if (car_state == FOWARD) {
      is_breaking = true;
      is_user_controlling = false;
    }
  } else if (car_mode != MODE_NORMAL && wall_distance >= BREAK_DISTANCES[car_mode - 1]) {
    car_mode--;
  }


  long my_pulse_length;
  if (is_breaking && get_controller_state() == DRIVE) {
    my_pulse_length = VAL_BREAK;
  } else if (!is_user_controlling && get_controller_state() == DRIVE) {
    my_pulse_length = VAL_NEUTRAL;
  } else if (car_mode == MODE_SAFE && get_controller_state() == DRIVE) {
    my_pulse_length = ((rx_pulse_length - 1495) * 0.25) + 1495;
  } else if (car_mode == MODE_DANGER && get_controller_state() == DRIVE) {
    my_pulse_length = VAL_NEUTRAL;
  } else {
    my_pulse_length = rx_pulse_length;
  }

  my_transmitter.writeMicroseconds(my_pulse_length);

  //////////DEBUG//////////
  if (is_using_serial_debug) {
    write_serial_debug(my_pulse_length, is_user_controlling, is_breaking);
  }
  write_debug_led_state(is_breaking);
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
  //checkControllerState();
}


void handle_uds_echo_isr() {
  static long time_start_echo;
  //test = true;
  if (digitalRead(ECHO_PIN) == HIGH) {
    time_start_echo = micros();
  } else {
    float echo_length = micros() - time_start_echo;
    float temp_wall_distance = (echo_length / 2) * 0.34;//distance en mm
    if (temp_wall_distance != 0 && temp_wall_distance < 4000) {
      wall_distance = temp_wall_distance;
      is_accurate_distance = true;
    } else {
      is_accurate_distance = false;
    }
  }
}

void send_pulse_uds() {
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
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
void write_debug_led_state(bool is_breaking) {
  digitalWrite(BLUE_PIN, is_breaking && get_controller_state() == DRIVE);
  digitalWrite(GREEN_PIN, car_mode == MODE_NORMAL);
  digitalWrite(YELLOW_PIN, car_mode == MODE_SAFE);
  digitalWrite(RED_PIN, car_mode == MODE_DANGER);
}

void serialEvent() {
  if (!is_using_serial_debug) {

    is_using_serial_debug = true;
    Serial.println("DEBUG");
  }
  if (Serial.available() > 0) {
    rx_pulse_length = Serial.readString().toInt();
  }
}

void write_serial_debug(long my_pulse_length, bool is_user_controlling, bool is_breaking) {
  static long testTimer;
  if (testTimer <= micros()) {
    //checkControllerState();
    Serial.println(wall_distance);
    Serial.println(my_pulse_length);
    Serial.print(is_user_controlling); Serial.println(is_breaking);
    testTimer = micros() + 1000000;
  }
}
