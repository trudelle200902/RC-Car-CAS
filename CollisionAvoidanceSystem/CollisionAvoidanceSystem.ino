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
 * TODO
 *  if rc in neutral and near a wall dont go on reverse
 *  if rc is stopped wait for user to go on neutral then give back control to user but in safeMode 
 *  if in safe mode and user get away from wall disable safe mode
 *  if user go foward in safeMode go 10% of speed
 *  if user is in safeMode and get really close to a wall user enter danger mode
 *  if user enter dangerMode stop car until user go to neutral and allow only reverse and neutral contrl
 *  if user is in dangerMode, it is enabled until user isn't near wall
 */
#include <Servo.h>//to help sending pwm signal to ESC 

const int IN_PWM = 2;//interrupt pin intercepting signal sent by receiver to ESC
const int OUT_PWM = 8;//output pin sending back intercepted signal to the ESC
const int ECHO = 3;//interrupt pin receiving the signal sent back by the UDS representing the time to receive signal after sending it in us
const int TRIGGER = 4;//output pin sending a signal of 10 us to the HC-Sr04 to activate it

const long BREAKVAL = 1000;//width in Micro seconds of PWM signal associated to breaking

long breakDistance = 10;//in cm
Servo transmitedPwm;

volatile bool breaking = false, sendNewTrig = true, immobile;
volatile unsigned long pulseStartTime, pulseWidth, lastPulse, echoStart, echoDuration, wallDistance;

/////////////////testing/debuging variable
const bool TESTING = false;
const bool DEBUGING = false;
//volatile int signalState = LOW;
//const int arrayLength = 100;
//volatile int rxValue[arrayLength];
//volatile unsigned int index = 0;


/*
   method called at the start of program. Set
   the pinMode, asign ISR to interrups pin
   and set pin that return new PWM signal
*/
void setup() {
  if (TESTING || DEBUGING) {
    Serial.begin(9600);
  }

  pinMode(IN_PWM, INPUT);
  pinMode(ECHO, INPUT);
  pinMode(TRIGGER, OUTPUT);
  transmitedPwm.attach(OUT_PWM);

  attachInterrupt(digitalPinToInterrupt(IN_PWM), calcSignal, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO), calcDistance, CHANGE);

  digitalWrite(TRIGGER, LOW);//not sure is useful
}

/*
   Main loop
   - transmit signal to the ESC depending on value of breaking
*/
void loop() {
  if (breaking) {
    transmitedPwm.writeMicroseconds(BREAKVAL);
  } else {
    transmitedPwm.writeMicroseconds(pulseWidth);
  }
  if (sendNewTrig) {
    sendNewTrig = false;
    digitalWrite(TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER, LOW);
  }
  delay(300);
}

/*
   Interrupt method called when the tension change
   on the receiver pin.It is used to calculate the value
   of the PWM signal sent by the receiver.
*/
void calcSignal() {
  if (digitalRead(IN_PWM) == HIGH) {
    pulseStartTime = micros();
  } else {
    pulseWidth = micros() - pulseStartTime;
  }
}

void calcDistance() {
  if (digitalRead(ECHO) == HIGH) {
    echoStart = micros();
  } else {
    echoDuration = micros() - echoStart;
    wallDistance = (echoDuration / 2) * 0.034;//distance en cm
    if (wallDistance != 0) {
      breaking = (wallDistance <= breakDistance);
      //Serial.println(wallDistance);
      sendNewTrig = true;
    }
  }
}



/*
   when debugging called to manually break
   car when an input is sent on Serial monitor
*/
void serialEvent() {
  if (TESTING || DEBUGING) {
    breaking = true;
  }
}
