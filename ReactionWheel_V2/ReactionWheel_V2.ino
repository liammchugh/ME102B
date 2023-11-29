//MAIN CODE 
//::::::Libraries::::::://
#include <Arduino.h>
//#include <esp_now.h>
//#include <WiFi.h>
//#include <Wire.h>
//#include <SPI.h>
//#include "Adafruit_MAX31855.h"
#include <ESP32Encoder.h>
#include <Adafruit_MLX90640.h>

// DEFINE ESP PINS
#define BIN_1 26
#define BIN_2 25
#define PWM 39
#define LED_PIN 13
#define BTN 36


// DEFINE SENSOR READRATE
#define READ_DELAY 70
#define sendDelay 140

/////////////////////IR SETUP///////////////////////////
#define WIDTH  32
#define HEIGHT 24
#define RADIUS 8 // vertical distance to look at
Adafruit_MLX90640 mlx;
int iMax = 0;
int error = 0;
float frame[WIDTH*HEIGHT]; // buffer for full frame of temperatures


///////////////////////// Communication Setup /////////////////////////////////
int WIFIDEBUG = 1; // IF TOGGLED TO 1: Don't send/receive data.
int DEBUG = 1 ;
short int queueLength = 0;
// Create a struct_message called Packet to be sent.
//struct struct_message Packet;
//// Create a queue for Packet in case Packets are dropped.
//struct struct_message PacketQueue[120];

//::::::Broadcast Variables::::::://
//esp_now_peer_info_t peerInfo;
// REPLACE WITH THE MAC Address of your receiver
//uint8_t broadcastAddress[] = {0xB0, 0xA7, 0x32, 0xDE, 0xC1, 0xFC};
// Callback when data is sent
// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//  sendTime = millis();
// }

// Callback when data is received
//void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
//  memcpy(&Commands, incomingData, sizeof(Commands));
//  SERIALState = Commands.COMState;
//}


//::::::STATE VARIABLES::::::://
enum STATES {IDLE, CTDWN, SEEKMODE}; //add initialize state for countdown
String state_names[] = {"IDLE", "CTDWN", "SEEK"};
const float pi = 3.1415;
int DAQState;
int CommandState;
int ctdwn_time = 2000;
float angle = 180; // ANGLE FROM DESIRED ORIENTATION. Initiated as 180deg(pi)
float lastangle = 180; //used for speed deriv.
double targetAngle = 0; // Angle target point
float loopOnce=0; // Used to reset loop routine

int event_timer;
int del = 150;
//Setup variables ------------------------------------
volatile bool buttonIsPressed = false;

//Initialization ------------------------------------
void IRAM_ATTR isr() {  // the function to be called when interrupt is triggered
  if (millis()-event_timer > del) {
  buttonIsPressed = true; }
  Serial.println("Boop");
}

// setting PWM properties ----------------------------
const int freq = 5000;
const int ledChannel_1 = 1;
const int ledChannel_2 = 2;
const int ledChannel_PWM = 3;
const int resolution = 8;
int MAX_PWM_VOLTAGE = 240;
int motor_PWM;
int i = 0;
int pwm_sign;
float timed = 0;
int lasttime;
int sendTime;

///////////////////// Control Parameters ///////////////////////////
int ctrl_freq = 16;
int deltat = 1/16;
int max_rpm = 2000;
int lastcalc = 0;

////////////////////// Begin of PID parameters ///////////////////////////////
// Good YouTube video resource for PID's https://www.youtube.com/watch?v=0vqWyramGy8
double kp = 0.5; //75
double ki = 0; 
double kd = 0.0025; 
////////////////////// End of PID parameters /////////////////////////////////

///////////////////////////////// Begin of PI speed loop Vars -UNUSED- //////////////////////
double kp_speed =  10; 
double ki_speed = 0.72;
double kd_speed = 0; // NOT USED  
float pwmOut=0;
float pwmOut2=0; 
///////////////////////////////// End of PID speed loop Vars //////////////////////


//////////////////////////////// Begin of PI_pwm Vars //////////////////////////
float speeds_filterold=0;
float positions=0;
int cc;
float speeds_filter;
//////////////////////////////// End of PI_pwm Vars /////////////////////////////

////////////////////// Begin of pulse count /////////////////////////
int rw = 0;
int pulseCount = 0;
int rwPulse;
////////////////////// End of pulse count //////////////////////////
int loopcount; // Counter var for D13 onboard LED flash evey 1 second




//######################## Begin of Kalman Filter UNUSED ###################################################
// Good YouTube video resource for Kalman Filter https://www.youtube.com/watch?v=mwn8xhgNpFY
float Q_angle = 0.001;    // Covariance of gyroscope noise    
float Q_gyro = 0.003;    // Covariance of gyroscope drift noise
float R_angle = 0.5;    // Covariance of accelerometer
char C_0 = 1;
float dt = 0.005; // The value of dt is the filter sampling time
float K1 = 0.05; // a function containing the Kalman gain is used to calculate the deviation of the optimal estimate
float K_0,K_1,t_0,t_1;
float angle_err;
float q_bias;    // Instrument Drift
float angleY_one;
float angle_speed;
float Pdot[4] = { 0, 0, 0, 0};
float P[2][2] = {{ 1, 0 }, { 0, 1 }};
float  PCt_0, PCt_1, E;
//########################### End of Kalman Filter Vars ##################################################



///////////////////////// ENCODER SETUP ////////////////////////////////////////////
ESP32Encoder encoder;

//Setup interrupt variables ----------------------------
volatile float count = 0; // encoder count
float cpr = 211.2; //encoder multiplied by gear ratio
volatile bool interruptCounter = false;    // check timer interrupt 1
volatile bool deltaT = false;     // check timer interrupt 2
int totalInterrupts = 0;   // counts the number of triggering of the alarm
hw_timer_t * timer0 = NULL;
hw_timer_t * timer1 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;

// encoder properties ------------------------------
int wheelspd = 0;

//Initialization ------------------------------------
void IRAM_ATTR onTime0() {
  portENTER_CRITICAL_ISR(&timerMux0);
  interruptCounter = true; // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux0);
}

void IRAM_ATTR onTime1() {
  portENTER_CRITICAL_ISR(&timerMux1);
  count = encoder.getCount( );
  encoder.clearCount ( );
  deltaT = true; // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux1);
}
////////////////////// ENCODER SETUP END ////////////////////////////////////////////



void setup() {
  Serial.begin(115200);
  while (!Serial) delay(1); // wait for Serial
  pinMode(LED_PIN, OUTPUT); // configures the specified pin to behave either as an input or an output
  digitalWrite(LED_PIN, LOW); // sets the initial state of LED as turned-off
  
  Serial.begin(115200);
  ESP32Encoder::useInternalWeakPullResistors = UP; // Enable the weak pull up resistors
  encoder.attachHalfQuad(33, 27); // Attache pins for use as encoder pins
  encoder.setCount(0);  // set starting count value after attaching

  // configure LED PWM functionalitites
  ledcSetup(ledChannel_1, freq, resolution);
  ledcSetup(ledChannel_2, freq, resolution);
  ledcSetup(ledChannel_PWM, freq, resolution);

  // attach the channel to the GPIO to be controlled 
  ledcAttachPin(BIN_1, ledChannel_1);
  ledcAttachPin(BIN_2, ledChannel_2);
  ledcAttachPin(PWM, ledChannel_PWM);

  // initialize timer
  timer0 = timerBegin(0, 80, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer0, &onTime0, true); // edge (not level) triggered 
  timerAlarmWrite(timer0, 2000000, true); // 2000000 * 1 us = 2 s, autoreload true

  timer1 = timerBegin(1, 80, true);  // timer 1, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer1, &onTime1, true); // edge (not level) triggered 
  timerAlarmWrite(timer1, 10000, true); // 10000 * 1 us = 10 ms, deltaT period

  // at least enable the timer alarms
  timerAlarmEnable(timer0); // enable
  timerAlarmEnable(timer1); // enable
  ledcWrite(ledChannel_PWM, LOW);
  pwm_sign = MAX_PWM_VOLTAGE;


  ////////////////////////////////Event//////////////////////////
  pinMode(BTN, INPUT);  
  attachInterrupt(BTN, isr, FALLING);  // set the "BTN" pin as the interrupt pin;
  event_timer = millis();
  lastcalc = millis();



////////////////// Communication //////////////////////////////////

  // Broadcast setup.
  // Set device as a Wi-Fi Station
//  WiFi.mode(WIFI_STA);
//  // Print MAC Accress on startup for easier connections
//  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
//  if (esp_now_init() != ESP_OK) {
//    Serial.println("Error initializing ESP-NOW");
//    return;
//  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  // esp_now_register_send_cb(OnDataSent);

  // Register peer
//  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
//  peerInfo.channel = 0;
//  peerInfo.encrypt = false;

  // Add peer
//  if (esp_now_add_peer(&peerInfo) != ESP_OK){
//    Serial.println("Failed to add peer");
//    return;
//  }
  // Register for a callback function that will be called when data is received
//  if (!WIFIDEBUG) {
//    esp_now_register_recv_cb(OnDataRecv);
//  }

  sendTime = millis();
  lasttime = millis();
  DAQState = IDLE;


  if (! mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("MLX90640 not found!");
  } else {
    Serial.println("Found Adafruit MLX90640");

    Serial.print("Serial number: ");
    Serial.print(mlx.serialNumber[0], HEX);
    Serial.print(mlx.serialNumber[1], HEX);
    Serial.println(mlx.serialNumber[2], HEX);
  }

  mlx.setMode(MLX90640_CHESS);
  mlx.setResolution(MLX90640_ADC_16BIT);
  mlx.setRefreshRate(MLX90640_16_HZ);
  Wire.setClock(1000000); // max 1 MHz

}




void loop() { // main code here, to run repeatedly:
  syncDAQState();
  Get_Readings();
  Calculate_Angle();
  if (deltaT) { //Speed Interrupt Action
    wheelspd = count/cpr*(100*60); //speed in counts per Timer1 interrupt
    deltaT = false;
//    Serial.println("motor_PWM, speed");
//    Serial.print(motor_PWM);
//    Serial.print(" ");
//    Serial.println(v);
  }
  logData();
  switch (DAQState) {
    case (IDLE):
      if (CommandState == CTDWN) { DAQState = CommandState; }
      digitalWrite(LED_PIN, LOW);
      ledcWrite(ledChannel_2, LOW);
      ledcWrite(ledChannel_1, LOW);
      if (buttonIsPressed == true && millis()-event_timer > del) {
        CommandState = CTDWN;
        event_timer = millis();
        buttonIsPressed = false;
      }
      break;
    case (CTDWN): 
      if (CommandState == SEEKMODE) { DAQState = CommandState; }
      digitalWrite(LED_PIN, HIGH);
      if ((millis() - event_timer) > ctdwn_time){
        CommandState = SEEKMODE;
      }
      break;
    case (SEEKMODE):
      if (CommandState == IDLE) { DAQState = CommandState; }
      // if (millis() - lasttime > 1/(ctrl_freq)) {
      Center();
      lasttime = millis();
      // }
      if (buttonIsPressed == true && millis()-event_timer > del) {
        CommandState = IDLE;
        event_timer = millis();
        buttonIsPressed = false;
      }
      break;
  }
}


void syncDAQState() {
    if (Serial.available() > 0) {
    // Serial.read reads a single character as ASCII. Number 1 is 49 in ASCII.
    // Serial sends character and new line character "\n", which is 10 in ASCII.
    int CommandState = Serial.read() - 48;}
}

void Get_Readings() {
  // take IR camera readingsn if newtime is over GenDelay-Lasttime
  mlx.getFrame(frame);

  // Compress to 1D array by averaging
  int i;
  int n;
  float line[WIDTH];
  for(int col = 0; col < WIDTH; col++){
    n = 0;
    for(int row = HEIGHT/2 - RADIUS; row <= HEIGHT/2 + RADIUS; row++){
      i = WIDTH * row + col;
      if(!isnan(frame[i])){
        line[col] += frame[i];
        n++;
      }
    }
    line[col] = line[col] / n;
  }

  // Blur to smooth data
  float line_filt[WIDTH];
  line_filt[0] = 0.25 * line[0] + 0.5 * line[0] + 0.25 * line[1];
  for(int i = 1; i < WIDTH-1; i++){
    line_filt[i] = 0.25 * line[i] + 0.5 * line[i+1] + 0.25 * line[i+2];
  }
  line_filt[WIDTH-1] = 0.25 * line[WIDTH-2] + 0.5 * line[WIDTH-1] + 0.25 * line[WIDTH-1];

  // Find max temp location
  //int iMax = 0;
  float max = 0;
  for(int i = 0; i < WIDTH; i++){
    if(line_filt[i] > max){
      max = line_filt[i];
      iMax = i;
    }
  }
  // Serial.print(iMax);
//  Serial.print("  ");
//  for(int i = 0; i < WIDTH; i++){
//    Serial.print(line_filt[i]);
//    Serial.print(",");
//  }
//  Serial.println("");
}

void Calculate_Angle() {
  //calculate angle from ideal. Use for state-space control
  //use angle memory to calculate angular velocity
  error = iMax - WIDTH/2;
  angle = (error*110)/32;
  deltat = (millis()-lastcalc)*1000;
  angle_speed = (lastangle-angle)*deltat;
  lastcalc = millis();
  lastangle = angle;
//  Kalman_Filter(angle, angle_speed)
}


void Center() {
  PID();         //angle loop PD control
  ReactionWheelPWM();
  
  // cc++;
  // if(cc>=6)     //every 6th control loop，enter PI algorithm
  // {
  //   SpeedPI();   
  //   cc=0;  //Clear
  // }
 
  loopcount++;
  if(loopcount == 1)  { // used to blink LED every 100 control loops.
    loopcount = 0;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));}
  }


////////////////// Angle PD_pwm ////////////////////
void PID(){
  positions += ((angle + targetAngle)*deltat);
  positions = constrain(positions, -355,355); 
  motor_PWM = motor_PWM + (kp * (angle + targetAngle) + ki * (positions) - kd * angle_speed); //PD angle loop control
}

void ReactionWheelPWM(){
  if (motor_PWM > 0) {
    if (motor_PWM > MAX_PWM_VOLTAGE) { 
      motor_PWM = MAX_PWM_VOLTAGE;
    }
    ledcWrite(ledChannel_2, LOW); // clockwise
    ledcWrite(ledChannel_1, motor_PWM); // power control while cw
  } else {
    int pos_motor_PWM = abs(motor_PWM);
    if (pos_motor_PWM > MAX_PWM_VOLTAGE) { 
      pos_motor_PWM = MAX_PWM_VOLTAGE;
      motor_PWM = -MAX_PWM_VOLTAGE;
    }
    ledcWrite(ledChannel_1, LOW); // ccw
    ledcWrite(ledChannel_2, pos_motor_PWM); // power control while ccw
  }
}

/////////////////////////////// Kalman Filter Calculations /////////////////////
void Kalman_Filter(double angle_m, double vel_m)
{
  angle += (vel_m - q_bias) * dt;          //prior estimate
  angle_err = angle_m - angle;
  
  Pdot[0] = Q_angle - P[0][1] - P[1][0];    //The differential of the covariance of the prior estimate error
  Pdot[1] = - P[1][1];
  Pdot[2] = - P[1][1];
  Pdot[3] = Q_gyro;
  
  P[0][0] += Pdot[0] * dt;    //The integral of the covariance differential of the prior estimate error
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
  
  //Intermediate variables in matrix multiplication 
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  //denominator
  E = R_angle + C_0 * PCt_0;
  //gain value
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  
  t_0 = PCt_0;  // Intermediate variables in matrix multiplication
  t_1 = C_0 * P[0][1];
  
  P[0][0] -= K_0 * t_0;    // Posterior estimation error covariance
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  
  q_bias += K_1 * angle_err;    // Posterior estimate
  angle_speed = vel_m - q_bias;   // The differential of the output value gives the optimal angular velocity
  angle += K_0 * angle_err; // Posterior estimation; get the optimal angle
}



//////////////// DATALOGGING ////////////////////////////////////////
void logData() {
  if (DEBUG) {printReadings();}
  if (millis()-sendTime > sendDelay) {
    sendTime = millis();
    addPacketToQueue();
    sendQueue();
  }
}

void printReadings() {
  Serial.print(iMax);
  Serial.print("   Err: ");
  Serial.print(error);
  Serial.print("   Ang: ");
  Serial.print(angle);
  Serial.print("   DAQ: ");
  Serial.print(DAQState);
  Serial.print("   CMD: ");
  Serial.println(CommandState);
}

void addPacketToQueue() {
//  if (queueLength < 40) {
//    queueLength += 1;
//    PacketQueue[queueLength].messageTime = millis();
//    PacketQueue[queueLength].angle = angle;
//    PacketQueue[queueLength].queueLength = queueLength;
//    PacketQueue[queueLength].DAQState    = DAQState;
//  }
}

void sendQueue() { //need to add supporting structs
//  if (queueLength < 0) {
//    return;
//  }
  // Set values to send
//  Packet = PacketQueue[queueLength];
//  if (!WIFIDEBUG) {
//    // Send message via ESP-NOW
//    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Packet, sizeof(Packet));
//    if (result == ESP_OK) {
//      // Serial.println("Sent with success Data Send");
//      queueLength -= 1;
//    } else {
//      Serial.println("Error sending the data");
//    }
//  }
}
