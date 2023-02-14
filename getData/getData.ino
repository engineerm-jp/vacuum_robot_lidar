#include <SoftwareSerial.h>
#include <string.h>

// serial communication packet specification
/* 
PACKET STRUCTURE


    <HEAD> <ANGLE> <SPEED_LSB> <SPEED_MSB> <16 BYTES FOR DATA> <CHECKSUM_LSB> <CHECKSUM_MSB>

    Of which inside the 16 bytes of data, there are four sets of the following bytes: 

    <DISTANCE_LSB><DISTANCE_MSB><SIGNAL_STR_LSB><SIGNAL_STR_MSB>

*/

// index of the bytes
#define ANGLE_IDX 1
#define SPEED_LSB 2
#define SPEED_MSB 3
#define DATA_1 4
#define DATA_2 8
#define DATA_3 12
#define DATA_4 16 
#define CHECKSUM_LSB 20
#define CHECKSUM_MSB 21

#define PACKET_SIZE 22  // size of packet
#define DATA_SIZE 7 // angle, speed, distance x 4, irradiance, validity

#define RX_PIN 10       // rx pin from the sensor
#define BAUDRATE_SENSOR 115200  // baudrate of the sensor
#define BAUDRATE 115200
#define MIN_POWER 0     // minimum power of the motor
#define MAX_POWER 255   // maximum power of the motor
#define MOTOR_PIN 5     // motor pin number on Arduino Uno
#define MOTOR_SPEED 250 // motor speed in RPM


int data[DATA_SIZE]; // [angle, speed, distance 1, distance 2, distance 3, distance 4, checksum]
uint8_t packet[PACKET_SIZE];    // packet buffer
const unsigned char HEAD_BYTE = 0xFA;   // start byte of the packet
unsigned int packetIndex      = 0;      // packet index
uint8_t receivedByte    = 0;    // received byte
uint8_t lowStrengthFlag = 0; 
bool PACKET_OK  = true;         // if the packet is valid
bool waitPacket = true;         // true if waiting for a packet


// PID control variables
double proportionalTerm = 0;
double derivativeTerm   = 0; 
double integralTerm     = 0;
double previousSpeed    = 0;

int currentSpeed    = 0; 
int baseSpeed       = 0; 
double t_sample     = 64*65536/16000000.0; // pre-scaler of 64
double kp = 2;
double ki = 0.3; 
double kd = 0.3;
int controlEffort = 0; 

// sensor ouput data
int angle = 0; 
int speed = 0; 

// uint16_t dist_1, dist_2, dist_3, dist_4 = 0, 0, 0, 0; 
SoftwareSerial lidarSensor =  SoftwareSerial(RX_PIN, 3);
void setup() {
    // setup TIMER0 -> trigger timer ISR every approx. 0.2s
    noInterrupts();     
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1B |= (1 << CS11)|(1 << CS10);   
    TIMSK1 |= (1 << TOIE1); 
    interrupts();
    
    // setup serial LiDAR -> Arduino
    lidarSensor.begin(BAUDRATE_SENSOR);

    // setup serial Arduino -> PC
    pinMode(RX_PIN, INPUT);
    Serial.begin(BAUDRATE);

    // setup motor
    analogWrite(MOTOR_PIN,MAX_POWER);   // kick start motor with MAX power
    delay(2000);
    analogWrite(MOTOR_PIN,125);
    // initialize packet buffer
    for (int idx = 0; idx < PACKET_SIZE; idx++) packet[idx] = 0;    // initialize packet buffer
}


void loop() {

    // check if any packet if arrived
    if (lidarSensor.available() > 0) {

        receivedByte = lidarSensor.read();

        if (waitPacket) { // wait for a new packet to arrive
            if (receivedByte == HEAD_BYTE) {
                packetIndex = 0;    // initialise packet index
                waitPacket = false;
                packet[packetIndex++] = receivedByte;
            }

        } else {  // if currently receiving packet

            if (packet[0] == HEAD_BYTE) { // ensure the head of the packet is valid
                
                packet[packetIndex++] = receivedByte; // store received byte
                
                if (packetIndex >= PACKET_SIZE) { // if packet buffer is full
                    waitPacket = true; // wait for a new packet
                    decodePacket(packet, PACKET_SIZE); // process the packet
                    sendData(data, DATA_SIZE);       
                }
            }
        }
    }
}

// timer interrupt handler for motor speed control
ISR(TIMER1_OVF_vect)
{
    motorSpeedPID(MOTOR_SPEED, currentSpeed, 0.262, kp, ki, kd);
}


void decodePacket(uint8_t packet[], int packetSize) {
    int data_idx = 0; 

    for (int idx = 0; idx < DATA_SIZE; idx++) data[idx] = 0;  // initialise data array

    for (int i = 0; i < packetSize; i++){
      // Serial.print("0x"); Serial.print(packet[i]); Serial.print('\t');
    
        if (i == 0) {   // header byte
          // Serial.print("data: ");
          continue;
        }
        else if (i == 1) {
            uint16_t angle = (packet[i] - 0xA0) * 4;  // convert to values between 0 ~ 360
            if (angle > 360) return; 
            // Serial.print(angle); 
            data[data_idx++] = angle;
            // Serial.print('\t');
        }
        else if (i == 2) {
            int speed = 0x00; 
            speed |= ( (packet[3]<<8) | packet[2]);     

            // an attempt to smoothen the speed readings since they sometimes spikes due to unknown issue
            currentSpeed = abs(speed/64-currentSpeed) > 100 ? currentSpeed*0.95 + (speed/64)*0.05: speed/64; 
            // Serial.print(currentSpeed);
            data[data_idx++] = currentSpeed;
            // Serial.print('\t');
        }
        else if (i == 4 || i == 8 || i == 12 || i == 16) {
            uint16_t distance = 0x00;
            distance |= ((packet[i+1]&0x3F) << 8) | packet[i]; 
            // Serial.print(distance);
            data[data_idx++] = distance;

            // Serial.print('\t');
            // Serial.print('\t'); Serial.print(packet[i+3]>>8 + packet[i+2]);
            // if (packet[i+1]&& 1<<7) {
            //   Serial.print("inv: ");
            //   if (packet[i+1] && 1 << 6) {
            //     Serial.print("str: ");
            //   }
            // }     
            // else {
            //   Serial.print(distance/10.0);
            // }
        }
      
    }

    uint16_t chksum = checksum(packet, (unsigned int)(packet[PACKET_SIZE-2] + (packet[PACKET_SIZE-1]<<8)),PACKET_SIZE-2);
    data[data_idx++] = chksum; 
    
}

int sendData(int data[], int dataSize) { // send a packet of size packetSize
  for (int i = 0; i < dataSize; i++) {
    Serial.print(data[i]); 
    Serial.print('\t'); 
  }
  Serial.println();  
}

uint16_t checksum(uint8_t packet[], uint16_t sum, uint8_t size)
{
    uint32_t chk32 = 0;
    uint16_t data[size/2]; 
    uint8_t  sensorData[size]; 

    for (int i = 0; i < size; i++) sensorData[i] = packet[i];

    for (int i = 0; i < size/2; i++) {
        data[i] = ((sensorData[i*2+1] << 8) + sensorData[i*2]);
        chk32 = (chk32 << 1) + data[i];
    }

    uint32_t checksum=(chk32 & 0x7FFF) + (chk32 >> 15);
    return  (uint16_t) (checksum & 0x7FFF) == sum;
}

unsigned int saturatePower(unsigned int power, unsigned int minPower, unsigned int maxPower) {
    if (power > maxPower) return power = maxPower;
    else if (power < minPower) return power = minPower;
    else return power;
}

void moveMotor(unsigned int motorPin, unsigned int power) {
    power = saturatePower(power, MIN_POWER, MAX_POWER);  // saturate power (0 ~ 255)
    analogWrite(motorPin, power);   // send PWM signal to the motor controller
}


// a simple PID controller for keeping the motor speed to a specified RPM value
int motorSpeedPID(int targetSpeed, int currentSpeed, double deltaT, double kp, double ki, double kd) {

    proportionalTerm = targetSpeed - currentSpeed;  // current error to the target speed
    derivativeTerm = (currentSpeed - previousSpeed) * 1/deltaT;  // derivative error to the target speed (sample rate = 1 Hz)
    integralTerm += proportionalTerm * deltaT;               // integral term (sample rate = 1 Hz)
    controlEffort = (kp * proportionalTerm + kd * derivativeTerm + ki * integralTerm) + currentSpeed;

    if (controlEffort > MAX_POWER) {        // simple anti-windup
        integralTerm -= proportionalTerm * deltaT;   // stop accumulating integral term if power is saturated
        controlEffort = (kp * proportionalTerm + kd * derivativeTerm + ki * integralTerm) + currentSpeed;  // recalculate control effort
    }

    previousSpeed = currentSpeed;         // record previous speed
    moveMotor(MOTOR_PIN, controlEffort);  // drive motor
}
