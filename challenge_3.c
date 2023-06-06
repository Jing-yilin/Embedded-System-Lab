/**
Task: Challenge 3
Machine: Based on ATmega 2560
Machine Number: 4107
Author: Yilin Jing
Candidate Number: 238628
IDE: Arduino IDE
*/

unsigned char pwm_right = 5;                              // pin number to control the right motor
unsigned char dir_right = 2;                              // pin number to control the right motor
unsigned char pwm_left = 6;                               // pin number to control the left motor
unsigned char dir_left = 7;                               // pin number to control the left motor

int proximityIRSensor[8] = {0, 1, 2, 3, 4, 5, 6, 7};      // 8 IR proximity sensor
int proximityIRLed[8] = {22, 23, 24, 25, 26, 27, 28, 29}; // 8 IR LED
int proximity[8] = {};                                    // store the value of IR sensor
int distanceThread_15mm = 800;                            // the threshold of 15mm

int groundIRSensor[4] = {8, 9, 10, 11};                   // 4 ground IR proximity sensor
int line[4] = {};                                         // store the value of ground IR sensor
int blackThread[4] = {900, 900, 900, 900};                // the threshold of black line

float duty = 0.09;                                        // duty cycle to control the speed of motor by pwm
float left_adjust = 1.20;                                 // adjust the left motor
float right_adjust = 1.22;                                // adjust the right motor

/*************** Register Initialization Control ***************/

/**
 * @brief To initialize the register for ground IR sensor and ground IR LED
 */
void groundIRInit()
{
    DDRK = 0xf0; // set input for ground IR sensor
    DDRJ = 0x0f; // set output for ground IR LED
}
/**
 * @brief To initialize the register for left and right motor
 */
void motorInit()
{
    DDRE |= 0B00011000; // set left DC motor output
    DDRH |= 0B00011000; // set right DC motor output
}
/**
 * @brief To initialize the register for green LED
 */
void greenLEDInit()
{
    DDRL |= ~(1 << 3); // set green LED output
    DDRG |= (1 << 3);  // set green LED output
}

/**
 * @brief To initialize the register for proximity IR sensor and proximity IR LED
 */
void proximityIRInit()
{
    DDRF &= 0x00; // set 8 IR proximity sensor input
    DDRA |= 0xff; // set 8 IR LED output
}

/*************** Ground IR LED Control ***************/

/**
 * @brief Turn on the ground LED numbered `lineIndex`
 * @param lineIndex
 */
void groundLEDon(unsigned char lineIndex)
{
    PORTJ &= ~(1 << lineIndex);
}
/**
 * @brief Turn off the ground LED numbered `lineIndex`
 * @param lineIndex
 */
void groundLEDoff(unsigned char lineIndex)
{
    PORTJ |= (1 << lineIndex);
}

/*************** Green LED Control ***************/

/**
 * @brief Turn on the green LED numbered `ledIndex`
 * @param ledIndex
 */
void greenLEDon(unsigned char ledIndex)
{
    if (ledIndex == 3)
    {
        PORTG &= ~(1 << 3);
    }
    else
    {
        PORTL &= ~(1 << ledIndex);
    }
}

/**
 * @brief Turn off the green LED numbered `ledIndex`
 * @param ledIndex
 */
void greenLEDoff(unsigned char ledIndex)
{
    if (ledIndex == 3)
    {
        PORTG |= (1 << 3);
    }
    else
    {
        PORTL |= (1 << ledIndex);
    }
}

/*************** GroundSensor Control ***************/

/**
 * @brief Read the value of ground IR sensor
 */
void readGroundIRSensors()
{
    for (int i = 0; i < 4; i++)
    {
        groundLEDon(i);
        delay(10);
        line[i] = analogRead(groundIRSensor[i]);
        groundLEDoff(i);
    }
}

/**
 * @brief Show the value of ground IR sensor
 */
void showGroundIRSensorLine()
{
    for (int i = 0; i < 4; i++)
    {
        Serial.print(line[i], DEC);
        Serial.print(" ");
    }
    Serial.println();
}

/*************** IR Proximity LED Control ***************/

/**
 * @brief Turn on the IR LED numbered `proxIndex`
 * @param proxIndex
 */
void proxLEDon(unsigned char proxIndex)
{
    PORTA |= (1 << proxIndex);
}

/**
 * @brief Turn off the IR LED numbered `proxIndex`
 * @param proxIndex
 */
void proxLEDoff(unsigned char proxIndex)
{
    PORTA &= ~(1 << proxIndex);
}

/*************** IR Proximity Sensor Control ***************/

/**
 * @brief Read the value of IR proximity sensor
 */
void readProximityIRSensors()
{
    for (int i = 0; i < 8; i++)
    {
        proxLEDon(i);
        proximity[i] = analogRead(proximityIRSensor[i]);
        proxLEDoff(i);
    }
}

/**
 * @brief Calculate the practical value of IR proximity sensor
 */
double getRealProximity(int proximityValue)
{
    double realProximity = 0.02161 * proximityValue + -0.6154;
    return realProximity;
}

/*************** Motor Control ***************/

/**
 * @brief Control the left motor to move forward
 * @param duty
 */
void leftMotorForward(float duty)
{
    unsigned char pwm = (unsigned char)(duty * 255 * left_adjust);
    analogWrite(pwm_left, pwm);
    digitalWrite(dir_left, LOW);
}

/**
 * @brief Control the right motor to move forward
 * @param duty
 */
void rightMotorForward(float duty)
{
    unsigned char pwm = (unsigned char)(duty * 255 * right_adjust);
    analogWrite(pwm_right, pwm);
    digitalWrite(dir_right, LOW);
}

/**
 * @brief Control the left motor to move backward
 * @param duty
 */
void leftMotorBackward(float duty)
{
    unsigned char pwm = (unsigned char)((1 - duty) * 255);
    analogWrite(pwm_left, pwm);
    digitalWrite(dir_left, HIGH);
}

/**
 * @brief Control the right motor to move backward
 * @param duty
 */
void rightMotorBackward(float duty)
{
    unsigned char pwm = (unsigned char)((1 - duty) * 255);
    analogWrite(pwm_right, pwm);
    digitalWrite(dir_right, HIGH);
}

/**
 * @brief Control the left motor to stop
 */
void leftMotorStop()
{
    analogWrite(pwm_left, 255);
    digitalWrite(dir_left, HIGH);
}

/**
 * @brief Control the right motor to stop
 */
void rightMotorStop()
{
    analogWrite(pwm_right, 255);
    digitalWrite(dir_right, HIGH);
}

/**
 * @brief Turn left
 * @param duty
 */
void leftTurn(float duty)
{
    leftMotorForward(duty / 2.5);
    rightMotorForward(duty);
}

/**
 * @brief Turn right
 * @param duty
 */
void rightTurn(float duty)
{
    rightMotorForward(duty / 2.5);
    leftMotorForward(duty);
}

/**
 * @brief Turn left with big angle
 * @param duty
 */
void leftTurnBigAngle(float duty)
{
    leftMotorForward(duty / 2);
    rightMotorForward(duty * 1.4);
}

/**
 * @brief Turn right with big angle
 * @param duty
 */
void rightTurnBigAngle(float duty)
{
    rightMotorForward(duty / 2);
    leftMotorForward(duty * 1.4);
}

/**
 * @brief Turn left with 90 degree
 * @param duty
 */
void leftTurn90Angle(float duty)
{
    leftMotorForward(0);
    rightMotorForward(duty * 1.4);
}

/**
 * @brief Turn right with 90 degree
 * @param duty
 */
void rightTurn90Angle(float duty)
{
    rightMotorForward(0);
    leftMotorForward(duty * 1.4);
}

/**
 * @brief Move forward
 * @param duty
 */
void moveForward(float duty)
{
    leftMotorForward(duty);
    rightMotorForward(duty);
}

/**
 * @brief Move backward
 * @param duty
 */
void stop()
{
    leftMotorStop();
    rightMotorStop();
}

/*************** Setup Control ***************/

/**
 * @brief To initialize all the registers and serial port
 */
void setup()
{
    proximityIRInit();
    greenLEDInit();
    groundIRInit();
    motorInit();
    Serial.begin(9600);
}

/*************** main loop Control ***************/

void loop()
{
    readGroundIRSensors();
    readProximityIRSensors();
    moveForward(duty);

    if (proximity[0] < distanceThread_15mm) // IR sensor 0 is very close to the wall
    {
        leftTurn90Angle(duty);
        delay(30);
        moveForward(duty);
        delay(5);
    }
    if (proximity[1] < distanceThread_15mm) // IR sensor 1 is very close to the wall
    {
        leftTurn(duty);
        delay(5);
    }
    if (proximity[7] < distanceThread_15mm + 150) // IR sensor 7 is very close to the wall
    {
        rightTurn(duty);
        delay(5);
    }
    if (proximity[2] < distanceThread_15mm) // IR sensor 2 is very close to the wall
    {
        leftTurn(duty);
        delay(5);
    }
    if (proximity[6] < distanceThread_15mm) // IR sensor 6 is very close to the wall
    {
        rightTurn(duty);
        delay(5);
    }

    if (line[1] > blackThread[1]) // black line on left
    {
        greenLEDon(1);
        rightTurn90Angle(duty); // turn right 90 degree
        delay(20);
        moveForward(duty);
        delay(5);
    }
    else
    {
        greenLEDoff(1);
    }

    if (line[1] > blackThread[1] & line[2] > blackThread[2]) // The car has reached the corder
    {
        greenLEDon(1);
        rightTurn90Angle(duty); // turn right 90 degree
        delay(40);
        moveForward(duty);
        delay(5);
    }
    else
    {
        greenLEDoff(1);
    }
}