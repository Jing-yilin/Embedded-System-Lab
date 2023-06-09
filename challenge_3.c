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
int distanceThreashold_15mm = 800;                        // the threshold of 15mm

int groundIRSensor[4] = {8, 9, 10, 11};                   // 4 ground IR proximity sensor
int line[4] = {};                                         // store the value of ground IR sensor
int blackThreashold[4] = {900, 900, 900, 900};                // the threshold of black line

float duty = 0.09;                                        // duty cycle to control the speed of motor by pwm
float left_adjust = 1.20;                                 // adjust the left motor
float right_adjust = 1.22;                                // adjust the right motor

/*************** Register Initialization Control ***************/

/**
 * @brief To initialize the register for ground IR sensor and ground IR LED
 * @evaluation Works well
 */
void groundIRInit()
{
    DDRK = 0xf0; // set input for ground IR sensor
    DDRJ = 0x0f; // set output for ground IR LED
}
/**
 * @brief To initialize the register for left and right motor
 * @evaluation Works well
 */
void motorInit()
{
    DDRE |= 0B00011000; // set left DC motor output
    DDRH |= 0B00011000; // set right DC motor output
}
/**
 * @brief To initialize the register for green LED
 * @evaluation Works well
 */
void greenLEDInit()
{
    DDRL |= ~(1 << 3); // set green LED output
    DDRG |= (1 << 3);  // set green LED output
}

/**
 * @brief To initialize the register for proximity IR sensor and proximity IR LED
 * @evaluation Works well
 */
void proximityIRInit()
{
    DDRF &= 0x00; // set 8 IR proximity sensor input
    DDRA |= 0xff; // set 8 IR LED output
}

/*************** Ground IR LED Control ***************/

/**
 * @brief Turn on the ground LED numbered `lineIndex`
 * @evaluation Works well
 * @param lineIndex
 */
void groundLEDon(unsigned char lineIndex)
{
    PORTJ &= ~(1 << lineIndex);
}
/**
 * @brief Turn off the ground LED numbered `lineIndex`
 * @evaluation Works well
 * @param lineIndex
 */
void groundLEDoff(unsigned char lineIndex)
{
    PORTJ |= (1 << lineIndex);
}

/*************** Green LED Control ***************/

/**
 * @brief Turn on the green LED numbered `ledIndex`
 * @evaluation Works well
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
 * @evaluation Works well
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
 * @evaluation Works well
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
 * @evaluation Works well
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
 * @evaluation Works well
 * @param proxIndex
 */
void proxLEDon(unsigned char proxIndex)
{
    PORTA |= (1 << proxIndex);
}

/**
 * @brief Turn off the IR LED numbered `proxIndex`
 * @evaluation Works well
 * @param proxIndex
 */
void proxLEDoff(unsigned char proxIndex)
{
    PORTA &= ~(1 << proxIndex);
}

/*************** IR Proximity Sensor Control ***************/

/**
 * @brief Read the value of IR proximity sensor
 * @evaluation Works well
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
 * @evaluation Works well
 */
double getRealProximity(int proximityValue)
{
    double realProximity = 0.02161 * proximityValue + -0.6154;
    return realProximity;
}

/*************** Motor Control ***************/

/**
 * @brief Control the left motor to move forward
 * @evaluation Works well
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
 * @evaluation Works well
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
 * @evaluation Works well
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
 * @evaluation Works well
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
 * @evaluation Works well
 */
void leftMotorStop()
{
    analogWrite(pwm_left, 255);
    digitalWrite(dir_left, HIGH);
}

/**
 * @brief Control the right motor to stop
 * @evaluation Works well
 */
void rightMotorStop()
{
    analogWrite(pwm_right, 255);
    digitalWrite(dir_right, HIGH);
}

/**
 * @brief Turn left
 * @evaluation Works well
 * @param duty
 */
void leftTurn(float duty)
{
    leftMotorForward(duty / 2.5);
    rightMotorForward(duty);
}

/**
 * @brief Turn right
 * @evaluation Works well
 * @param duty
 */
void rightTurn(float duty)
{
    rightMotorForward(duty / 2.5);
    leftMotorForward(duty);
}

/**
 * @brief Turn left with big angle
 * @evaluation Works well
 * @param duty
 */
void leftTurnBigAngle(float duty)
{
    leftMotorForward(duty / 2);
    rightMotorForward(duty * 1.4);
}

/**
 * @brief Turn right with big angle
 * @evaluation Works well
 * @param duty
 */
void rightTurnBigAngle(float duty)
{
    rightMotorForward(duty / 2);
    leftMotorForward(duty * 1.4);
}

/**
 * @brief Turn left in the same place
 * @evaluation Works well
 * @param duty
 */
void leftTurnInPlace(float duty)
{
    leftMotorBackward(duty);
    rightMotorForward(duty);
}

/**
 * @brief Turn right in the same place
 * @evaluation Works well
 * @param duty
 */
void rightTurnInPlace(float duty)
{
    rightMotorBackward(duty);
    leftMotorForward(duty);
}

/**
 * @brief Move forward
 * @evaluation Works well
 * @param duty
 */
void moveForward(float duty)
{
    leftMotorForward(duty);
    rightMotorForward(duty);
}

/**
 * @brief Move backward
 * @evaluation Works well
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

    float random_angle = random(15, 25) / 10.0; // random_angle is between 1.5 and 2.5
    readGroundIRSensors();
    readProximityIRSensors();
    moveForward(duty);

    /**
     * 1. Avoidance algrithm
     */
    if (proximity[0] < distanceThreashold_15mm) // IR sensor 0 is very close to the wall
    {
        leftTurnInPlace(duty * 1.6);
        delay(30);
        moveForward(duty);
        delay(5);
    }
    if (proximity[1] < distanceThreashold_15mm) // IR sensor 1 is very close to the wall
    {
        leftTurn(duty);
        delay(5);
    }
    if (proximity[7] < distanceThreashold_15mm + 150) // IR sensor 7 is very close to the wall
    {
        rightTurn(duty);
        delay(5);
    }
    if (proximity[2] < distanceThreashold_15mm) // IR sensor 2 is very close to the wall
    {
        leftTurn(duty);
        delay(5);
    }
    if (proximity[6] < distanceThreashold_15mm) // IR sensor 6 is very close to the wall
    {
        rightTurn(duty);
        delay(5);
    }

    /**
     * 2. Border detection algrithm
     */
    if (line[1] > blackThreashold[1]) // black line on left
    {
        greenLEDon(1);
        rightTurnInPlace(duty * random_angle); // turn right in random angle
        delay(20);
        moveForward(duty);
        delay(5);
    }
    else
    {
        greenLEDoff(1);
    }

    if (line[2] > blackThreashold[2]) // black line on right
    {
        greenLEDon(1);
        leftTurnInPlace(duty * random_angle); // turn left in random angle
        delay(20);
        moveForward(duty);
        delay(5);
    }
    else
    {
        greenLEDoff(1);
    }

    if (line[1] > blackThreashold[1] & line[2] > blackThreashold[2]) // The car has reached the corder
    {
        greenLEDon(1);
        rightTurnInPlace(duty * random_angle); // turn right in random angle
        delay(40);
        moveForward(duty);
        delay(5);
    }
    else
    {
        greenLEDoff(1);
    }
}