/**
Task: Challenge 4
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
int blackThreashold[4] = {850, 900, 900, 850};                // the threshold of black line

float duty = 0.095;                                       // duty cycle to control the speed of motor by pwm
float left_adjust = 1.20;                                 // adjust the left motor
float right_adjust = 1.22;                                // adjust the right motor

int count_corner = 0;                                     // count the number of corner
int state = 0;                                            // state of the robot
int count_loop = 0;                                       // count the number of loop

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
 * @brief Turn left in the same place
 * @param duty
 */
void leftTurnInPlace(float duty)
{
    leftMotorBackward(duty);
    rightMotorForward(duty);
}

/**
 * @brief Turn right in the same place
 * @param duty
 */
void rightTurnInPlace(float duty)
{
    rightMotorBackward(duty);
    leftMotorForward(duty);
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

/*************** Check Control ***************/

/**
 * @brief Check if the robot is at the corner
 * @return true if the robot is at the corner
 */
bool isCorner()
{
    if (line[0] > blackThreashold[0] & line[3] > blackThreashold[3])
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief Check if the robot is at the destination
 * @return true if the robot is at the destination
 */
bool isDestination()
{
    if (line[0] > blackThreashold[0] & line[1] > blackThreashold[1] & line[2] > blackThreashold[2] & line[3] > blackThreashold[3])
    {
        return true;
    }
    else
    {
        return false;
    }
}

/*************** State Control ***************/

/**
 * @brief Move forward based on the black line
 * @param duty
 */
void moveForwardSteadyByBlackLine(float duty)
{
    moveForward(duty);
    if (line[1] > blackThreashold[1]) // black line on left
    {
        leftTurn(duty);
    }
    if (line[2] > blackThreashold[2]) // black line on right
    {
        rightTurn(duty);
    }
}

/**
 * @brief Turn left at the corner and move forward based on the black line
 * @param duty
 */
void turnLeft90DegreeAtCorner(float duty)
{
    leftTurnInPlace(duty);
    delay(350);
    moveForward(duty);
    if (line[1] > blackThreashold[1]) // black line on left
    {
        leftTurn(duty);
    }
    if (line[2] > blackThreashold[2]) // black line on right
    {
        rightTurn(duty);
    }
    delay(50);
}

/**
 * @brief Turn right at the corner and move forward based on the black line
 * @param duty
 */
void turnRight90DegreeAtCorner(float duty)
{
    rightTurnInPlace(duty);
    delay(90);
    moveForward(duty);
    if (line[1] > blackThreashold[1]) // black line on left
    {
        leftTurn(duty);
    }
    if (line[2] > blackThreashold[2]) // black line on right
    {
        rightTurn(duty);
    }
    delay(30);
}

/**
 * @brief Move forward based on the proximity
 * @param duty
 */
void moveForwardSteadyByProximity(float duty)
{
    moveForward(duty);
    if (proximity[1] < distanceThreashold_15mm | proximity[2] < distanceThreashold_15mm) // the robot is too close to the wall on the right
    {
        leftTurn(duty);
    }
    if (proximity[7] < distanceThreashold_15mm | proximity[6] < distanceThreashold_15mm) // the robot is too close to the wall on the left
    {
        rightTurn(duty);
    }
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
    count_loop++;             // count the number of loop, used for delay
    readGroundIRSensors();    // read the ground IR sensors
    readProximityIRSensors(); // read the proximity IR sensors

    /**
     * switch th state of the robot
     */
    switch (state)
    {
    case 0: // Move Forward base on BlackLine
        moveForwardSteadyByBlackLine(duty);
        break;
    case 1: // Turn Left base on BlackLine
        turnLeft90DegreeAtCorner(duty*1.6);
        state = 0;
        break;
    case 2: // Turn Right base on BlackLine
        turnRight90DegreeAtCorner(duty*1.6);
        state = 0;
        break;
    case 3: // Move Forward base on avoidance
        moveForwardSteadyByProximity(duty);
        break;
    case 4: // Stop
        stop();
        delay(100000);
        break;
    default:
        break;
    }

    /**
     * check the state of the robot
     */
    if (isCorner() & count_corner == 0 & count_loop > 20) // meet the 1st corner
    {
        greenLEDon(1);
        state = 1; // turn left
        count_corner++;
        count_loop = 0;
    }
    else if (isCorner() & count_corner == 1 & count_loop > 20) // meet the 2nd corner
    {
        greenLEDon(2);
        state = 2; // turn right
        count_corner++;
        count_loop = 0;
    }
    else if (isCorner() & count_corner == 2 & count_loop > 20) // meet the 3rd corner
    {
        greenLEDon(3);
        state = 1; // turn left
        count_corner++;
        count_loop = 0;
    }
    else if (isCorner() & count_corner == 3 & count_loop > 20) // meet the 4th corner
    {
        greenLEDon(4);
        state = 2; // turn right
        count_corner++;
        count_loop = 0;
    }
    else if (isDestination() & count_corner == 4 & count_loop > 20) // meet the destination
    {
        greenLEDon(5);
        state = 4; // stop
    }

    if (count_corner >= 4 & count_loop == 20) // After the 4th corner, the robot will move forward based on avoidance
    {
        state = 3; // To move forward based on avoidance
    }
}