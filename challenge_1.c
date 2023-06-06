/**
Task: Challenge 1
Machine: Based on ATmega 2560
Machine Number: 4107
Author: Yilin Jing
Candidate Number: 238628
IDE: Arduino IDE
*/

unsigned char pwm_right = 5;               // pin number to control the right motor
unsigned char dir_right = 2;               // pin number to control the right motor
unsigned char pwm_left = 6;                // pin number to control the left motor
unsigned char dir_left = 7;                // pin number to control the left motor

int groundIRSensor[4] = {8, 9, 10, 11};    // 4 ground IR proximity sensor
int line[4] = {};                          // store the value of ground IR sensor
int blackThread[4] = {900, 900, 900, 900}; // the threshold of black line
float duty = 0.09;                         // duty cycle
float left_adjust = 1.20;                  // adjust the left motor
float right_adjust = 1.22;                 // adjust the right motor

/*************** Initialization Control ***************/

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
    DDRL |= 0x01; // set a green LED output
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
 * @brief Turn on the green LED
 */
void greenLEDon()
{
    PORTL &= ~(1 << 0);
}

/**
 * @brief Turn off the green LED
 */
void greenLEDoff()
{
    PORTL |= (1 << 0);
}

/*************** GroundSensor Control ***************/

/**
 * @brief Read the value of ground IR sensor and store it in `line`
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

/*************** Motor Control ***************/

/**
 * @brief Move the left motor forward
 * @param duty
 */
void leftMotorForward(float duty)
{
    unsigned char pwm = (unsigned char)(duty * 255 * left_adjust);
    analogWrite(pwm_left, pwm);
    digitalWrite(dir_left, LOW);
}

/**
 * @brief Move the right motor forward
 * @param duty
 */
void rightMotorForward(float duty)
{
    unsigned char pwm = (unsigned char)(duty * 255 * right_adjust);
    analogWrite(pwm_right, pwm);
    digitalWrite(dir_right, LOW);
}

/**
 * @brief Move the left motor backward
 * @param duty
 */
void leftMotorBackward(float duty)
{
    unsigned char pwm = (unsigned char)((1 - duty) * 255);
    analogWrite(pwm_left, pwm);
    digitalWrite(dir_left, HIGH);
}

/**
 * @brief Move the right motor backward
 * @param duty
 */
void rightMotorBackward(float duty)
{
    unsigned char pwm = (unsigned char)((1 - duty) * 255);
    analogWrite(pwm_right, pwm);
    digitalWrite(dir_right, HIGH);
}

/**
 * @brief Stop the left motor
 */
void leftMotorStop()
{
    analogWrite(pwm_left, 255);
    digitalWrite(dir_left, HIGH);
}

/**
 * @brief Stop the right motor
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
 * @brief Move forward
 * @param duty
 */
void moveForward(float duty)
{
    leftMotorForward(duty);
    rightMotorForward(duty);
}

/**
 * @brief Stop the car
 */
void stop()
{
    leftMotorStop();
    rightMotorStop();
}

/*************** Setup Control ***************/

/**
 * @brief Initialize the registers and serial port
 */
void setup()
{
    greenLEDInit();
    groundIRInit();
    motorInit();
    Serial.begin(9600);
}

/*************** main loop Control ***************/

void loop()
{
    readGroundIRSensors(); // read the value of ground IR sensor
    moveForward(duty);     // move forward

    if (line[1] > blackThread[1]) // black line on left
    {
        greenLEDon();   // turn on the green LED
        leftTurn(duty); // turn left
    }
    else
    {
        greenLEDoff(); // turn off the green LED
    }
    if (line[2] > blackThread[2]) // black line on right
    {
        greenLEDon();    // turn on the green LED
        rightTurn(duty); // turn right
    }
    else
    {
        greenLEDoff(); // turn off the green LED
    }
}
