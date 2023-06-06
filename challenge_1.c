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
int blackThreashold[4] = {900, 900, 900, 900}; // the threshold of black line

float duty = 0.09;                         // duty cycle
float left_adjust = 1.20;                  // adjust the left motor
float right_adjust = 1.22;                 // adjust the right motor

/*************** Initialization Control ***************/

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
    DDRL |= 0x01; // set a green LED output
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
 * @brief Turn on the green LED
 * @evaluation Works well
 */
void greenLEDon()
{
    PORTL &= ~(1 << 0);
}

/**
 * @brief Turn off the green LED
 * @evaluation Works well
 */
void greenLEDoff()
{
    PORTL |= (1 << 0);
}

/*************** GroundSensor Control ***************/

/**
 * @brief Read the value of ground IR sensor and store it in `line`
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

/*************** Motor Control ***************/

/**
 * @brief Move the left motor forward
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
 * @brief Move the right motor forward
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
 * @brief Move the left motor backward
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
 * @brief Move the right motor backward
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
 * @brief Stop the left motor
 * @evaluation Works well
 */
void leftMotorStop()
{
    analogWrite(pwm_left, 255);
    digitalWrite(dir_left, HIGH);
}

/**
 * @brief Stop the right motor
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
 * @brief Stop the car
 * @evaluation Works well
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

    if (line[1] > blackThreashold[1]) // black line on left
    {
        greenLEDon();   // turn on the green LED
        leftTurn(duty); // turn left
    }
    else
    {
        greenLEDoff(); // turn off the green LED
    }
    if (line[2] > blackThreashold[2]) // black line on right
    {
        greenLEDon();    // turn on the green LED
        rightTurn(duty); // turn right
    }
    else
    {
        greenLEDoff(); // turn off the green LED
    }
}
