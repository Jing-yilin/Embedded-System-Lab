- [Introduction](#introduction)
- [Challenge 1 - Line follower, Part 1](#challenge-1---line-follower-part-1)
  - [Description](#description)
  - [Results](#results)
- [Challenge 2 - Line follower, Part 2](#challenge-2---line-follower-part-2)
  - [Description](#description-1)
  - [Results](#results-1)
- [Challenge 3 - Stay inside box and obstacle avoidance](#challenge-3---stay-inside-box-and-obstacle-avoidance)
  - [Description](#description-2)
  - [Results](#results-2)
- [Challenge 4 - Line and corridor follower](#challenge-4---line-and-corridor-follower)
  - [Description](#description-3)
  - [Results](#results-3)
- [Conclusion](#conclusion)


# Introduction

# Challenge 1 - Line follower, Part 1

![image-20230606102822213](imgs/image-20230606102822213.png)

## Description

Challenge 1 aims to **make the robot follow a black line of a rounded shape, based on threashold values of the ground sensors**. Ground LEDs emit light to the ground, and the ground sensors will detect the reflected light. 

The main priciple is that **the ground sensors will detect a higher value if the ground is black, and a lower value if the ground is white**. The robot will move forward if both ground sensors detect white, turn left if the left ground sensor detects black, and turn right if the right ground sensor detects black.

**PWM control** is used to control the speed of the robot.

## Results

Firstly, I have to make sure the car can **move forward in a straight line**.  **I use a duty cycle of 0.09 (9%), which is `float duty = 0.09;`, to make sure the car moves forward not too fastly or too slowly**. However, the car does not move in a straight line at times. It may turn left or turn right. Thus, **I add other two variables `float left_adjust = 1.20;` and `float right_adjust = 1.22;` to adjust the speed of the left and right motor respectively**. Therefore, the car moves in a straight line after the adjustment.

Secondly, **black line threashold values of the ground sensors need to be determined.** I put the robot on the black line, and the values of the ground sensors are read and printed to the serial monitor. The values of the ground sensors are around 900 (threashold values are not stable and vary among the robots), so **I set the black line threashold values to 900, `int blackThread[4] = {900, 900, 900, 900};`.**

Furthermore, I **use a green LED to indicate the status of the robot**. The green LED will be on if the robot detects a black line, and off if the robot detects a white line.

However, I want to mention that although four ground sensors are used, **only two sensors values (sensor 1 and sensor 2) are utilized** to determine the black line. Other two sensor values will be used in Challenge 2 to detect big angles.

Cosequently, the robot can follow a black line with the following main code (only global variables, stepup and main loop are shown):


```c
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

/**
 * @brief To initialize the register for ground IR sensor and ground IR LED
 */
void groundIRInit()
{
...
}
/**
 * @brief To initialize the register for left and right motor
 */
void motorInit()
{
...
}
/**
 * @brief To initialize the register for green LED
 */
void greenLEDInit()
{
...
}

/*************** Ground IR LED Control ***************/

/**
 * @brief Turn on the ground LED numbered `lineIndex`
 * @param lineIndex
 */
void groundLEDon(unsigned char lineIndex)
{
...
}
/**
 * @brief Turn off the ground LED numbered `lineIndex`
 * @param lineIndex
 */
void groundLEDoff(unsigned char lineIndex)
{
...
}

/*************** Green LED Control ***************/

/**
 * @brief Turn on the green LED
 */
void greenLEDon()
{
...
}

/**
 * @brief Turn off the green LED
 */
void greenLEDoff()
{
...
}

/*************** GroundSensor Control ***************/

/**
 * @brief Read the value of ground IR sensor and store it in `line`
 */
void readGroundIRSensors()
{
...
}

/**
 * @brief Show the value of ground IR sensor
 */
void showGroundIRSensorLine()
{
...
}

/*************** Motor Control ***************/

/**
 * @brief Move the left motor forward
 * @param duty
 */
void leftMotorForward(float duty)
{
...
}

/**
 * @brief Move the right motor forward
 * @param duty
 */
void rightMotorForward(float duty)
{
...
}

/**
 * @brief Move the left motor backward
 * @param duty
 */
void leftMotorBackward(float duty)
{
...
}

/**
 * @brief Move the right motor backward
 * @param duty
 */
void rightMotorBackward(float duty)
{
...
}

/**
 * @brief Stop the left motor
 */
void leftMotorStop()
{
...
}

/**
 * @brief Stop the right motor
 */
void rightMotorStop()
{
...
}

/**
 * @brief Turn left
 * @param duty
 */
void leftTurn(float duty)
{
...
}

/**
 * @brief Turn right
 * @param duty
 */
void rightTurn(float duty)
{
...
}

/**
 * @brief Move forward
 * @param duty
 */
void moveForward(float duty)
{
...
}

/**
 * @brief Stop the car
 */
void stop()
{
...
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
```

<video src="./videoChallenge1.mp4"></video>



# Challenge 2 - Line follower, Part 2

![image-20230606102840649](imgs/image-20230606102840649.png)

## Description

Challenge 2 **extends the line follower in Challenge 1 to a more complex shape** with big angles. Comparing to Challenge 1, where only ground sensor 1 and 2 are used, **Challenge 2 uses all four ground sensors**.

The robot will move forward if all the ground sensors detect white, turn left if the  ground sensor 1 detects black, and turn right if the ground sensor 2 detects black. The robot can also detect big angles with ground sensor 0 and ground sensor 3. It will turn left and right in a big angle if the sensor 0 and sensor 3 detect black respectively.

## Results

Most of the code is the same as Challenge 1. The main difference is that the robot can detect big angles with ground sensor 0 and ground sensor 3. So I add two other functions `void leftTurnBigAngle(float duty)` and `void rightTurnBigAngle(float duty)` to turn left and right in a big angle.

The robot can follow a black line in Challeng 2 with the following main code (only global variables, stepup and main loop are shown):

```c
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

/**
 * @brief To initialize the register for ground IR sensor and ground IR LED
 */
void groundIRInit()
{
...
}
/**
 * @brief To initialize the register for left and right motor
 */
void motorInit()
{
...
}
/**
 * @brief To initialize the register for green LED
 */
void greenLEDInit()
{
...
}

/*************** Ground IR LED Control ***************/

/**
 * @brief Turn on the ground LED numbered `lineIndex`
 * @param lineIndex
 */
void groundLEDon(unsigned char lineIndex)
{
...
}
/**
 * @brief Turn off the ground LED numbered `lineIndex`
 * @param lineIndex
 */
void groundLEDoff(unsigned char lineIndex)
{
...
}

/*************** Green LED Control ***************/

/**
 * @brief Turn on the green LED
 */
void greenLEDon()
{
...
}

/**
 * @brief Turn off the green LED
 */
void greenLEDoff()
{
...
}

/*************** GroundSensor Control ***************/

/**
 * @brief Read the value of ground IR sensor and store it in `line`
 */
void readGroundIRSensors()
{
...
}

/**
 * @brief Show the value of ground IR sensor
 */
void showGroundIRSensorLine()
{
...
}

/*************** Motor Control ***************/

/**
 * @brief Move the left motor forward
 * @param duty
 */
void leftMotorForward(float duty)
{
...
}

/**
 * @brief Move the right motor forward
 * @param duty
 */
void rightMotorForward(float duty)
{
...
}

/**
 * @brief Move the left motor backward
 * @param duty
 */
void leftMotorBackward(float duty)
{
...
}

/**
 * @brief Move the right motor backward
 * @param duty
 */
void rightMotorBackward(float duty)
{
...
}

/**
 * @brief Stop the left motor
 */
void leftMotorStop()
{
...
}

/**
 * @brief Stop the right motor
 */
void rightMotorStop()
{
...
}

/**
 * @brief Turn left
 * @param duty
 */
void leftTurn(float duty)
{
...
}

/**
 * @brief Turn right
 * @param duty
 */
void rightTurn(float duty)
{
...
}

/**
 * @brief Turn left with big angle
 * @param duty
 */
void leftTurnBigAngle(float duty)
{
...
}

/**
 * @brief Turn right with big angle
 * @param duty
 */
void rightTurnBigAngle(float duty)
{
...
}

/**
 * @brief Move forward
 * @param duty
 */
void moveForward(float duty)
{
...
}

/**
 * @brief Stop the car
 */
void stop()
{
...
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
    readGroundIRSensors();          // read the value of ground IR sensor
    moveForward(duty);              // move forward
    if ((line[0] > blackThread[0])) // black line on the most left sensor
    {
        leftTurnBigAngle(duty); // turn left with big angle
        greenLEDon(0);          // turn on the green LED 0
    }
    else
    {
        greenLEDoff(0); // turn off the green LED 0
    }
    if ((line[3] > blackThread[3])) // black line on the most right sensor
    {
        rightTurnBigAngle(duty); // turn right with big angle
        greenLEDon(3);           // turn on the green LED 3
    }
    else
    {
        greenLEDoff(3); // turn off the green LED 3
    }
    if (line[1] > blackThread[1]) // black line on left
    {
        greenLEDon(1);  // turn on the green LED 1
        leftTurn(duty); // turn left
    }
    else
    {
        greenLEDoff(1); // turn off the green LED 1
    }
    if (line[2] > blackThread[2]) // black line on right
    {
        greenLEDon(2);   // turn on the green LED 2
        rightTurn(duty); // turn right
    }
    else
    {
        greenLEDoff(2); // turn off the green LED 2
    }
}
```

<video src="./video/Challenge2.mp4"></video>

# Challenge 3 - Stay inside box and obstacle avoidance

![image-20230606105733112](imgs/image-20230606105733112.png)

## Description




## Results


# Challenge 4 - Line and corridor follower

## Description

## Results



<video src="./video/Challenge4.mp4"></video>

# Conclusion

