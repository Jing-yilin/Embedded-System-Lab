[toc]

# Introduction

# Challenge 1 - Line follower, Part 1

## Description

Challenge 1 aims to make the robot follow a black line, based on threadhold values of the ground sensors. GroundLEDs emit light to the ground, and the ground sensors will detect the reflected light. The main priciple is that the ground sensors will detect a higher value if the ground is black, and a lower value if the ground is white. The robot will move forward if both sensors detect white, turn left if the left sensor detects black, and turn right if the right sensor detects black.

PWM control is used to control the speed of the robot.

## Results

Firstly, I have to make sure the car can move forward in a straight line. I use the following code to test the motor. I use a duty cycle of 0.09 (9%), which is `float duty = 0.09;`, to make sure the car moves forward not too fastly or too slowly. However, the car does not move in a straight line at times. It may turn left or turn right. Thus, I add other two variables `float left_adjust = 1.20;` and `float right_adjust = 1.22;` to adjust the speed of the left and right motor respectively. Therefore, the car moves in a straight line after the adjustment.


Secondly, black line threadhold values of the ground sensors need to be determined.
I put the robot on the black line, and the values of the ground sensors are read and printed to the serial monitor. The values of the ground sensors are around 900 (threadhold values are not stable and vary among the robots), so I set the black line threadhold values to 900, `int blackThread[4] = {900, 900, 900, 900};`.

Furthermore, I use a green LED to indicate the status of the robot. The green LED will be on if the robot detects a black line, and off if the robot detects a white line.

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

/*************** Initialization Control ***************/
...
/*************** Ground IR LED Control ***************/
...
/*************** Green LED Control ***************/
...
/*************** GroundSensor Control ***************/
...
/*************** Motor Control ***************/
..
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


## Description



## Results

# Challenge 2 - Line follower, Part 2

## Description

## Results

# Challenge 3 - Stay inside box and obstacle avoidance

## Description

## Results


# Challenge 4 - Line and corridor follower

## Description

## Results

# Conclusion

