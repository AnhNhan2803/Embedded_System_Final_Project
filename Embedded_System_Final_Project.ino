/*******************************************************************************
* Title                 :   Main program for Final project - Embedded System Class
* Filename              :   Final_Project.ino
* Author                :   Nhan and Ma
* Origin Date           :   04/18/2023
* Notes                 :   None
********************** ********************************************************/

/*******************************************************************************
* INCLUDE
*******************************************************************************/
#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include <Servo.h>
#include <ICM_20948.h>
#include "ultrasonic.h"

// These must be defined before including TinyEKF.h
#define Nsta 2 // 1 state values: orientation (or the yaw euler angle)
#define Mobs 4 // 3 measurements: gyroZ, and (accelX, accelY, accelZ) or (magX, magY, MagZ)
#include <TinyEKF.h>

/******************************************************************************
* PREPROCESSOR CONSTANTS
*******************************************************************************/
#define AD0_VAL           1
#define SERVO_LEFT_ANGLE  83
#define SERVO_RIGHT_ANGLE 83
#define SERVO_STRIGHT     83
#define MAX_OBSTACLE_THRESHOLD   30
#define RAD_TO_DEDGREE    57.3
#define MAX_ANGLE_THRESHOLD 5

/******************************************************************************
* PREPROCESSOR MACROS
*******************************************************************************/
// #define QUAT_ANIMATION // Uncomment this line to output data in the correct format for ZaneL's Node.js Quaternion animation tool: https://github.com/ZaneL/quaternion_sensor_3d_nodejs

// Define USE_ADVANCED_IMU_FEATURE if we want the DMP within the IMU sensor to 
// calculate the filtered orientation without the involvement of Kalman filter
// #define USE_ADVANCED_IMU_FEATURE
// #define CAR_TESTING

/******************************************************************************
* TYPEDEFS
*******************************************************************************/
enum class carState{
    STRAIGHT = 0,
    BACK,
    LEFT,
    RIGHT,
    STOP
};

struct Point {
  double x;
  double y;
};


/// @brief ////////////////////////////////////////////////////
// struct Waypoint {
//   int id;
//   float x;
//   float y;
//   float direction; // angle in degrees
// };

// const int numWaypoints = 3;

// Waypoint waypoints[numWaypoints] = {
//   {0, 0, 0, 0},       // A
//   {1, 10, 0, 0},      // B
//   {2, 10, 10, 90}     // C
// };

// struct Edge {
//   int startId;
//   int endId;
//   float distance;
// };

// const int numEdges = 2;

// Edge edges[numEdges] = {
//   {0, 1, 10}, // connection between A and B
//   {1, 2, 10}  // connection between B and C
// };

class Fuser : public TinyEKF {
    public:
        Fuser() 
        {
            for (uint8_t i = 0; i < Nsta; ++i) 
            {
                setQ(i, i, 0.001);
            }

            for (uint8_t i = 0; i < Mobs; ++i) 
            {
                setR(i, i, 0.01);
            }
        }

        void syncDeltaTime(double dt)
        {
            deltaTime = dt;
        }

    protected:
        double deltaTime;

        void Fuser::model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]) 
        {
            // Process model
            fx[0] = this->x[0]; // Orientation (yaw) as the state

            // So process model Jacobian is an identity matrix
            F[0][0] = 1;

            // Measurement model (hx)
            hx[0] = this->x[0]; // gyroZ is related to orientation (yaw)
            hx[1] = this->x[0]; // magnetometers XvsY is related to orientation (yaw)

            // Measurement model Jacobian (H)
            H[0][0] = 1; // gyroZ to orientation (yaw)
            H[1][0] = 1; // magnetometers to orientation (yaw)
        }
};

/******************************************************************************
* VARIABLE DEFINITIONS
*******************************************************************************/
const byte pingPin = 3; // Trigger Pin of Ultrasonic Sensor
const byte echoPin = 2; // Echo Pin of Ultrasonic Sensor

ultrasonic hcsr04(pingPin, echoPin, 400);

ICM_20948_I2C myICM;
Servo servo;

// Temporarily predefined path at the beginning, will edit later
Point waypoints[4] = {
  {0, 0},
  {10, 0},
  {10, 10},
  {0, 10}
};

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Select motor M1 and M2
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2);
const int carMaxSpeed = 200;

float gyroZError = 0.0;
float driftYawAngle;
float gyroYawAngle;

carState systemState = carState::STOP;
Fuser ekf;

/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
void initializeIMU(void);
void printRawAGMTIMU(ICM_20948_AGMT_t agmt);
void printPaddedInt16bIMU(int16_t val);
double distance(Point a, Point b);
void updateCarMovement(float yawAngle, carState direction);
void updateServoMotors(carState desiredDirection, uint8_t leftSpeed, uint8_t rightSpeed);

float driftHeadingvsNorth(void);
bool detectObstacle(void);
bool detectDeadEnd(void);
carState updatePath(void);

/******************************************************************************
* 2 MAIN ENTRANCE FUNCTIONS
*******************************************************************************/
void setup() 
{
    Serial.begin(115200);

    // wait until serial port opens for native USB devices
    while (!Serial) 
    {
        delay(1);
    }

    // Start the hardware I2C here with high speed clock
    // of 400kHz
    Wire.begin();   
    Wire.setClock(400000);

    /******************* HCSR04 Setup *********************/
    // Ultrasonic sensor does not require a specific order of
    // intialization, just need to config the trigger, echo
    // and do extra calculation within the ultrasonic class
    pinMode(pingPin, OUTPUT);
    pinMode(echoPin, INPUT);

    /********************* IMU Setup **********************/
    // Do not need to capture the return error since 
    // we will enter a 4ever while loop if falied to
    // init the ICM20948 
    initializeIMU();

    /********************* Servo Setup **********************/
    byte servoPin = 10;
    servo.attach(servoPin);

    // Initialize the motorshield here
    if (!AFMS.begin()) 
    {         
        // create with the default frequency 1.6KHz
        Serial.println("Could not find Motor Shield. Check wiring.");
        while (1);
    }
    Serial.println("Motor Shield found.");

    // Set the speed to start, from 0 (off) to 255 (max speed)
    rightMotor->setSpeed(150);
    rightMotor->run(FORWARD);
    // turn on motor
    rightMotor->run(RELEASE);

    leftMotor->setSpeed(150);
    leftMotor->run(FORWARD);
    // turn on motor
    leftMotor->run(RELEASE);

    // Calibrate the error coefficient of IMU gyroscope in resting
    for (int i=0; i<200; i++)
    {
        myICM.getAGMT();
        gyroZError += myICM.gyrZ();     
    }
    gyroZError /= 200;

    // Determinte the initial drift between the current heading and the North
    driftYawAngle = driftHeadingvsNorth();
    gyroYawAngle = driftYawAngle;
}

void loop() 
{
    ICM_20948_AGMT_t rawData ;
    static uint32_t timer = 0;

    // Measure the detatime of each sensors samples collection
    double dt = (timer != 0) ? ((double)(micros() - timer) / 1000000) : 0; // Calculate delta time in s
    timer = micros();

    // Get IMU data
    if(myICM.dataReady())
    {
        rawData = myICM.getAGMT();
        // printRawAGMTIMU(rawData);
    }

    // Assign to local variable for simple debugging first
    // float accelX = myICM.accX();
    // float accelY = myICM.accY();
    // float accelZ = myICM.accZ();

    // float gyroX = myICM.gyrX();
    // float gyroY = myICM.gyrY();
    float gyroZ = myICM.gyrZ();

    float magX = myICM.magX();
    float magY = myICM.magY();
    // float magZ = myICM.magZ();


    // Calculate the yaw angle based on magnetometer data and pitch+roll
    // float pitch = atan2(accelY ,( sqrt ((accelX * accelX) + (accelZ * accelZ))));
    // float roll = atan2(-accelX ,( sqrt((accelY * accelY) + (accelZ * accelZ))));

    // float Yh = (magZ * sin(roll)) + (magX * cos(roll));
    // float Xh = (magY * cos(roll)) + (magX * sin(roll) * sin(pitch)) - (magZ * sin(roll) * cos(pitch));

    // float yawMag = atan2(Yh, Xh) * RAD_TO_DEDGREE;

    // Calculate the yaw angle based on accelerometer
    // yaw =  atan(sqrt((accelX * accelX) + (accelY * accelY)) / accelZ) * RAD_TO_DEDGREE;

    // Yaw angle measured directly from the gyroscope_Z
    gyroYawAngle += (gyroZ - gyroZError) * dt;
    // The yaw angle measured by Magnetometer is relative to the North
    float magYawAngle = atan2(magY, magX) * RAD_TO_DEDGREE;

    // Do some signal processing and running the EKF filtering here
    // There are 2 measurements of gyroZ and distance by the ultrasonic
    double z[Mobs] = {gyroYawAngle, magYawAngle}; // CHANGE: Measurement vector
    ekf.syncDeltaTime(dt);
    ekf.step(z);

    // Report measured and predicte/fused values
    float filteredYawAngle = ekf.getX(0);

    Serial.print("gyroYawAngle: ");
    Serial.println(gyroYawAngle);
    Serial.print("magYawAngle: ");
    Serial.println(magYawAngle);

    Serial.print("Heading: ");
    Serial.println(filteredYawAngle);
    Serial.println("");

    bool obstacleDetected = detectObstacle();
    bool deadEndDetected = detectDeadEnd();

    // Update the car's planned path if necessary
    carState desiredDirection = carState::STRAIGHT;
    if (obstacleDetected || deadEndDetected) 
    {
        desiredDirection = updatePath();
    }

    // Handle the carState::STOP case
    if (desiredDirection == carState::STOP) 
    {
        // Stop the car
        leftMotor->run(RELEASE);
        rightMotor->run(RELEASE);
        
        // Add any additional actions here, such as:
        // - Notify the user
        // - Attempt to backtrack
        // - Wait for the obstacles to be removed

        // Add a delay to pause the loop for a certain amount of time
        delay(1000); // Wait for 1 second

        // After taking appropriate actions, re-check for obstacles and update the path
        obstacleDetected = detectObstacle();
        deadEndDetected = detectDeadEnd();
        if (obstacleDetected || deadEndDetected) 
        {
            desiredDirection = updatePath();
        }
    }

    // delay to allow the servo motor to move to the new position
    // Calculate the desired direction
    updateCarMovement(magYawAngle, desiredDirection);
}

/******************************************************************************
* STATIC FUNCTIONS
*******************************************************************************/
void initializeIMU(void)
{

    bool initialized = false;
    while (!initialized)
    {
        myICM.begin(Wire, AD0_VAL);

        Serial.print(F("Initialization of the sensor returned: "));
        Serial.println(myICM.statusString());
        if (myICM.status != ICM_20948_Stat_Ok)
        {
            Serial.println("Trying again...");
            delay(500);
        }
        else
        {
            initialized = true;
        }    
    }

    // Here we are doing a SW reset to make sure the device starts in a known state
    myICM.swReset();
    if (myICM.status != ICM_20948_Stat_Ok)
    {
        Serial.print(F("Software Reset returned: "));
        Serial.println(myICM.statusString());
    }
    delay(250);
    bool success = true; // Use success to show if the DMP configuration was successful

    // Now wake the sensor up
    myICM.sleep(false);
    myICM.lowPower(false);

    // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

    // Set Gyro and Accelerometer to a particular sample mode
    // options: ICM_20948_Sample_Mode_Continuous
    //          ICM_20948_Sample_Mode_Cycled
    myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
    if (myICM.status != ICM_20948_Stat_Ok)
    {
        Serial.print(F("setSampleMode returned: "));
        Serial.println(myICM.statusString());
    }
    
    // Set gyro sample rate divider with GYRO_SMPLRT_DIV
    // Set accel sample rate divider with ACCEL_SMPLRT_DIV_2
    ICM_20948_smplrt_t mySmplrt;
    mySmplrt.g = 19; // ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]). 19 = 55Hz. InvenSense Nucleo example uses 19 (0x13).
    mySmplrt.a = 19; // ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]). 19 = 56.25Hz. InvenSense Nucleo example uses 19 (0x13).
    //mySmplrt.g = 4; // 225Hz
    //mySmplrt.a = 4; // 225Hz
    //mySmplrt.g = 8; // 112Hz
    //mySmplrt.a = 8; // 112Hz
    myICM.setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt);
    if (myICM.status != ICM_20948_Stat_Ok)
    {
        Serial.print(F("setSampleRate returned: "));
        Serial.println(myICM.statusString());
    }

    // Set full scale ranges for both acc and gyr
    ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

    myFSS.a = gpm2; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                    // gpm2
                    // gpm4
                    // gpm8
                    // gpm16

    myFSS.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                    // dps250
                    // dps500
                    // dps1000
                    // dps2000

    myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
    if (myICM.status != ICM_20948_Stat_Ok)
    {
        Serial.print(F("setFullScale returned: "));
        Serial.println(myICM.statusString());
    }

    // Set up Digital Low-Pass Filter configuration
    ICM_20948_dlpcfg_t myDLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors
    myDLPcfg.a = acc_d473bw_n499bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                    // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                    // acc_d111bw4_n136bw
                                    // acc_d50bw4_n68bw8
                                    // acc_d23bw9_n34bw4
                                    // acc_d11bw5_n17bw
                                    // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                    // acc_d473bw_n499bw

    myDLPcfg.g = gyr_d361bw4_n376bw5; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                    // gyr_d196bw6_n229bw8
                                    // gyr_d151bw8_n187bw6
                                    // gyr_d119bw5_n154bw3
                                    // gyr_d51bw2_n73bw3
                                    // gyr_d23bw9_n35bw9
                                    // gyr_d11bw6_n17bw8
                                    // gyr_d5bw7_n8bw9
                                    // gyr_d361bw4_n376bw5

    myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
    if (myICM.status != ICM_20948_Stat_Ok)
    {
        Serial.print(F("setDLPcfg returned: "));
        Serial.println(myICM.statusString());
    }

    // Choose whether or not to use DLPF
    // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
    ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Acc, false);
    ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Gyr, false);
    Serial.print(F("Enable DLPF for Accelerometer returned: "));
    Serial.println(myICM.statusString(accDLPEnableStat));
    Serial.print(F("Enable DLPF for Gyroscope returned: "));
    Serial.println(myICM.statusString(gyrDLPEnableStat));

    // Choose whether or not to start the magnetometer
    myICM.startupMagnetometer();
    Serial.print(F("startupMagnetometer returned: "));
    Serial.println(myICM.statusString());

    Serial.println();
    Serial.println(F("Configuration complete!"));
}

// Below here are some helper functions to print the data nicely!
void printPaddedInt16bIMU(int16_t val)
{
  if (val > 0)
  {
    Serial.print(" ");
    if (val < 10000)
    {
      Serial.print(F("0"));
    }
    if (val < 1000)
    {
      Serial.print(F("0"));
    }
    if (val < 100)
    {
      Serial.print(F("0"));
    }
    if (val < 10)
    {
      Serial.print(F("0"));
    }
  }
  else
  {
    Serial.print(F("-"));
    if (abs(val) < 10000)
    {
      Serial.print(F("0"));
    }
    if (abs(val) < 1000)
    {
      Serial.print(F("0"));
    }
    if (abs(val) < 100)
    {
      Serial.print(F("0"));
    }
    if (abs(val) < 10)
    {
      Serial.print(F("0"));
    }
  }
  Serial.print(abs(val));
}

void printRawAGMTIMU(ICM_20948_AGMT_t agmt)
{
  Serial.print(F("RAW. Acc [ "));
  printPaddedInt16bIMU(agmt.acc.axes.x);
  Serial.print(F(", "));
  printPaddedInt16bIMU(agmt.acc.axes.y);
  Serial.print(F(", "));
  printPaddedInt16bIMU(agmt.acc.axes.z);
  Serial.print(F(" ], Gyr [ "));
  printPaddedInt16bIMU(agmt.gyr.axes.x);
  Serial.print(F(", "));
  printPaddedInt16bIMU(agmt.gyr.axes.y);
  Serial.print(F(", "));
  printPaddedInt16bIMU(agmt.gyr.axes.z);
  Serial.print(F(" ], Mag [ "));
  printPaddedInt16bIMU(agmt.mag.axes.x);
  Serial.print(F(", "));
  printPaddedInt16bIMU(agmt.mag.axes.y);
  Serial.print(F(", "));
  printPaddedInt16bIMU(agmt.mag.axes.z);
  Serial.print(F(" ], Tmp [ "));
  printPaddedInt16bIMU(agmt.tmp.val);
  Serial.print(F(" ]"));
  Serial.println();
}

///////////////////////////////////////////////////////////
// Car control functions

// void updateCarMovement(double distance)
// {
//     const double distanceThreshold = 10;
//     const int turningDelay = 500;

//     if (distance < distanceThreshold)
//     {
//         // If an obstacle is detected, stop the car
//         throttleServo.write(throttleValueStop);
//         delay(1000); // Wait for a moment

//         // Turn the car to find a new path
//         steeringServo.write(steeringAngleLeft);
//         throttleServo.write(throttleValueForward);
//         delay(turningDelay); // Adjust this delay based on the turning time required

//         // Move forward again
//         steeringServo.write(steeringAngleCenter);
//     }
//     else
//     {
//         // If there is no obstacle, continue moving forward
//         steeringServo.write(steeringAngleCenter);
//         throttleServo.write(throttleValueForward);
//     }
// }


void updateCarMovement(float yawAngle, carState direction)
{
    // Analyze the car orientation here to ensure that the car is moving straight
    int leftMotorSpeed = carMaxSpeed;
    int rightMotorSpeed = carMaxSpeed;
    carState desiredDirection;

#ifndef CAR_TESTING
    if (systemState == carState::STRAIGHT)
    {
        // The desired straight angle, assuming 0 degrees is straight or we can assign it directly
        // to the drift angle between car's heading orientation and the North
        const float desiredYaw = driftYawAngle;

        // Proportional control constant
        const float Kp = 5;

        // Calculate the error between the current yaw angle and the desired straight angle
        const float error = desiredYaw - yawAngle;

        // Calculate the speed adjustment based on the error and the proportional control constant
        int speedAdjustment = Kp * error;

        // Limit the speed adjustment to be within the motor's speed range
        speedAdjustment = constrain(speedAdjustment, -carMaxSpeed, carMaxSpeed);

        // Calculate the individual motor speeds
        leftMotorSpeed = carMaxSpeed - speedAdjustment;
        rightMotorSpeed = carMaxSpeed + speedAdjustment;

        // Ensure the motor speeds are within the motor's speed range
        leftMotorSpeed = constrain(leftMotorSpeed, 0, carMaxSpeed);
        rightMotorSpeed = constrain(rightMotorSpeed, 0, carMaxSpeed);
        desiredDirection = carState::STRAIGHT;
    }
    else
    {
        // We do not need to do any orientation correction during turning sides
        desiredDirection = carState::STRAIGHT;
    }

    updateServoMotors(desiredDirection, leftMotorSpeed, rightMotorSpeed);

#else
    updateServoMotors(carState::STRAIGHT, carMaxSpeed, carMaxSpeed);
#endif
}

// This function will be called at the startup phase or when the car just change the direction
float driftHeadingvsNorth(void)
{
    uint8_t idx = 0;
    float drift;

    while(1)
    {
        if(myICM.dataReady())
        {
            myICM.getAGMT();
            drift += (atan2(myICM.magY(), myICM.magX()) * RAD_TO_DEDGREE);
            if(++idx == 10)
            {
                break;
            }
        }
    }

    // Calculate the average value to reduce the bias
    drift /= idx;

    return drift;
}

bool detectObstacle(void) 
{
    // Calculate the distance to the obstacle, measure the distance 3 times
    // and take average value to reduce the noise
    int distance = 0;
    for (uint8_t idx=0; idx<3; idx++)
    {
        distance+=hcsr04.measureDistance(true);
    }
    distance/=3;

    // If there's an obstacle within a certain distance threshold, return true
    if (distance < MAX_OBSTACLE_THRESHOLD) 
    {
        return true;
    }
    return false;
}


bool detectDeadEnd(void) 
{
    bool leftObstacle, rightObstacle, frontObstacle;

    // Check for an obstacle in front of the car
    servo.write(SERVO_STRIGHT);
    delay(200); // Give the servo time to rotate
    frontObstacle = detectObstacle();


    // Rotate the ultrasonic sensor to the left
    servo.write(SERVO_LEFT_ANGLE);
    delay(200); // Give the servo time to rotate
    leftObstacle = detectObstacle();

    // Rotate the ultrasonic sensor to the right
    servo.write(SERVO_RIGHT_ANGLE);
    delay(200); // Give the servo time to rotate
    rightObstacle = detectObstacle();

    // Rotate the ultrasonic sensor back to the center
    servo.write(SERVO_STRIGHT);
    delay(200); // Give the servo time to rotate

    // If obstacles are detected in all directions, it's a dead end
    if (leftObstacle && rightObstacle && frontObstacle) {
        return true;
    }

    return false;
}

carState updatePath(void) 
{
    carState direction;
    // Check for obstacles in front, left, and right directions
    bool frontObstacle = detectObstacle();
    servo.write(SERVO_LEFT_ANGLE);
    delay(200);
    bool leftObstacle = detectObstacle();
    servo.write(SERVO_RIGHT_ANGLE);
    delay(200);
    bool rightObstacle = detectObstacle();
    servo.write(SERVO_STRIGHT);
    delay(200);

    // Decide on a direction based on the presence of obstacles
    if (!frontObstacle) 
    {
        direction = carState::STRAIGHT;
    } 
    else if (!leftObstacle) 
    {
        direction = carState::LEFT;
    } 
    else if (!rightObstacle) 
    {
        direction = carState::RIGHT;
    } 
    else 
    {
        direction = carState::STOP;
    }

    return direction;
}


// Returns the Euclidean distance between two points
double distance(Point a, Point b) 
{
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

// Function to calculate the desired direction based on the current position (posX, posY) and orientation (theta)
carState calculateDesiredDirection(double posX, double posY, double theta) 
{
    // Define a threshold distance to consider a waypoint reached
    const double threshold = 0.5;

    // Find the closest waypoint
    uint8_t closestWaypoint = 0;
    double minDistance = distance(waypoints[0], Point{posX, posY});

    uint8_t iteration = (uint8_t)sizeof(waypoints)/sizeof(Point);

    for (uint8_t i = 1; i < iteration; i++) 
    {
        double currentDistance = distance(waypoints[i], Point{posX, posY});
        if (currentDistance < minDistance) 
        {
            closestWaypoint = i;
            minDistance = currentDistance;
        }
    }

    // Check if the car is close enough to the waypoint
    if (minDistance < threshold) 
    {
        // Move to the next waypoint in the list (circular)
        closestWaypoint = (closestWaypoint + 1) % iteration;
    }

    // Calculate the angle to the closest waypoint
    double angleToWaypoint = atan2(waypoints[closestWaypoint].y - posY, waypoints[closestWaypoint].x - posX);

    // Calculate the difference between the current orientation and the angle to the waypoint
    double angleError = angleToWaypoint - theta;

    // Make sure the angle error is between -pi and pi
    while (angleError > M_PI) 
    {
        angleError -= 2 * M_PI;
    }
    while (angleError < -M_PI) 
    {
        angleError += 2 * M_PI;
    }

    // Determine the desired direction based on the angle error
    carState desiredDirection;
    if (fabs(angleError) < M_PI / 8) 
    {  
        desiredDirection = carState::STRAIGHT;
    } 
    else if (angleError > 0) 
    {
        desiredDirection = carState::LEFT;
    } 
    else 
    {
        desiredDirection = carState::RIGHT;
    }

    return desiredDirection;
}

// Update the Servo Motor to control car wheels
void updateServoMotors(carState desiredDirection, uint8_t leftSpeed, uint8_t rightSpeed) 
{
    systemState = desiredDirection;

    switch (systemState) 
    {
        case carState::LEFT:
            rightMotor->run(FORWARD);
            leftMotor->run(RELEASE);
            rightMotor->setSpeed(rightSpeed);
            leftMotor->setSpeed(leftSpeed);
            break;

        case carState::RIGHT:
            rightMotor->run(RELEASE);
            leftMotor->run(FORWARD);
            rightMotor->setSpeed(rightSpeed);
            leftMotor->setSpeed(leftSpeed);
            break;

        case carState::BACK:
            rightMotor->run(BACKWARD);
            leftMotor->run(BACKWARD);
            rightMotor->setSpeed(rightSpeed);
            leftMotor->setSpeed(leftSpeed);
            break;

        case carState::STRAIGHT:
        default:
            rightMotor->run(FORWARD);
            leftMotor->run(FORWARD);
            rightMotor->setSpeed(rightSpeed);
            leftMotor->setSpeed(leftSpeed);
            break;
    }
}
