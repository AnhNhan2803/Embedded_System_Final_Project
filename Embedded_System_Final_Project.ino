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
#include <avr/pgmspace.h>
#include <Wire.h>
#include <Servo.h>
#include <ICM_20948.h>
#include "ultrasonic.h"
// #include "HCSR04.h"

// These must be defined before including TinyEKF.h
#define Nsta 3 // 3 state values: posX, posY, and orientation
#define Mobs 3 // 3 measurements: gyroZ, accelX, and accelY
// #define Mobs 4 // 4 measurements: gyroZ, accelX, and accelY and distance (optional due to the lack of resources)

#include <TinyEKF.h>

/******************************************************************************
* PREPROCESSOR CONSTANTS
*******************************************************************************/
#define AD0_VAL           1
#define SERVO_LEFT_ANGLE  30
#define SERVO_RIGHT_ANGLE 150
#define SERVO_STRIGHT     90

/******************************************************************************
* PREPROCESSOR MACROS
*******************************************************************************/


/******************************************************************************
* TYPEDEFS
*******************************************************************************/
enum class car_direction{
    FORWARD = 0,
    BACKWARD,
    LEFT,
    RIGHT
};

struct Point {
  double x;
  double y;
};

class Fuser : public TinyEKF {
    public:
        Fuser() 
        {
            for (int i = 0; i < Nsta; ++i) 
            {
                setQ(i, i, 0.001);
            }

            for (int i = 0; i < Mobs; ++i) 
            {
                setR(i, i, 0.01);
            }
        }

    protected:
        void Fuser::model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]) 
        {
            // Process model
            double dt = 0.1; // Time step, assuming constant

            // double gyroZ = this->z[0]; // Get the gyroZ measurement from the EKF's z array

            // Process model is f(x) = x, try to make it linearly
            fx[0] = this->x[0]; // Keep posX constant
            fx[1] = this->x[1]; // Keep posY constant
            // fx[2] = this->x[2] + dt * gyroZ; // Update orientation based on gyroZ measurement
            fx[2] = this->x[2] + dt * this->x[2]; // Update orientation based on gyroZ measurement

            // So process model Jacobian is identity matrix
            F[0][0] = 1;
            F[1][1] = 1;
            F[2][2] = 1;

            // Measurement model (hx)
            hx[0] = this->x[2]; // gyroZ is directly mapped to orientation
            hx[1] = this->x[0]; // accelX is directly mapped to posX
            hx[2] = this->x[1]; // accelY is directly mapped to posY

            // Measurement model Jacobian (H)
            H[0][2] = 1; // gyroZ to orientation
            H[1][0] = 1; // accelX to posX
            H[2][1] = 1; // accelY to posY
        }
};

/******************************************************************************
* VARIABLE DEFINITIONS
*******************************************************************************/
const byte pingPin = 9; // Trigger Pin of Ultrasonic Sensor
const byte echoPin = 10; // Echo Pin of Ultrasonic Sensor
ultrasonic hcsr04(pingPin, echoPin, 400); 
ICM_20948_I2C myICM;
Servo servo;
Servo servoMotorLeft;   
Servo servoMotorRight;
Fuser ekf;

// Temporarily predefined path at the beginning, will edit later
Point waypoints[4] = {
  {0, 0},
  {10, 0},
  {10, 10},
  {0, 10}
};

// Servo motor objects for steering and throttle control
Servo steeringServo;
Servo throttleServo;

/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
void initializeIMU(void);
void printRawAGMTIMU(ICM_20948_AGMT_t agmt);
void printPaddedInt16bIMU(int16_t val);
car_direction updateOrientation(void);
double distance(Point a, Point b);
void updateCarMovement(void);
void updateServoMotors(car_direction desiredDirection);

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
    byte servoPin = 11;
    servo.attach(servoPin);

    // Attach the servo motors to the corresponding pins
    steeringServo.attach(3);
    throttleServo.attach(5);

    // Initialize the servo motors
    steeringServo.write(90);
    throttleServo.write(90);

    // Start some configuration for signal processing and 
    // required algorithms here
}

void loop() 
{
    static ICM_20948_AGMT_t rawData ;
    // Get IMU data
    if(myICM.dataReady())
    {
        rawData = myICM.getAGMT();
        printRawAGMTIMU(rawData);
    }

    // delay(2);

    // Process the Ultrasonic sensor here
    // car_direction direction = updateOrientation();

    // Do some signal processing and running the EKF filtering here
    // There are 2 measurements of gyroZ and distance by the ultrasonic
    double z[Mobs] = {myICM.gyrZ(), myICM.accX(), myICM.accY()};
    ekf.step(z);

    // Report measured and predicte/fused values
    // 2 measurements and 3 states
    Serial.print(z[0]);
    Serial.print(" ");
    Serial.print(z[1]);
    Serial.print(" ");
    Serial.print(ekf.getX(0));
    Serial.print(" ");
    Serial.print(ekf.getX(1));
    Serial.print(" ");
    Serial.println(ekf.getX(2));

    // delay to allow the servo motor to move to the new position
    // Calculate the desired direction
    updateCarMovement();

    delay(100);
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
    if (myICM.status != ICM_20948_Stat_Ok)
    {
        Serial.print(F("startupMagnetometer returned: "));
        Serial.println(myICM.statusString());
    }

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
      Serial.print("0");
    }
    if (val < 1000)
    {
      Serial.print("0");
    }
    if (val < 100)
    {
      Serial.print("0");
    }
    if (val < 10)
    {
      Serial.print("0");
    }
  }
  else
  {
    Serial.print("-");
    if (abs(val) < 10000)
    {
      Serial.print("0");
    }
    if (abs(val) < 1000)
    {
      Serial.print("0");
    }
    if (abs(val) < 100)
    {
      Serial.print("0");
    }
    if (abs(val) < 10)
    {
      Serial.print("0");
    }
  }
  Serial.print(abs(val));
}

void printRawAGMTIMU(ICM_20948_AGMT_t agmt)
{
  Serial.print("RAW. Acc [ ");
  printPaddedInt16bIMU(agmt.acc.axes.x);
  Serial.print(", ");
  printPaddedInt16bIMU(agmt.acc.axes.y);
  Serial.print(", ");
  printPaddedInt16bIMU(agmt.acc.axes.z);
  Serial.print(" ], Gyr [ ");
  printPaddedInt16bIMU(agmt.gyr.axes.x);
  Serial.print(", ");
  printPaddedInt16bIMU(agmt.gyr.axes.y);
  Serial.print(", ");
  printPaddedInt16bIMU(agmt.gyr.axes.z);
  Serial.print(" ], Mag [ ");
  printPaddedInt16bIMU(agmt.mag.axes.x);
  Serial.print(", ");
  printPaddedInt16bIMU(agmt.mag.axes.y);
  Serial.print(", ");
  printPaddedInt16bIMU(agmt.mag.axes.z);
  Serial.print(" ], Tmp [ ");
  printPaddedInt16bIMU(agmt.tmp.val);
  Serial.print(" ]");
  Serial.println();
}

car_direction updateOrientation(void)
{
    delay(20);
    servo.write(SERVO_LEFT_ANGLE);
    delay(350);
    float left = hcsr04.measureDistance(true);
    servo.write(SERVO_RIGHT_ANGLE);
    delay(450);
    float right = hcsr04.measureDistance(true);
    servo.write(SERVO_STRIGHT);
    delay(200);
    if (left <= right)
      return car_direction::RIGHT; //represents turn right
    return car_direction::LEFT;//turn left
}

void updateCarMovement(void) 
{
    // Get the current posX, posY, and theta values from the EKF
    double posX = ekf.getX(0);
    double posY = ekf.getX(1);
    double theta = ekf.getX(2);

    // Updated desired direction based on 3 states position X, Y and orientation
    car_direction desiredDirection = calculateDesiredDirection(posX, posY, theta);

    // Update servo motors based on the determined direction
    updateServoMotors(desiredDirection);
}

// Returns the Euclidean distance between two points
double distance(Point a, Point b) 
{
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

// Function to calculate the desired direction based on the current position (posX, posY) and orientation (theta)
car_direction calculateDesiredDirection(double posX, double posY, double theta) 
{
    // Define a threshold distance to consider a waypoint reached
    const double threshold = 0.5;

    // Find the closest waypoint
    int closestWaypoint = 0;
    double minDistance = distance(waypoints[0], Point{posX, posY});

    int iteration = sizeof(waypoints)/sizeof(Point);

    for (int i = 1; i < iteration; i++) 
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
    car_direction desiredDirection;
    if (fabs(angleError) < M_PI / 8) 
    {  
        desiredDirection = car_direction::FORWARD;
    } 
    else if (angleError > 0) 
    {
        desiredDirection = car_direction::LEFT;
    } 
    else 
    {
        desiredDirection = car_direction::RIGHT;
    }

    return desiredDirection;
}

// Update the Servo Motor to control car wheels
void updateServoMotors(car_direction desiredDirection) 
{
    switch (desiredDirection) 
    {
        case car_direction::LEFT:
            steeringServo.write(70);
            throttleServo.write(180);
            break;

        case car_direction::RIGHT:
            steeringServo.write(110);
            throttleServo.write(180);
            break;

        case car_direction::BACKWARD:
            steeringServo.write(90);
            throttleServo.write(0);
            break;

        case car_direction::FORWARD:
        default:
            steeringServo.write(90);
            throttleServo.write(180);
            break;
    }
}
