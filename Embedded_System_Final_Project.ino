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
#include <EEPROM.h>
#include "ultrasonic.h"

// These must be defined before including TinyEKF.h
#define Nsta 1 // 1 state values: orientation (or the yaw euler angle)
#define Mobs 2 // 3 measurements: gyroZ, and (accelX, accelY, accelZ) or (magX, magY, MagZ)
#include <TinyEKF.h>

/******************************************************************************
* PREPROCESSOR CONSTANTS
*******************************************************************************/
#define AD0_VAL                     1
#define SERVO_LEFT_ANGLE            143
#define SERVO_RIGHT_ANGLE           83
#define SERVO_STRIGHT               23

#define RAD_TO_DEDGREE              (180 / M_PI)
#define DEDGREE_TO_RAD              (M_PI / 180)
#define MAX_ANGLE_THRESHOLD         5
#define CALIBRATION_ITERATIONS      200
#define CALIBRATION_ITERATIONS_MAG  (CALIBRATION_ITERATIONS * 50) // 1min30 for magneto calibration

// Macro for PID car control
#define MAX_OBSTACLE_THRESHOLD      7
// Need to tune this carefully
#define MAX_WALL_THRESHOLD          MAX_OBSTACLE_THRESHOLD
#define MAX_WALL_RIGHT_THRESHOLD    (MAX_OBSTACLE_THRESHOLD * 2)

/******************************************************************************
* PREPROCESSOR MACROS
*******************************************************************************/
// #define CAR_TESTING
#define IMU_CALIBRATION_ACCEL_GYRO
#define IMU_CALIBRATION_MAGNETOMETER

/******************************************************************************
* TYPEDEFS
*******************************************************************************/
enum class carState{
    STRAIGHT = 0,
    BACK,
    LEFT,
    RIGHT,
    ROTATE_LEFT_90,
    ROTATE_RIGHT_90,
    STOP
};

struct Point {
  double x;
  double y;
};

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
// Only use 2 Ultrasonic sensors right and center (the center can rotate but optionally)
ultrasonic hcsr04Center(3, 2, 400);
ultrasonic hcsr04Right(5, 4, 400);

ICM_20948_I2C myICM;
Servo servo;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Select motor M1 and M2
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2);
const int carMaxSpeed = 120;
const int carMinSpeed = 70;

#ifdef IMU_CALIBRATION_ACCEL_GYRO
float gyroZError = 0.0;
float accelXError = 0.0;
float accelYError = 0.0;
float accelZError = 0.0;
#endif

#ifdef IMU_CALIBRATION_MAGNETOMETER
// Calibration values
float magOffsetX = 0.0, magOffsetY = 0.0, magOffsetZ = 0.0;
float magScaleX = 1.0, magScaleY = 1.0, magScaleZ = 1.0;
#endif

float driftYawAngle;
float prevMagYawAngle;
float cumulativeMagYawAngle;
float gyroYawAngle;

Fuser ekf;

/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
void initializeIMU(void);
void printRawAGMTIMU(ICM_20948_AGMT_t agmt);
void printPaddedInt16bIMU(int16_t val);
void updateCarMovement(float yawAngle, carState direction, uint8_t leftSpeed, uint8_t rightSpeed);
void updateServoMotors(carState desiredDirection, uint8_t leftSpeed, uint8_t rightSpeed);
float driftHeadingvsNorth(void);
void resetIMUParameters(void);

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
    pinMode(3, OUTPUT);
    pinMode(2, INPUT);
    pinMode(5, OUTPUT);
    pinMode(4, INPUT);


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
        Serial.println(F("Could not find Motor Shield. Check wiring."));
        while (1);
    }
    Serial.println(F("Motor Shield found."));

    // Set the speed to start, from 0 (off) to 255 (max speed)
    rightMotor->setSpeed(150);
    rightMotor->run(FORWARD);
    // turn on motor
    rightMotor->run(RELEASE);

    leftMotor->setSpeed(150);
    leftMotor->run(FORWARD);
    // turn on motor
    leftMotor->run(RELEASE);

    uint16_t sampleCnt = 0;

#ifdef IMU_CALIBRATION_ACCEL_GYRO
    /////////////////////////////////////////////////////
    /// Calculate the zero error at the startup phase ///
    /////////////////////////////////////////////////////
    /// We calibrate the IMU sensor under the raw format
    int32_t accel_bias[3] = {0, 0, 0};
    int32_t gyro_bias = 0; // Only account for gyro Z
    ICM_20948_AGMT_t rawData;
    // Since teh sparkfun library does not support retrieving the resolution
    // from hardware, temporarily use the fix value here
    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384; // = 16384 LSB/g

    Serial.println(F("Start the Accelerometer and Gyroscope calbration!"));
    while(1)
    {
        if(myICM.dataReady())
        {
            sampleCnt++;
            rawData = myICM.getAGMT();

            // Accumulate the zero error for accelerometer and gyroscope
            accel_bias[0] += (int32_t)rawData.acc.axes.x;
            accel_bias[1] += (int32_t)rawData.acc.axes.y;
            accel_bias[2] += (int32_t)rawData.acc.axes.z;
            gyro_bias  += (int32_t)rawData.gyr.axes.z;
        }

        if(sampleCnt == CALIBRATION_ITERATIONS)
        {
            accel_bias[0] /= (int32_t)sampleCnt;
            accel_bias[1] /= (int32_t)sampleCnt;
            accel_bias[2] /= (int32_t)sampleCnt;
            gyro_bias  /= (int32_t)sampleCnt;

            // Since Accel z lies on the same axis with gravity, need to treat it different
            if (accel_bias[2] > 0L)
            {
                accel_bias[2] -= (int32_t) accelsensitivity;
            }
            else
            {
                accel_bias[2] += (int32_t) accelsensitivity;
            }

            // Convert to valid format of float value
            gyroZError = (float)gyro_bias/(float)gyrosensitivity;
            accelXError = (float)accel_bias[0] / (float)accelsensitivity; 
            accelYError = (float)accel_bias[1] / (float)accelsensitivity; 
            accelZError = (float)accel_bias[2] / (float)accelsensitivity; 
    
            // Completing the calibration section the break the loop
            break;
        }

        delay(10);
    }
    Serial.println(F("Complete Accel and Gyro calibration!"));
#endif

#ifdef IMU_CALIBRATION_MAGNETOMETER
    // Start calibrate the magnetometer here if requireed
    Serial.println(F("Start Magnetometer calibration ?"));

    while(!Serial.available())
    {   
        delay(5);
    }

    int character = Serial.read();

    if(character == 'y')
    {
        //Reset chracter to capture the stop calibration keyword
        character = 0;

        // Prepare all required parameters to calibrate magnetometer
        float minX = 4900, maxX = -4900;
        float minY = 4900, maxY = -4900;
        float minZ = 4900, maxZ = -4900;
        float magX, magY, magZ;
        float magDiffX, magDiffY, magDiffZ;
        sampleCnt = 0;

        // Calibrate Magnetometer separately
        Serial.println(F("Start the Magnetometer calbration!"));
        while(1)
        {
            if(myICM.dataReady())
            {
                sampleCnt++;
                
                rawData = myICM.getAGMT();
                
                // Accumulate error for magnetometer 
                magX = myICM.magX();
                magY = myICM.magY();
                magZ = myICM.magZ();

                // Update min and max values
                minX = min(minX, magX);
                maxX = max(maxX, magX);
                minY = min(minY, magY);
                maxY = max(maxY, magY);
                minZ = min(minZ, magZ);
                maxZ = max(maxZ, magZ);
            }

            if(sampleCnt == CALIBRATION_ITERATIONS_MAG || character == 'e')
            {
                magDiffX = maxX - minX;
                magDiffY = maxY - minY;
                magDiffZ = maxZ - minZ;

                float diff = (magDiffX + magDiffY + magDiffZ) / 3;

                // Calculate offsets and scaling factors
                magOffsetX = (minX + maxX) / -2.0;
                magOffsetY = (minY + maxY) / -2.0;
                magOffsetZ = (minZ + maxZ) / -2.0;

                magScaleX = diff / magDiffX;
                magScaleY = diff / magDiffY;
                magScaleZ = diff / magDiffZ;

                Serial.println(magOffsetX);
                Serial.println(magOffsetY);
                Serial.println(magOffsetZ);
                Serial.println(magScaleX);
                Serial.println(magScaleY);
                Serial.println(magScaleZ);

                // Store calibration data here so we do not need to calibrate in the next time
                EEPROM.put(0, magOffsetX); 
                EEPROM.put(4, magOffsetY); 
                EEPROM.put(8, magOffsetZ); 
                EEPROM.put(12, magScaleX); 
                EEPROM.put(16, magScaleY); 
                EEPROM.put(20, magScaleZ); 

                // Completing the calibration section the break the loop
                break;
            }

            if(Serial.available())
            {
                // Maybe we want to stop Magnetometer calibration before the deadline
                // by receiving 'e' keyword which mean END
                character = Serial.read();
            }

            delay(5);
        }
        Serial.println(F("Complete Magneto calibration!"));
    }
    // Reload the set of Magnetometer calibration from EEPROM
    else if(character == 'r')
    {
        Serial.println(F("Bypass Magnetometer calibration!"));
        EEPROM.get(0, magOffsetX); 
        EEPROM.get(4, magOffsetY); 
        EEPROM.get(8, magOffsetZ); 
        EEPROM.get(12, magScaleX); 
        EEPROM.get(16, magScaleY); 
        EEPROM.get(20, magScaleZ); 

        Serial.println(magOffsetX);
        Serial.println(magOffsetY);
        Serial.println(magOffsetZ);
        Serial.println(magScaleX);
        Serial.println(magScaleY);
        Serial.println(magScaleZ);    
    }
    else
    {
        // mostly receive 'n' here, use the scale of 1.0 and offset of 0.0
        Serial.println(F("Bypass Magnetometer calibration!"));
    }
#endif

    // Determinte the initial drift between the current heading and the North
    resetIMUParameters();
    // gyroYawAngle = driftYawAngle;
}

// Loop
void loop() 
{
    ICM_20948_AGMT_t rawData;
    static uint32_t timer = 0;
    static uint16_t counter = 0;
    static bool isStartMotor = false;
    static int previousPIDErr = 0;
    static int integral = 0;
    float filteredYawAngle;    

    // Measure the detatime of each sensors samples collection
    double dt = (timer != 0) ? ((double)(micros() - timer) / 1000000) : 0; 
    timer = micros();

    if(Serial.available())
    {
        int character = Serial.read();

        if(character == 's')
        {
            isStartMotor = true;
        }
        else if(character == 'e')
        {
            isStartMotor = false;
        }
        else if(character == 'r')
        {
            // Restart every parameters
            resetIMUParameters();
        }
    }

    // Get IMU data if there is at least on available sample
    if(myICM.dataReady())
    {
        rawData = myICM.getAGMT();
        // printRawAGMTIMU(rawData);

#ifdef IMU_CALIBRATION_MAGNETOMETER
        float magX = (myICM.magX() + magOffsetX) * magScaleX;
        float magY = (myICM.magY() + magOffsetY) * magScaleY;
        float magZ = (myICM.magY() + magOffsetZ) * magScaleZ;
#else
        float magX = myICM.magX();
        float magY = myICM.magY();
        float magZ = myICM.magY();
#endif

#ifdef IMU_CALIBRATION_ACCEL_GYRO
        float accelX = myICM.accX() - accelXError;
        float accelY = myICM.accY() - accelYError;
        float accelZ = myICM.accZ();

        float angularZ = (myICM.gyrZ() - gyroZError) * dt;
#else
        float accelX = myICM.accX();
        float accelY = myICM.accY();
        float accelZ = myICM.accZ();

        float angularZ = myICM.gyrZ() * dt;
#endif
        //////////////////////////////////////////////////////
        // Yaw angle measured directly from the gyroscope_Z //
        //////////////////////////////////////////////////////
        gyroYawAngle += angularZ;

        // Do the compensation for angularZ if exceed valid range
        if(gyroYawAngle >= 360.0)
        {
            // Reset if completing a circle
            gyroYawAngle-=360.0;
        }
        else if(gyroYawAngle <= -360.0)
        {
            // Reset if completing a circle
            gyroYawAngle+=360.0;
        }

        ///////////////////////////////////////////////////////////////////////////////
        // Yaw angle calculated by the combination of accelerometer and magnetometer //
        ///////////////////////////////////////////////////////////////////////////////
        // float roll = atan2(accelY, (sqrt((accelZ * accelZ) + (accelX * accelX))));
        // float pitch = atan2(-accelX, (sqrt ((accelY * accelY) + (accelY * accelY))));

        // // Calculate the compensation of of compass X and compass Y
        // float Mx_comp = magX * cos(pitch) + magY * sin(roll) * sin(pitch) + magZ * cos(roll) * sin(pitch);
        // float My_comp = magY * cos(roll) - magZ * sin(roll);

        // // Calculate the final yaw angle based on magnetometer and accelerometer
        // float magYawAngle = atan2(-My_comp, Mx_comp) * RAD_TO_DEDGREE * -1.0;
        float magYawAngle = -1.0 * atan2(magX, magY) * RAD_TO_DEDGREE;


        // Calculate the difference between previous and current yaw angle
        float deltaMagYawAngle = magYawAngle - prevMagYawAngle;

        // Do the compensation for deltaMagYawAngle under different conditions
        if (deltaMagYawAngle > 180.0) 
        {
            deltaMagYawAngle -= 360.0;
        }
        if (deltaMagYawAngle < -180.0) 
        {
            deltaMagYawAngle += 360.0;
        }

        // Update the cumulative yaw angle and store the current magnetometer-based yaw angle for the next iteration
        cumulativeMagYawAngle += deltaMagYawAngle;
        prevMagYawAngle = magYawAngle;

        if((gyroYawAngle < 0 && cumulativeMagYawAngle > 0) || (gyroYawAngle > 0 && cumulativeMagYawAngle < 0))
        {   
            cumulativeMagYawAngle *= -1.0;
        }

        /////////////////////////////////////////////////////////////////////////////////////
        // TinyEKF to fuse the above 2 yaw angles gyroZ and (accelerometer + magnetometer) //
        /////////////////////////////////////////////////////////////////////////////////////
        double z[Mobs] = {gyroYawAngle, cumulativeMagYawAngle}; 
        ekf.syncDeltaTime(dt);
        ekf.step(z);

        // Report measured and predicte/fused values
        filteredYawAngle = ekf.getX(0);
    }

    // Update car movement with the desired angle as output from EKF
    if(!isStartMotor) // Only start control the motor under permission
    {
        // Do the distance caculation here

        int centerDistance = 0;
        int rightDistance = 0;

        for (uint8_t idx=0; idx<3; idx++)
        {
            centerDistance += hcsr04Center.measureDistance(true);
            rightDistance += hcsr04Right.measureDistance(true);
        }

        centerDistance/=3;
        rightDistance/=3;

        Serial.print(F("Right distance: "));
        Serial.println(rightDistance);
        Serial.print(F("Center distance: "));
        Serial.println(centerDistance);

        if(centerDistance > 10)
        {
            if(rightDistance > 11)
            {
                // Slight right
                updateCarMovement(filteredYawAngle, carState::STRAIGHT, 150, 50);
            }
            else if(rightDistance  > 8 && rightDistance <= 11)
            {
                // Reset all IMU parameters here 
                resetIMUParameters();
                // Move forward
                updateCarMovement(filteredYawAngle, carState::STRAIGHT, 250, 250);
            }
            else
            {
                updateCarMovement(filteredYawAngle, carState::LEFT, 120, 120);
            }
        }
        else
        {
            // Stop first
            updateCarMovement(filteredYawAngle, carState::STOP, 0, 0);

            if(rightDistance <= 11)
            {
                updateCarMovement(filteredYawAngle, carState::ROTATE_LEFT_90, 150, 150);
            }
            else
            {
                // Slight right
                updateCarMovement(filteredYawAngle, carState::STRAIGHT, 150, 60);
            }
        }
    }
    else
    {
        updateCarMovement(filteredYawAngle, carState::STOP, 0, 0);
    }

    delay(40);
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

    // // Set full scale ranges for both acc and gyr
    // ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

    // myFSS.a = gpm2; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
    //                 // gpm2
    //                 // gpm4
    //                 // gpm8
    //                 // gpm16

    // myFSS.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
    //                 // dps250
    //                 // dps500
    //                 // dps1000
    //                 // dps2000

    // myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
    // if (myICM.status != ICM_20948_Stat_Ok)
    // {
    //     Serial.print(F("setFullScale returned: "));
    //     Serial.println(myICM.statusString());
    // }

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
void resetIMUParameters(void)
{
    gyroYawAngle = 0;

    driftYawAngle = driftHeadingvsNorth();
    prevMagYawAngle = driftYawAngle;
    cumulativeMagYawAngle = 0;
}

void updateCarMovement(float yawAngle, carState direction, uint8_t leftSpeed, uint8_t rightSpeed)
{
    // Analyze the car orientation here to ensure that the car is moving straight
    int leftMotorSpeed = leftSpeed;
    int rightMotorSpeed = rightSpeed;

#ifndef CAR_TESTING
    // Only need to keep the car's orientation if the car is moving forward
    if (direction == carState::STRAIGHT && leftSpeed == rightSpeed)
    {
        // Proportional control constant
        const float Kp = 5;

        // Calculate the speed adjustment based on the error (the lolerance between the curent yaw angle 
        // and the mark at the beginning) and the proportional control constant
        int speedAdjustment = Kp * yawAngle;

        // Limit the speed adjustment to be within the motor's speed range
        speedAdjustment = constrain(speedAdjustment, -carMaxSpeed, carMaxSpeed);

        // Calculate the individual motor speeds
        leftMotorSpeed = carMaxSpeed + speedAdjustment;
        rightMotorSpeed = carMaxSpeed - speedAdjustment;

        // Ensure the motor speeds are within the motor's speed range from carMinSpeed to carMaxSpeed
        leftMotorSpeed = constrain(leftMotorSpeed, carMinSpeed, carMaxSpeed);
        rightMotorSpeed = constrain(rightMotorSpeed, carMinSpeed, carMaxSpeed);

        // Serial.println(leftMotorSpeed);
        // Serial.println(rightMotorSpeed);
    }

    updateServoMotors(direction, leftMotorSpeed, rightMotorSpeed);

#else
    updateServoMotors(direction, carMaxSpeed, carMaxSpeed);
#endif
}

// This function will be called at the startup phase or when the car just change the direction
float driftHeadingvsNorth(void)
{
    uint8_t idx = 0;
    float drift = 0.0;
    float magX = 0.0;
    float magY = 0.0;

    while(1)
    {
        if(myICM.dataReady())
        {
            myICM.getAGMT();

            // Remove hard iron and soft iron here
            magX = (myICM.magX() + magOffsetX) * magScaleX;
            magY = (myICM.magY() + magOffsetY) * magScaleY;

            // Accumulate the drift
            drift += (-1.0 * atan2(magX, magY) * RAD_TO_DEDGREE);

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

// Update the Servo Motor to control car wheels
void updateServoMotors(carState desiredDirection, uint8_t leftSpeed, uint8_t rightSpeed) 
{
    switch (desiredDirection) 
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
            rightMotor->run(FORWARD);
            leftMotor->run(FORWARD);
            rightMotor->setSpeed(rightSpeed);
            leftMotor->setSpeed(leftSpeed);
            break;

        case carState::STOP:
            rightMotor->run(RELEASE);
            leftMotor->run(RELEASE);
            break;

        case carState::ROTATE_LEFT_90:
            rightMotor->run(FORWARD);
            leftMotor->run(BACKWARD);
            rightMotor->setSpeed(rightSpeed);
            leftMotor->setSpeed(leftSpeed);
            delay(450); 
            rightMotor->run(RELEASE);
            leftMotor->run(RELEASE);
            break;

        case carState::ROTATE_RIGHT_90:
            rightMotor->run(BACKWARD);
            leftMotor->run(FORWARD);
            rightMotor->setSpeed(rightSpeed);
            leftMotor->setSpeed(leftSpeed);
            delay(450); 
            rightMotor->run(RELEASE);
            leftMotor->run(RELEASE);
            break;

        default:
            break;
    }
}

