#include <Arduino.h>               // Arduino library
#include "src_group/dRehmFlight.h" //  Modified and used dRehmFlight: https://github.com/nickrehm/dRehmFlight
                                   //  Credit to: Nicholas Rehm
                                   //  Department of Aerospace Engineering
                                   //  University of Maryland
                                   //  College Park 20742
                                   //  Email: nrehm@umd.edu

#include <Wire.h> // I2C library
#include <SD.h>   // SD card library

const int datalogRate = 50; // The rate at which data is logged to the SD card (in Hz)

// Variables for Flight Control
float timeInMillis;                             // The time in milliseconds since the flight controller has started
int loopCounter = 0;                            // The number of times the loop has run
float pitch_IMU_rad, roll_IMU_rad, yaw_IMU_rad; // The raw pitch, roll, and yaw angles in radians from the IMU
float accelData[3];                             // The raw accelerometer data from the IMU
float gyroData[3];                              // The raw gyro data from the IMU
float yaw_IMU_rad_prev;
float heading_changed_last_loop;

float desiredPivotServo_command_PWM = 90; // The desired sweep angle of the wing in degrees
float pivotServo_command_PWM_float = 90;  // The current sweep angle of the wing in degrees
// Variables for Data Logging
const int COLUMNS = 11;            // 16 columns of data to be logged to the SD card
const int ROWS = 9000;             //  rows of data to be logged to the SD card
float dataLogArray[ROWS][COLUMNS]; // The array that stores the data to be logged to the SD card
boolean dataLogged = false;        // Used to determine if the data has been logged to the SD card
boolean toggle = false;            // Used to toggle the LED
int currentRow = 0;                // The current row of the data log array
boolean logSuccessful = false;     // Used to determine if the data has been successfully logged to the SD card

// Flight Phases
float flight_phase; // The current flight phase
enum flight_phases  // Flight phases for the flight controller
{
    manual_control = 1,
    anti_coupling = 2,
};

File dataFile; // File object for SD card

// Functions
void setupSD();
void logDataToRAM();
void clearDataInRAM();
void writeDataToSD();
void antiCouplingMixing();

//define all the variables in the matrix K
//EDIT THIS FOR TUNING

//phi is roll, theta is pitch, psi is yaw
// a is aileron, e is elevator, r is rudder

const double K_phi_a = 1.0; //KEEP AT 1.0 BECAUSE THIS IS JUST PASSTHROUGH
const double K_theta_a = 0.0;
const double K_psi_a = 0.0;
const double K_phi_e = 0.0;
const double K_theta_e = 1.0; //KEEP AT 1.0 BECAUSE THIS IS JUST PASSTHROUGH
const double K_psi_e = 0.0;
const double K_phi_r = 0.0;
const double K_theta_r = 0.0;
const double K_psi_r = 1.0; //KEEP AT 1.0 BECAUSE THIS IS JUST PASSTHROUGH

//define varibles in P
double P_phi = 0.0;
double P_theta = 0.0;
double P_psi = 0.0;


// define 3x3 gain matrix for coupling
const double K[3][3] = {
    {K_phi_a, K_theta_a, K_psi_a},
    {K_phi_e, K_theta_e, K_psi_e},
    {K_phi_r, K_theta_r, K_psi_r}};

double P[3] = {P_phi, P_theta, P_psi};

// Resultant C vector (3x1)
double C[3];

// Flight Controller Setup
// This function is run once when the flight controller is turned on
// It is used to initialize the flight controller and set the initial values of the variables and objects used in the flight controller loop function (loop())
void setup()
{
    // Constants for PID (no PID control for now...)
    // Kp_roll_angle = 1.0;
    // Ki_roll_angle = 0.3;
    // Kd_roll_angle = 0.2;
    // Kp_pitch_angle = 2.0;
    // Ki_pitch_angle = 0.5;
    // Kd_pitch_angle = 0.4;

    Serial.begin(500000);
    Serial.println("serial works");
    Wire.begin();
    Wire.setClock(1000000);
    Serial.println("wire works");
    pinMode(13, OUTPUT);

    IMUinit();
    Serial.println("passed IMU init");

    aileronServo.attach(aileronServoPin, 900, 2100);
    elevatorServo.attach(elevatorServoPin, 900, 2100);
    rudderServo.attach(rudderServoPin, 900, 2100);
    pivotServo.attach(pivotServoPin, 900, 2100);
    Serial.println("passed attach");
    delay(100);
    radioSetup();
    Serial.println("passed radio setup");

    float AccErrorX = 0.03;
    float AccErrorY = -0.02;
    float AccErrorZ = -1.91;
    float GyroErrorX = 0.05;
    float GyroErrorY = 0.10;
    float GyroErrorZ = -0.36;

    for (int i = 0; i < 1000; i++)
    {
        getIMUdata();
        Madgwick6DOF(GyroX, GyroY, GyroZ, -AccX, AccY, AccZ, dt);
    }

    clearDataInRAM();
    setupSD();

    Serial.println("SD setup complete");

    roll_channel = roll_fs;
    pitch_channel = pitch_fs;
    yaw_channel = yaw_fs;
    pivot_channel = mode1_fs;
    mode2_channel = mode2_fs; // to log data
    delay(100);
    aileronServo.write(90);
    elevatorServo.write(90);
    rudderServo.write(90);
    pivotServo.write(90);
    delay(100);
    calibrateAttitude(); // runs IMU for a few seconds to allow it to stabilize

    Serial.println("setup complete");
}
void loop()
{
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;
    timeInMillis = millis();
    getIMUdata();
    Madgwick6DOF(GyroX, GyroY, GyroZ, -AccX, AccY, AccZ, dt);
    // yaw and pitch are backwrads, so just flip them
    yaw_IMU = -yaw_IMU;
    pitch_IMU = -pitch_IMU;

    accelData[0] = AccX;
    accelData[1] = AccY;
    accelData[2] = AccZ;
    gyroData[0] = GyroX * DEG_TO_RAD;
    gyroData[1] = GyroY * DEG_TO_RAD;
    gyroData[2] = GyroZ * DEG_TO_RAD;

    getCommands();
    // failSafe();

    pitch_IMU_rad = pitch_IMU * DEG_TO_RAD;
    roll_IMU_rad = roll_IMU * DEG_TO_RAD;
    yaw_IMU_rad = yaw_IMU * DEG_TO_RAD;
    getDesState();

    // FLIGHT COMPUTER ACTIVITY
    if (pivot_channel < 1400)
    {
        flight_phase = manual_control;
        desiredPivotServo_command_PWM = 0;

        aileron_command_scaled = roll_passthru;
        elevator_command_scaled = pitch_passthru;
        rudder_command_scaled = yaw_passthru;
        // no pid control for now
        //  integral_pitch = 0;
        //  integral_roll = 0;
        //  roll_PID = 0;
        //  pitch_PID = 0;
    }
    else if (pivot_channel > 1600)
    {
        aileron_command_scaled = roll_passthru;
        elevator_command_scaled = pitch_passthru;
        rudder_command_scaled = yaw_passthru;
        flight_phase = anti_coupling;
        antiCouplingMixing();

        desiredPivotServo_command_PWM = 140;


    }


    // Log data to RAM
    if (loopCounter > (2000 / datalogRate)) // 2000 is the loop rate in microseconds
    {
        logDataToRAM();
        loopCounter = 0;
    }
    else
    {
        loopCounter++;
    }

    // Log data to SD in flight if needed
    if (currentRow >= ROWS)
    {
        writeDataToSD();
        delay(5);
        clearDataInRAM();
    }

    // Log data to SD using switch (for use on the ground only)
    else if (mode2_channel < 1500)
    {
        if (!dataLogged)
        {
            writeDataToSD();
            delay(5);
            clearDataInRAM();
            // blink the LED 3 times
            for (int i = 0; i < 3; i++)
            {
                digitalWrite(13, HIGH);
                delay(100);
                digitalWrite(13, LOW);
                delay(100);
            }
        }
        dataLogged = true;
    }
    else
    {
        dataLogged = false;
    }





    scaleCommands();

    aileronServo.write(aileron_command_PWM);
    elevatorServo.write(elevator_command_PWM);
    rudderServo.write(rudder_command_PWM);
    pivotServo.write(pivotServo_command_PWM);
    loopBlink();
    loopRate(2000);

    // // print roll pitch yaw angles
    // Serial.print(F("Roll: "));
    // Serial.print(roll_IMU);
    // Serial.print(F(" Pitch: "));
    // Serial.print(pitch_IMU);
    // Serial.print(F(" Yaw: "));
    // Serial.println(yaw_IMU);

    // Serial.print(F(" CH1: "));
    // Serial.print(throttle_channel);
    // Serial.print(F(" CH2: "));
    // Serial.print(roll_channel);
    // Serial.print(F(" CH3: "));
    // Serial.print(pitch_channel);
    // Serial.print(F(" CH4: "));
    // Serial.print(yaw_channel);
    // Serial.print(F(" CH5: "));
    // Serial.print(pivot_channel);
    // Serial.print(F(" CH6: "));
    // Serial.println(mode2_channel);
}

void setupSD()
{
    while (!SD.begin(BUILTIN_SDCARD))
    {
        delay(1000);
    }

    // write a line of sample data to the SD card
    dataFile = SD.open("SDtest.txt", FILE_WRITE);
    dataFile.print("TEST DATA");
    dataFile.println();
    dataFile.close();

        // write a line of sample data to the SD card
    dataFile = SD.open("K matrix.txt", FILE_WRITE);
    //print the K matrix to the file, labeling each element
    dataFile.print("K_phi_a: ");
    dataFile.println(K_phi_a);
    dataFile.print("K_theta_a: ");
    dataFile.println(K_theta_a);
    dataFile.print("K_psi_a: ");
    dataFile.println(K_psi_a);
    dataFile.print("K_phi_e: ");
    dataFile.println(K_phi_e);
    dataFile.print("K_theta_e: ");
    dataFile.println(K_theta_e);
    dataFile.print("K_psi_e: ");
    dataFile.println(K_psi_e);
    dataFile.print("K_phi_r: ");
    dataFile.println(K_phi_r);
    dataFile.print("K_theta_r: ");
    dataFile.println(K_theta_r);
    dataFile.print("K_psi_r: ");
    dataFile.println(K_psi_r);
    dataFile.close();

    delay(100);

    // read the line of sample data from the SD card, only continue if it the data is correct
    while (!logSuccessful)
    {
        dataFile = SD.open("SDtest.txt");
        if (dataFile)
        {
            String dataString = dataFile.readStringUntil('\r'); // read the first line of the file
            if (dataString == "TEST DATA")
            {
                Serial.println("SD card initialized correctly");
                logSuccessful = true;
            }
            else
            {
                Serial.println("SD card initialized incorrectly");
                logSuccessful = false;
            }
        }
        else
        {
            Serial.println("SD card failed to open");
            logSuccessful = false;
        }
        dataFile.close();
    }

    // blink LED 10 times to indicate SD card is ready
    for (int i = 0; i < 10; i++)
    {
        digitalWrite(13, HIGH);
        delay(100);
        digitalWrite(13, LOW);
        delay(100);
    }
}

void logDataToRAM()
{
    // log data to RAM

    if (currentRow < ROWS)
    {

        // time and fight phase
        dataLogArray[currentRow][0] = timeInMillis; // time in milliseconds
        dataLogArray[currentRow][1] = flight_phase; // if anticoupling mixing is active

        // roll variables
        dataLogArray[currentRow][2] = GyroX;                    // roll rate
        dataLogArray[currentRow][3] = roll_des;                 // desired roll angle in degrees
        dataLogArray[currentRow][4] = aileron_command_PWM - 90; // aileron command in degrees (90 is neutral)

        // pitch variables
        dataLogArray[currentRow][5] = GyroY;                     // pitch rate
        dataLogArray[currentRow][6] = pitch_des;                 // pilot desired pitch angle in degrees
        dataLogArray[currentRow][7] = elevator_command_PWM - 90; // elevator command in degrees (90 is neutral)

        // yaw
        dataLogArray[currentRow][8] = GyroZ; // yaw rate
        dataLogArray[currentRow][9] = yaw_des;
        dataLogArray[currentRow][10] = rudder_command_PWM - 90; // rudder command in degrees (90 is neutral)

        currentRow++;
    }
}

void writeDataToSD()
{
    dataFile = SD.open("flightData.txt", FILE_WRITE);
    for (int i = 0; i < currentRow; i++)
    {
        for (int j = 0; j < COLUMNS; j++)
        {
            dataFile.print(dataLogArray[i][j]);
            dataFile.print(",");
        }
        dataFile.println();
    }
    dataFile.close();
}

void clearDataInRAM()
{
    for (int i = 0; i < ROWS; i++)
    {
        for (int j = 0; j < COLUMNS; j++)
        {
            dataLogArray[i][j] = 0.0;
        }
    }
    currentRow = 0;
}

void antiCouplingMixing()
{
    // multipy gyroData matrix, a 1x3 matrix, by the coupling matrix, a 3x3 matrix

    P[0] = aileron_command_scaled; 
    P[1] = elevator_command_scaled;
    P[2] = rudder_command_scaled;

    // Perform the matrix multiplication
    for (int i = 0; i < 3; i++)
    {
        C[i] = 0; // Initialize the result array with zero
        for (int j = 0; j < 3; j++)
        {
            C[i] += K[i][j] * P[j];
        }
    }

    aileron_command_scaled = C[0];
    elevator_command_scaled = C[1];
    rudder_command_scaled  = C[2];

}