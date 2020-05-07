/*
 Name:		Ardu_Copter.ino
 Created:	5/2/2020 8:49:29 PM
 Author:	gferna14
*/

#include <Wire.h>   

#define LED_BUILTIN 2

// RC controller input channel
byte receiver_input_row_pin = 4;
byte receiver_input_pitch_pin = 5;
byte receiver_input_yaw_pin = 2;    //set
byte receiver_input_throttle_pin = 3; //set

byte last_row_channel, last_pitch_channel, last_yaw_channel, last_throttle_channel;         //keeps track if wave high or low
unsigned long timer_row, timer_pitch, timer_yaw, timer_throttle, current_time;                            //keeps track of ms
int receiver_row_input, receiver_pitch_input, receiver_yaw_input, receiver_throttle_input;  //actual input in ms {1000,2000}

//MPU & IMU address and variables
int gyro_address = 0x68;

int16_t temperature;
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal= 0;
int cal_int = 0;                            //used for gyro calibration
int level_calibration_on = 0;               // 1 when level calibrating, 0 else
int16_t acc_pitch_cal_value, acc_roll_cal_value;

long acc_x, acc_y, acc_z, acc_total_vector;

double gyro_pitch, gyro_roll, gyro_yaw;
float gyro_roll_input, gyro_pitch_input, gyro_yaw_input;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;

//PID variables


//Loop timing
unsigned long loop_timer;

// the setup function runs once when you press reset or power the board
void setup() {
    Serial.begin(115200);

    Wire.begin();
    
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, 1);

    Serial.println("MPU setup");
    mpu_setup();

    delay(1000);
    Serial.println("gyro cal");
    mpu_calibrate_gyro();

    Serial.println("level cal");
    mpu_calibrate_level();

    receiver_setup();

    loop_timer = micros();                                                    //Set the timer for the next loop.

    digitalWrite(LED_BUILTIN, LOW);
}

// the loop function runs over and over again forever
void loop() {     

    mpu_read();
    imu_calc_orientation();
    

    Serial.print("row: ");
    Serial.print(receiver_row_input);
    Serial.print("- pitch: ");
    Serial.print(receiver_pitch_input);
    Serial.print("- yaw: ");
    Serial.print(receiver_yaw_input);
    Serial.print("- throttle: ");
    Serial.println(receiver_throttle_input);

    if (micros() - loop_timer > 4050)digitalWrite(12, HIGH);
    //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
    while (micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
    loop_timer = micros();                                                    //Set the timer for the next loop.

    mpu_read();
}

void receiver_setup() {
    
}

// ISR(PCINT2_vect) {
//         current_time = micros();

//         //row input
//         if (digitalRead(receiver_input_row_pin) == HIGH) {
//             if (last_row_channel == 0) {
//                 last_row_channel = 1;
//                 timer_row = current_time;
//             }
//         }
//         else if (last_row_channel == 1) {
//             last_row_channel = 0;
//             receiver_row_input = current_time - timer_row;
//         }

//         //pitch input
//         if (digitalRead(receiver_input_pitch_pin) == HIGH) {
//             if (last_pitch_channel == 0) {
//                 last_pitch_channel = 1;
//                 timer_pitch = current_time;
//             }
//         }
//         else if (last_pitch_channel == 1) {
//             last_pitch_channel = 0;
//             receiver_pitch_input = current_time - timer_pitch;
//         }

//         //row yaw
//         if (digitalRead(receiver_input_yaw_pin) == HIGH) {
//             if (last_yaw_channel == 0) {
//                 last_yaw_channel = 1;
//                 timer_yaw = current_time;
//             }
//         }
//         else if (last_yaw_channel == 1) {
//             last_yaw_channel = 0;
//             receiver_yaw_input = current_time - timer_yaw;
//         }

//         //row throttle
//         if (digitalRead(receiver_input_throttle_pin) == HIGH) {
//             if (last_throttle_channel == 0) {
//                 last_throttle_channel = 1;
//                 timer_throttle = current_time;
//             }
//         }
//         else if (last_throttle_channel == 1) {
//             last_throttle_channel = 0;
//             receiver_throttle_input = current_time - timer_throttle;
//         }
//     }

void mpu_setup() {
    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x6B);                                                          //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                                                    //End the transmission with the gyro.

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search
    Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
    Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();                                                    //End the transmission with the gyro    
}

void mpu_calibrate_level(void) {
    level_calibration_on = 1;

    acc_pitch_cal_value = 0;
    acc_roll_cal_value = 0;

    int error;
    for (error = 0; error < 64; error++) {                                              //Send telemetry data to the ground station.
        mpu_read();
        acc_pitch_cal_value += acc_y;
        acc_roll_cal_value += acc_x;
        if (acc_y > 500 || acc_y < -500)error = 80;
        if (acc_x > 500 || acc_x < -500)error = 80;
        delayMicroseconds(3700);
    }

    acc_pitch_cal_value /= 64;
    acc_roll_cal_value /= 64;

    level_calibration_on = 0;
    mpu_read();
    //Accelerometer angle calculations
    acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));    //Calculate the total accelerometer vector.

    if (abs(acc_y) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
        angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;              //Calculate the pitch angle.
    }
    if (abs(acc_x) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
        angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;               //Calculate the roll angle.
    }
    angle_pitch = angle_pitch_acc;                                                   //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;
    loop_timer = micros();                                                           //Set the timer for the next loop.
}



void mpu_calibrate_gyro() {
    cal_int = 0;
    //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
    for (cal_int = 0; cal_int < 2000; cal_int++) {                           //Take 2000 readings for calibration.
        mpu_read();                                                        //Read the gyro output.
        gyro_roll_cal += gyro_roll;                                       //Ad roll value to gyro_roll_cal.
        gyro_pitch_cal += gyro_pitch;                                       //Ad pitch value to gyro_pitch_cal.
        gyro_yaw_cal += gyro_yaw;                                       //Ad yaw value to gyro_yaw_cal.
    }

    //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
    gyro_roll_cal /= 2000;                                                 //Divide the roll total by 2000.
    gyro_pitch_cal /= 2000;                                                 //Divide the pitch total by 2000.
    gyro_yaw_cal /= 2000;                                                 //Divide the yaw total by 2000.

}

void mpu_read() {
    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro.
    Wire.write(0x3B);                                                       //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                                 //End the transmission.
    Wire.requestFrom(gyro_address, 14);                                      //Request 14 bytes from the gyro.

    while (Wire.available() < 14);                                           //Wait until the 14 bytes are received.
    acc_y = Wire.read() << 8 | Wire.read();                               //Add the low and high byte to the acc_x variable.
    acc_x = Wire.read() << 8 | Wire.read();                               //Add the low and high byte to the acc_y variable.
    acc_z = Wire.read() << 8 | Wire.read();                               //Add the low and high byte to the acc_z variable.
    temperature = Wire.read() << 8 | Wire.read();                               //Add the low and high byte to the temperature variable.
    gyro_roll = Wire.read() << 8 | Wire.read();                              //Read high and low part of the angular data.
    gyro_pitch = Wire.read() << 8 | Wire.read();                              //Read high and low part of the angular data.
    gyro_yaw = Wire.read() << 8 | Wire.read();


    if (level_calibration_on == 0) {
        acc_y -= acc_pitch_cal_value;                              //Subtact the manual accelerometer pitch calibration value.
        acc_x -= acc_roll_cal_value;                               //Subtact the manual accelerometer roll calibration value.
    }

    if (cal_int == 2000) {
        gyro_roll -= gyro_roll_cal;                                       //Only compensate after the calibration.
        gyro_pitch -= gyro_pitch_cal;                                       //Only compensate after the calibration.
        gyro_yaw -= gyro_yaw_cal;                                       //Only compensate after the calibration.
    }
}

void imu_calc_orientation() {

    //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
    gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
    gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
    gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

    //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
    angle_pitch += gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
    angle_roll += gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.

    //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
    angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
    angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

    //Accelerometer angle calculations
    acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));       //Calculate the total accelerometer vector.

    if (abs(acc_y) < acc_total_vector) {                                        //Prevent the asin function to produce a NaN
        angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;          //Calculate the pitch angle.
    }
    if (abs(acc_x) < acc_total_vector) {                                        //Prevent the asin function to produce a NaN
        angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;          //Calculate the roll angle.
    }

    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

}
