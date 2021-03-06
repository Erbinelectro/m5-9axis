#define M5STACK_MPU6886

#include <Arduino.h>
#include "Preferences.h"
#include "M5Stack.h"
#include "math.h"
#include "utility/MPU6886.h"
#include "M5_BMM150.h"
#include "M5_BMM150_DEFS.h"
#include "MadgwickAHRS.h"

#define M_PI 3.141592

// for magnet
Preferences prefs;
struct bmm150_dev dev;
bmm150_mag_data mag_offset;
bmm150_mag_data mag_max;
bmm150_mag_data mag_min;

const float sampleFreq = 1000.0f;// bmm150 ssample code was setted 512Hz

// for imu
float accX[2] = {};  //[0] -> comtemporary data box, [1] -> old data box
float accY[2] = {};
float accZ[2] = {};

float gyroX[2] = {};
float gyroY[2] = {};
float gyroZ[2] = {};

float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;

float temp = 0.0F;

// for madgwick filter
Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

// for RC low pass filter
float a = 0.01;//1/(2 * M_PI * sampleFreq * (1 / sampleFreq) + 1);

int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *read_data, uint16_t len){
    if(M5.I2C.readBytes(dev_id, reg_addr, len, read_data)){ // Check whether the device ID, address, data exist.
        return BMM150_OK;                                   //判断器件的Id、地址、数据是否存在
    }else{
        return BMM150_E_DEV_NOT_FOUND;
    }
}

int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *read_data, uint16_t len){
    if(M5.I2C.writeBytes(dev_id, reg_addr, read_data, len)){    //Writes data of length len to the specified device address.
        return BMM150_OK;                                       //向指定器件地址写入长度为len的数据
    }else{
        return BMM150_E_DEV_NOT_FOUND;
    }
}

int8_t bmm150_initialization(){
    int8_t rslt = BMM150_OK;

    dev.dev_id = 0x10;  //Device address setting.  设备地址设置
    dev.intf = BMM150_I2C_INTF; //SPI or I2C interface setup.  SPI或I2C接口设置
    dev.read = i2c_read;    //Read the bus pointer.  读总线指针
    dev.write = i2c_write;  //Write the bus pointer.  写总线指针
    dev.delay_ms = delay;

    // Set the maximum range range
    //设置最大范围区间
    mag_max.x = -2000;
    mag_max.y = -2000;
    mag_max.z = -2000;

    // Set the minimum range
    //设置最小范围区间
    mag_min.x = 2000;
    mag_min.y = 2000;
    mag_min.z = 2000;

    rslt = bmm150_init(&dev);   //Memory chip ID.  存储芯片ID
    dev.settings.pwr_mode = BMM150_NORMAL_MODE;
    rslt |= bmm150_set_op_mode(&dev);   //Set the sensor power mode.  设置传感器电源工作模式
    dev.settings.preset_mode = BMM150_PRESETMODE_ENHANCED;
    rslt |= bmm150_set_presetmode(&dev);    //Set the preset mode of .  设置传感器的预置模式
    return rslt;
}

void bmm150_offset_save(){
    prefs.begin("bmm150", false);
    prefs.putBytes("offset", (uint8_t *)&mag_offset, sizeof(bmm150_mag_data));
    prefs.end();
}

void bmm150_offset_load(){
    if(prefs.begin("bmm150", true)){
        prefs.getBytes("offset", (uint8_t *)&mag_offset, sizeof(bmm150_mag_data));
        prefs.end();
        Serial.println("bmm150 load offset finish....");
    }else{
        Serial.println("bmm150 load offset failed....");
    }
}

void bmm150_calibrate(uint32_t calibrate_time){
    uint32_t calibrate_timeout = 0;

    calibrate_timeout = millis() + calibrate_time;
    Serial.printf("Go calibrate, use %d ms \r\n", calibrate_time);
    Serial.printf("running ...");

    while (calibrate_timeout > millis()){
        bmm150_read_mag_data(&dev);
        if(dev.data.x){
            mag_min.x = (dev.data.x < mag_min.x) ? dev.data.x : mag_min.x;
            mag_max.x = (dev.data.x > mag_max.x) ? dev.data.x : mag_max.x;
        }
        if(dev.data.y){
            mag_max.y = (dev.data.y > mag_max.y) ? dev.data.y : mag_max.y;
            mag_min.y = (dev.data.y < mag_min.y) ? dev.data.y : mag_min.y;
        }
        if(dev.data.z){
            mag_min.z = (dev.data.z < mag_min.z) ? dev.data.z : mag_min.z;
            mag_max.z = (dev.data.z > mag_max.z) ? dev.data.z : mag_max.z;
        }
        delay(100);
    }

    mag_offset.x = (mag_max.x + mag_min.x) / 2;
    mag_offset.y = (mag_max.y + mag_min.y) / 2;
    mag_offset.z = (mag_max.z + mag_min.z) / 2;
    bmm150_offset_save();

    Serial.printf("\n calibrate finish ... \r\n");
    Serial.printf("mag_max.x: %.2f x_min: %.2f \t", mag_max.x, mag_min.x);
    Serial.printf("y_max: %.2f y_min: %.2f \t", mag_max.y, mag_min.y);
    Serial.printf("z_max: %.2f z_min: %.2f \r\n", mag_max.z, mag_min.z);
}

void setup() {

    M5.begin(true, false, true, false);
    M5.Power.begin();
    M5.IMU.Init();
    Wire.begin(21, 22, 400000);

    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(GREEN , BLACK);
    M5.Lcd.setTextSize(2);

    if (M5.IMU.Init() != 0) {
        M5.Lcd.print("cannnot find MPU6886");
        exit(0);
    }
    Serial.printf("0x%x\r\n",M5.IMU.imuId);
    
    if(bmm150_initialization() != BMM150_OK){
        M5.Lcd.print("BMM150 init failed");
        exit(0);
    }

    bmm150_offset_load();
  
    filter.begin(sampleFreq);
    microsPerReading = 1000000 / sampleFreq;
    microsPrevious = micros();
}

void loop() {
    char text_string[100];
    float magX, magY, magZ;

    M5.update();
    bmm150_read_mag_data(&dev);
    magX = dev.data.x -  mag_offset.x;
    magY = dev.data.y - mag_offset.y;
    magZ = dev.data.z -  mag_offset.z;
    float head_dir = atan2(magX, magY) * 180.0 / M_PI;
    //Serial.printf("MID X : %.2f \t MID Y : %.2f \t MID Z : %.2f \n", mag_offset.x, mag_offset.y, mag_offset.z);

    M5.IMU.getGyroData(&gyroX[0],&gyroY[0],&gyroZ[0]);
    M5.IMU.getAccelData(&accX[0],&accY[0],&accZ[0]);
    M5.IMU.getAhrsData(&pitch,&roll,&yaw);
    M5.IMU.getTempData(&temp);

    /*
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.printf("MAG X: %.2f", dev.data.x);
    M5.Lcd.setCursor(0, 15);
    M5.Lcd.printf("MAG Y: %.2f", dev.data.y);
    M5.Lcd.setCursor(0, 30);
    M5.Lcd.printf("MAG Z: %.2f", dev.data.z);
    M5.Lcd.setCursor(0, 45);
    M5.Lcd.printf("HEAD Angle: %.2f", head_dir);
    */
    M5.Lcd.setCursor(0, 150);
    M5.Lcd.print("Press BtnA enter calibrate");
    
    if(M5.BtnA.wasPressed()){
        M5.Lcd.fillScreen(BLACK);
        M5.Lcd.setCursor(0, 110);
        M5.Lcd.print("Flip + rotate core calibration");
        
        bmm150_calibrate(10000);

        M5.Lcd.fillScreen(BLACK);
    }

    if (micros() - microsPrevious >= microsPerReading)
    {
        gyroX[0] = a * gyroX[1] + (1-a) * gyroX[0];
        gyroY[0] = a * gyroY[1] + (1-a) * gyroY[0];
        gyroZ[0] = a * gyroZ[1] + (1-a) * gyroZ[0];

        accX[0] = a * accX[1] + (1-a) * accX[0];
        accY[0] = a * accY[1] + (1-a) * accY[0];
        accZ[0] = a * accZ[1] + (1-a) * accZ[0];

        filter.update(gyroX[0], gyroY[0], gyroZ[0], accX[0], accY[0], accZ[0], magX, magY, magZ);

        roll    = filter.getRoll();
        pitch   = filter.getPitch();
        yaw     = filter.getYaw();

        M5.Lcd.setCursor(0, 0);
        M5.Lcd.printf("ROLL  : %.2f", roll);
        M5.Lcd.setCursor(0, 15);
        M5.Lcd.printf("PITCH : %.2f", pitch);
        M5.Lcd.setCursor(0, 30);
        M5.Lcd.printf("YAW   : %.2f", yaw);

        microsPrevious += microsPerReading;
    }

    delay(1000.0f / sampleFreq);
}