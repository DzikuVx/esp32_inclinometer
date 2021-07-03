#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "QmuTactile.h"
#include "math.h"
#include "types.h"
#include "SSD1306.h"
#include "oled_display.h"

#define MPU6050_UPDATE_TASK_MS 50

#define I2C1_SDA_PIN 4
#define I2C1_SCL_PIN 15

#define I2C2_SDA_PIN 27
#define I2C2_SCL_PIN 26

#define PIN_BUTTON 0

TwoWire I2C1 = TwoWire(0); //OLED bus
TwoWire I2C2 = TwoWire(1); //Gyro bus

imuData_t imu;
Adafruit_MPU6050 mpu;
sensors_event_t acc, gyro, temp;
QmuTactile button(PIN_BUTTON);

SSD1306Wire display(0x3c, &I2C1);
OledDisplay oledDisplay(&display);

TaskHandle_t mpu6050ResourceTask;
TaskHandle_t oledTask;

void sensorCalibrate(struct gyroCalibration_t *cal, float sampleX, float sampleY, float sampleZ, const float dev)
{
    if (cal->state == CALIBARTION_NOT_DONE)
    {
        cal->state = CALIBRATION_IN_PROGRESS;
        cal->sampleCount = 0;
        devClear(&cal->deviation[AXIS_X]);
        devClear(&cal->deviation[AXIS_Y]);
        devClear(&cal->deviation[AXIS_Z]);
        cal->accumulatedValue[AXIS_X] = 0;
        cal->accumulatedValue[AXIS_Y] = 0;
        cal->accumulatedValue[AXIS_Z] = 0;
    }
    if (cal->state == CALIBRATION_IN_PROGRESS)
    {
        cal->sampleCount++;
        devPush(&cal->deviation[AXIS_X], sampleX);
        devPush(&cal->deviation[AXIS_Y], sampleY);
        devPush(&cal->deviation[AXIS_Z], sampleZ);
        cal->accumulatedValue[AXIS_X] += sampleX;
        cal->accumulatedValue[AXIS_Y] += sampleY;
        cal->accumulatedValue[AXIS_Z] += sampleZ;

        if (cal->sampleCount == 40)
        {

            if (
                devStandardDeviation(&cal->deviation[AXIS_X]) > dev ||
                devStandardDeviation(&cal->deviation[AXIS_Y]) > dev ||
                devStandardDeviation(&cal->deviation[AXIS_Z]) > dev)
            {
                cal->sampleCount = 0;
                devClear(&cal->deviation[AXIS_X]);
                devClear(&cal->deviation[AXIS_Y]);
                devClear(&cal->deviation[AXIS_Z]);
                cal->accumulatedValue[AXIS_X] = 0;
                cal->accumulatedValue[AXIS_Y] = 0;
                cal->accumulatedValue[AXIS_Z] = 0;
            }
            else
            {
                cal->zero[AXIS_X] = cal->accumulatedValue[AXIS_X] / cal->sampleCount;
                cal->zero[AXIS_Y] = cal->accumulatedValue[AXIS_Y] / cal->sampleCount;
                cal->zero[AXIS_Z] = cal->accumulatedValue[AXIS_Z] / cal->sampleCount;
                cal->state = CALIBRATION_DONE;
            }
        }
    }
}

void setup()
{
    Serial.begin(115200);

    pinMode(16, OUTPUT);
    digitalWrite(16, LOW); // set GPIO16 low to reset OLED
    delay(50);
    digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 to high

    I2C1.begin(I2C1_SDA_PIN, I2C1_SCL_PIN, 30000);
    I2C2.begin(I2C2_SDA_PIN, I2C2_SCL_PIN, 100000);

    if (!mpu.begin(0x68, &I2C2, 0))
    {
        Serial.println("MPU6050 init fail");
        while (1)
        {
            delay(10);
        }
    }
    Serial.println("MPU6050 init success");

    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

    delay(50);

    oledDisplay.init();
    oledDisplay.setPage(OLED_PAGE_ANGLE);

    delay(50);

    button.start();

    xTaskCreatePinnedToCore(
        mpu6050ResourceTaskHandler, /* Function to implement the task */
        "imuTask",              /* Name of the task */
        10000,                  /* Stack size in words */
        NULL,                   /* Task input parameter */
        0,                      /* Priority of the task */
        &mpu6050ResourceTask,       /* Task handle. */
        0);

    xTaskCreatePinnedToCore(
        oledTaskHandler, /* Function to implement the task */
        "oledTask",  /* Name of the task */
        10000,         /* Stack size in words */
        NULL,          /* Task input parameter */
        0,             /* Priority of the task */
        &oledTask,       /* Task handle. */
        0);
}

void imuSubtask()
{
    static uint32_t prevMicros = 0;
    float dT = (micros() - prevMicros) * 0.000001f;
    prevMicros = micros();

    if (prevMicros > 0)
    {

        mpu.getEvent(&acc, &gyro, &temp);

        imu.accAngle.x = (atan(acc.acceleration.y / sqrt(pow(acc.acceleration.x, 2) + pow(acc.acceleration.z, 2))) * 180 / PI);
        imu.accAngle.y = (atan(-1 * acc.acceleration.x / sqrt(pow(acc.acceleration.y, 2) + pow(acc.acceleration.z, 2))) * 180 / PI);

        imu.accAngle.x -= imu.accAngleZero.x;
        imu.accAngle.y -= imu.accAngleZero.y;

        imu.gyro.x = (gyro.gyro.x * SENSORS_RADS_TO_DPS) - imu.gyroCalibration.zero[AXIS_X];
        imu.gyro.y = (gyro.gyro.y * SENSORS_RADS_TO_DPS) - imu.gyroCalibration.zero[AXIS_Y];
        imu.gyro.z = (gyro.gyro.z * SENSORS_RADS_TO_DPS) - imu.gyroCalibration.zero[AXIS_Z];

        imu.gyroNormalized.x = imu.gyro.x * dT;
        imu.gyroNormalized.y = imu.gyro.y * dT;
        imu.gyroNormalized.z = imu.gyro.z * dT;

        imu.angle.x = (0.95 * (imu.angle.x + imu.gyroNormalized.x)) + (0.05 * imu.accAngle.x);
        imu.angle.y = (0.95 * (imu.angle.y + imu.gyroNormalized.y)) + (0.05 * imu.accAngle.y);
        imu.angle.z = imu.angle.z + imu.gyroNormalized.z;

        /*
         * Calibration Routine
         */
        sensorCalibrate(&imu.gyroCalibration, imu.gyro.x, imu.gyro.y, imu.gyro.z, 3.0f);
    }
}

void mpu6050ResourceTaskHandler(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xPeriod = MPU6050_UPDATE_TASK_MS / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();

    /*
     * MPU6050 and OLED share the same I2C bus
     * To simplify the implementation and do not have to resolve resource conflicts,
     * both tasks are called in one thread pinned to the same core
     */
    for (;;)
    {
        /*
        * Read gyro
        */
        imuSubtask();

        /*
         * Process OLED display
         */
        // oledDisplay.loop();

        // Put task to sleep
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }

    vTaskDelete(NULL);
}

void oledTaskHandler(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xPeriod = 300 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();

    /*
     * MPU6050 and OLED share the same I2C bus
     * To simplify the implementation and do not have to resolve resource conflicts,
     * both tasks are called in one thread pinned to the same core
     */
    for (;;)
    {
        /*
         * Process OLED display
         */
        oledDisplay.loop();

        // Put task to sleep
        vTaskDelayUntil(&xLastWakeTime, xPeriod); //There is a conflict on a I2C due to too much load. Have to put to sleep for a period of time instead
    }

    vTaskDelete(NULL);
}

void loop()
{
	
    button.loop();

    if (button.getState() == TACTILE_STATE_SHORT_PRESS) {
        imu.accAngleZero.x = imu.angle.x;
        imu.accAngleZero.y = imu.angle.y;
        imu.angle.x = 0;
        imu.angle.y = 0;
    }

}
