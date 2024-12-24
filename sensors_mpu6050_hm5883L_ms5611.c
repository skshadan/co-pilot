/**
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011-2018 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Implements HAL for sensors MPU9250 and LPS25H
 *
 * 2016.06.15: Initial version by Mike Hamer, http://mikehamer.info
 */
#include <math.h>
#define LPS25H_LSB_PER_MBAR      4096UL
#define LPS25H_LSB_PER_CELSIUS   480UL
#define LPS25H_TEMP_OFFSET        (42.5f)

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/projdefs.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include "sensors_mpu6050_hm5883L_ms5611.h"
#include "system.h"
#include "configblock.h"
#include "param.h"
#include "log.h"

#include "imu.h"
#include "nvicconf.h"
#include "ledseq.h"
#include "sound.h"
#include "filter.h"
#include "config.h"
#include "stm32_legacy.h"
#include "bmp280.h"
#include "i2cdev.h"
// #include "lps25h.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "ms5611.h"
// #include "ak8963.h"
#include "zranger.h"
#include "zranger2.h"
#include "vl53l1x.h"
#include "flowdeck_v1v2.h"
#define DEBUG_MODULE "SENSORS"
#include "debug_cf.h"
#include "static_mem.h"
#include "crtp_commander.h"
#include "bmi270.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <bmi2.h>
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_system.h"
//#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h" 

/**
 * Enable 250Hz digital LPF mode. However does not work with
 * multiple slave reading through MPU9250 (MAG and BARO), only single for some reason.
 */
//#define SENSORS_mpu6050_DLPF_256HZ

//#define GYRO_ADD_RAW_AND_VARIANCE_LOG_VALUES

#define MAG_GAUSS_PER_LSB 666.7f

/**
 * Enable sensors on board 
 */
//#define SENSORS_ENABLE_MAG_HM5883L
//#define SENSORS_ENABLE_PRESSURE_BMP280
//#define SENSORS_ENABLE_PRESSURE_MS5611
//#define SENSORS_ENABLE_RANGE_VL53L0X
//#define SENSORS_ENABLE_RANGE_VL53L1X
//#define SENSORS_ENABLE_FLOW_PMW3901



// #define SENSORS_ACCEL_FS_CFG MPU6050_ACCEL_FS_16
// #define SENSORS_G_PER_LSB_CFG MPU6050_G_PER_LSB_16

#define SENSORS_ACCEL_FS_CFG BMI2_ACC_RANGE_16G
#define SENSORS_GYRO_FS_CFG BMI2_GYR_RANGE_2000
#define SENSORS_DEG_PER_LSB_CFG    (2000.0f/32768.0f)  // For BMI270 in 2000dps mode
#define SENSORS_G_PER_LSB_CFG    (16.0f/32768.0f)  // 16G full scale


#define SENSORS_VARIANCE_MAN_TEST_TIMEOUT M2T(2000) // Timeout in ms
#define SENSORS_MAN_TEST_LEVEL_MAX 5.0f             // Max degrees off

#define SENSORS_BIAS_SAMPLES 1000
#define SENSORS_ACC_SCALE_SAMPLES 200
#define SENSORS_GYRO_BIAS_CALCULATE_STDDEV

// Buffer length for MPU9250 slave reads
#define GPIO_INTA_MPU6050_IO CONFIG_MPU_PIN_INT
#define SENSORS_MPU6050_BUFF_LEN 14
#define SENSORS_MAG_BUFF_LEN 8
#define SENSORS_BARO_BUFF_S_P_LEN MS5611_D1D2_SIZE
#define SENSORS_BARO_BUFF_T_LEN MS5611_D1D2_SIZE
#define SENSORS_BARO_BUFF_LEN (SENSORS_BARO_BUFF_S_P_LEN + SENSORS_BARO_BUFF_T_LEN)

#define GYRO_NBR_OF_AXES 3
#define GYRO_MIN_BIAS_TIMEOUT_MS M2T(1 * 1000)
// Number of samples used in variance calculation. Changing this effects the threshold
#define SENSORS_NBR_OF_BIAS_SAMPLES 1024
// Variance threshold to take zero bias for gyro
#define GYRO_VARIANCE_BASE 5000
#define GYRO_VARIANCE_THRESHOLD_X (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z (GYRO_VARIANCE_BASE)
#define ESP_INTR_FLAG_DEFAULT 0

#define PITCH_CALIB (CONFIG_PITCH_CALIB*1.0/100)
#define ROLL_CALIB (CONFIG_ROLL_CALIB*1.0/100)


// FOR BMI
// Replace MPU6050 definitions with BMI270
// #define SENSORS_BMI270_DLPF_256HZ
// #define SENSORS_GYRO_FS_CFG BMI2_GYR_RANGE_2000
// #define SENSORS_ACCEL_FS_CFG BMI2_ACC_RANGE_16G

#define CONFIG_BMI270_PIN_INT CONFIG_MPU_PIN_INT  // Define interrupt pin




typedef struct {
    Axis3f bias;
    Axis3f variance;
    Axis3f mean;
    bool isBiasValueFound;
    bool isBufferFilled;
    Axis3i16 *bufHead;
    Axis3i16 buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
} BiasObj;

bmp280_data_t bmp280Data;
// At the top of your file with other global variables
static struct bmi2_dev bmi2_dev;

static xQueueHandle accelerometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(accelerometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle gyroDataQueue;
STATIC_MEM_QUEUE_ALLOC(gyroDataQueue, 1, sizeof(Axis3f));
static xQueueHandle magnetometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(magnetometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle barometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(barometerDataQueue, 1, sizeof(baro_t));

static xSemaphoreHandle sensorsDataReady;
static StaticSemaphore_t sensorsDataReadyBuffer;
static xSemaphoreHandle dataReady;
static StaticSemaphore_t dataReadyBuffer;

static bool isInit = false;
static sensorData_t sensorData;
static volatile uint64_t imuIntTimestamp;

static Axis3i16 gyroRaw;
static Axis3i16 accelRaw;
static BiasObj gyroBiasRunning;
static Axis3f gyroBias;
#if defined(SENSORS_GYRO_BIAS_CALCULATE_STDDEV) && defined(GYRO_BIAS_LIGHT_WEIGHT)
static Axis3f gyroBiasStdDev;
#endif
static bool gyroBiasFound = false;
static float accScaleSum = 0;
static float accScale = 1;

// Low Pass filtering
#define GYRO_LPF_CUTOFF_FREQ 80
#define ACCEL_LPF_CUTOFF_FREQ 30
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];
static void applyAxis3fLpf(lpf2pData *data, Axis3f *in);

static bool isBarometerPresent = false;
static bool isMagnetometerPresent = false;
static bool isVl53l1xPresent = false;
static bool isVl53l0xPresent = false;
static bool isPmw3901Present = false;
static bool isBmp280TestPassed = false;
static bool isMpu6050TestPassed = false;
static bool isHmc5883lTestPassed = false;
static bool isMs5611TestPassed = false;
static bool isVl53l1xTestPassed = false;
static bool isPmw3901TestPassed = false;
static bool isBmi270TestPassed = false;


// Pre-calculated values for accelerometer alignment
float cosPitch;
float sinPitch;
float cosRoll;
float sinRoll;

// This buffer needs to hold data from all sensors
static uint8_t buffer[SENSORS_MPU6050_BUFF_LEN + SENSORS_MAG_BUFF_LEN + SENSORS_BARO_BUFF_LEN] = {0};

static void processAccGyroMeasurements(void);
static void processMagnetometerMeasurements(const uint8_t *buffer);
static void processBarometerMeasurements(const uint8_t *buffer);
//static void sensorsSetupSlaveRead(void);

#ifdef GYRO_BIAS_LIGHT_WEIGHT  
static bool processGyroBiasNoBuffer(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);
#else
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut); 
#endif


static bool processAccScale(int16_t ax, int16_t ay, int16_t az);
static void sensorsBiasObjInit(BiasObj *bias);
static void sensorsCalculateVarianceAndMean(BiasObj *bias, Axis3f *varOut, Axis3f *meanOut);
static void sensorsCalculateBiasMean(BiasObj *bias, Axis3i32 *meanOut);
static void sensorsAddBiasValue(BiasObj *bias, int16_t x, int16_t y, int16_t z);
static bool sensorsFindBiasValue(BiasObj *bias);
static void sensorsAccAlignToGravity(Axis3f *in, Axis3f *out);
static bool bmi270SelfTest(void);  // Function prototype


STATIC_MEM_TASK_ALLOC(sensorsTask, SENSORS_TASK_STACKSIZE);
bool sensorsMpu6050Hmc5883lMs5611ReadGyro(Axis3f *gyro)
{
    return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}

bool sensorsMpu6050Hmc5883lMs5611ReadAcc(Axis3f *acc)
{
    return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}

bool sensorsMpu6050Hmc5883lMs5611ReadMag(Axis3f *mag)
{
    return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}

bool sensorsMpu6050Hmc5883lMs5611ReadBaro(baro_t *baro)
{
    return (pdTRUE == xQueueReceive(barometerDataQueue, baro, 0));
}

void sensorsMpu6050Hmc5883lMs5611Acquire(sensorData_t *sensors, const uint32_t tick)
{   DEBUG_PRINTI("sensorsMpu6050Hmc5883lMs5611Acquire tick");
    sensorsReadGyro(&sensors->gyro);
    sensorsReadAcc(&sensors->acc);
    sensorsReadMag(&sensors->mag);
    sensorsReadBaro(&sensors->baro);
    sensors->interruptTimestamp = sensorData.interruptTimestamp;
    DEBUG_PRINTI("Acquire complete\n");
}

// bool sensorsMpu6050Hmc5883lMs5611AreCalibrated(void) {
//     // ... existing code ...
//     return gyroBiasFound; // Dummy return for testing
// }

bool sensorsMpu6050Hmc5883lMs5611AreCalibrated(void) {
    // Restore proper calibration check
    //DEBUG_PRINTI("Calibration status: %d\n", gyroBiasFound);
    return gyroBiasFound; // Return actual calibration status instead of dummy true
}




static void sensorsTask(void *param)
{   
    DEBUG_PRINTI("Sensors task starting...");
    //TODO:
    systemWaitStart();
    vTaskDelay(M2T(200));
    DEBUG_PRINTD("xTaskCreate sensorsTask IN");
    // sensorsSetupSlaveRead(); //
    //DEBUG_PRINTD("xTaskCreate sensorsTask SetupSlave done");

    while (1) {
        //DEBUG_PRINTI("I AM INSIDE SENSOR TASK!!!!!!!");
        // /* mpu6050 interrupt trigger: data is ready to be read */
         if (pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY)) {
            sensorData.interruptTimestamp = imuIntTimestamp;
            
            // Process BMI270 data
            processAccGyroMeasurements();

            if (isMagnetometerPresent) {
                processMagnetometerMeasurements(&(buffer[SENSORS_MPU6050_BUFF_LEN]));
            }

            if (isBarometerPresent) {
                processBarometerMeasurements(&(buffer[isMagnetometerPresent ? SENSORS_MPU6050_BUFF_LEN + SENSORS_MAG_BUFF_LEN : SENSORS_MPU6050_BUFF_LEN]));
            }

            /* sensors step 3- queue sensors data  on the output queues */
            xQueueOverwrite(accelerometerDataQueue, &sensorData.acc);
            xQueueOverwrite(gyroDataQueue, &sensorData.gyro);

            if (isMagnetometerPresent) {
                xQueueOverwrite(magnetometerDataQueue, &sensorData.mag);
            }

            if (isBarometerPresent) {
                xQueueOverwrite(barometerDataQueue, &sensorData.baro);
            }

            /* sensors step 4- Unlock stabilizer task */
            xSemaphoreGive(dataReady);
#ifdef DEBUG_EP2
            DEBUG_PRINT_LOCAL("ax = %f,  ay = %f,  az = %f,  gx = %f,  gy = %f,  gz = %f , hx = %f , hy = %f, hz =%f \n", sensorData.acc.x, sensorData.acc.y, sensorData.acc.z, sensorData.gyro.x, sensorData.gyro.y, sensorData.gyro.z, sensorData.mag.x, sensorData.mag.y, sensorData.mag.z);
#endif
        }
    }
}

void sensorsMpu6050Hmc5883lMs5611WaitDataReady(void)
{
    xSemaphoreTake(dataReady, portMAX_DELAY);
}

#define CONVERSION_TIME_MS 10  // Adjusted conversion time for both
static uint8_t readState = 0;
static uint32_t lastConv = 0;

void processBarometerMeasurements(const uint8_t *buffer)
{
    static int32_t rawPressure = 0, rawTemp = 0, deltaT = 0;
    uint32_t now = xTaskGetTickCount();  // Get the current time

    // Check if enough time has passed for conversion
    if ((now - lastConv) < pdMS_TO_TICKS(CONVERSION_TIME_MS)) {
        return;
    }

    lastConv = now;  // Update the last conversion time

    if (readState == 0) {
        // Step 1: Read raw temperature and pressure data
        bmp280ReadRawData(&rawPressure, &rawTemp);  // Read both in one go
        deltaT = bmp280CompensateTemp(rawTemp);     // Compensate temperature
        sensorData.baro.temperature = (float)deltaT / 100.0f;  // Compensated temp

        // Debug raw temperature data
        //DEBUG_PRINTI("Raw Temp: %d, Raw Pressure: %d", rawTemp, rawPressure);

        // Move to pressure read state
        readState = 1;
    } else {
        // Step 2: Calculate pressure from saved raw data
        sensorData.baro.pressure = bmp280CompensatePressure(rawPressure, deltaT);

        // Calculate altitude based on the compensated pressure
        sensorData.baro.asl = bmp280PressureToAltitude(sensorData.baro.pressure / 100.0f);  // Convert to hPa

        // Debugging - Print results
        //DEBUG_PRINTI("Temp: %f°C, Pressure: %f Pa, Altitude: %f m", sensorData.baro.temperature, sensorData.baro.pressure, sensorData.baro.asl);

        // Reset back to temperature read state
        readState = 0;
    }
}

float getBaroAltitude(void) {
    return sensorData.baro.asl;
}


void processMagnetometerMeasurements(const uint8_t *buffer)
{
    //TODO: replace it to hmc5883l
    if (buffer[7] & (1 << HMC5883L_STATUS_READY_BIT)) {
        int16_t headingx = (((int16_t)buffer[2]) << 8) | buffer[1]; //hmc5883 different from
        int16_t headingz = (((int16_t)buffer[4]) << 8) | buffer[3];
        int16_t headingy = (((int16_t)buffer[6]) << 8) | buffer[5];

        sensorData.mag.x = (float)headingx / MAG_GAUSS_PER_LSB; //to gauss
        sensorData.mag.y = (float)headingy / MAG_GAUSS_PER_LSB;
        sensorData.mag.z = (float)headingz / MAG_GAUSS_PER_LSB;
        DEBUG_PRINTI("hmc5883l DATA ready");
    } else {

        DEBUG_PRINTW("hmc5883l DATA not ready");
    }
}


void processAccGyroMeasurements(void)
{
    Axis3f accScaled;
    struct bmi2_sens_data sensor_data;
    int8_t rslt = bmi2_get_sensor_data(&sensor_data, &bmi2_dev);
    
    if (rslt == BMI2_OK) {
        // Convert BMI270 data to raw format for compatibility
        accelRaw.x = sensor_data.acc.x;
        accelRaw.y = sensor_data.acc.y;
        accelRaw.z = sensor_data.acc.z;
        
        gyroRaw.x = sensor_data.gyr.x;
        gyroRaw.y = sensor_data.gyr.y;
        gyroRaw.z = sensor_data.gyr.z;

        // Process gyro bias
        #ifdef GYRO_BIAS_LIGHT_WEIGHT
            gyroBiasFound = processGyroBiasNoBuffer(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
        #else
            gyroBiasFound = processGyroBias(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
        #endif

        // Process accelerometer scale when gyro bias is found
        if (gyroBiasFound) {
            processAccScale(accelRaw.x, accelRaw.y, accelRaw.z);
        }

        // Convert gyro readings to degrees per second
        #ifdef CONFIG_TARGET_ESPLANE_V1
            sensorData.gyro.x = (gyroRaw.x - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;
        #else
            sensorData.gyro.x = -(gyroRaw.x - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;
        #endif
        
        sensorData.gyro.y = (gyroRaw.y - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
        sensorData.gyro.z = (gyroRaw.z - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;

        // Apply low-pass filter to gyro data
        applyAxis3fLpf((lpf2pData *)(&gyroLpf), &sensorData.gyro);

        // Convert accelerometer readings to G's
        #ifdef CONFIG_TARGET_ESPLANE_V1
            accScaled.x = (accelRaw.x) * SENSORS_G_PER_LSB_CFG / accScale;
        #else
            accScaled.x = -(accelRaw.x) * SENSORS_G_PER_LSB_CFG / accScale;
        #endif

        accScaled.y = (accelRaw.y) * SENSORS_G_PER_LSB_CFG / accScale;
        accScaled.z = (accelRaw.z) * SENSORS_G_PER_LSB_CFG / accScale;

        // Compensate for accelerometer alignment
        sensorsAccAlignToGravity(&accScaled, &sensorData.acc);
        applyAxis3fLpf((lpf2pData *)(&accLpf), &sensorData.acc);
    }
}



//Modified data processing function

// static void processAccGyroMeasurements(void)
// {
//     struct bmi2_sens_data sensor_data;
//     int8_t rslt;

//     // Get accelerometer and gyroscope data
//     rslt = bmi2_get_sensor_data(&sensor_data, &bmi2_dev);
//     if (rslt == BMI2_OK) {
//         // Update raw data
//         accelRaw.x = sensor_data.acc.x;
//         accelRaw.y = sensor_data.acc.y;
//         accelRaw.z = sensor_data.acc.z;

//         gyroRaw.x = sensor_data.gyr.x;
//         gyroRaw.y = sensor_data.gyr.y;
//         gyroRaw.z = sensor_data.gyr.z;

//         printf("\n----- Testing Sensor Directions -----\n");
//         printf("Raw Accelerometer (x,y,z): %d, %d, %d\n", accelRaw.x, accelRaw.y, accelRaw.z);
//         printf("Raw Gyroscope (x,y,z): %d, %d, %d\n", gyroRaw.x, gyroRaw.y, gyroRaw.z);
        
//         // Add delay to read values (500ms)
//         vTaskDelay(pdMS_TO_TICKS(2000));

//         // Process gyro bias if needed
//         if (!gyroBiasFound) {
//             gyroBiasFound = processGyroBias(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
//             printf("\nGyro Bias (x,y,z): %.2f, %.2f, %.2f\n", gyroBias.x, gyroBias.y, gyroBias.z);
//             // Add delay after bias calculation (1 second)
//             vTaskDelay(pdMS_TO_TICKS(1000));
//         }

//         // Convert to physical units and apply calibration
//         sensorData.gyro.x = (gyroRaw.x - gyroBias.x) * BMI2_GYR_SENSITIVITY_2000;
//         sensorData.gyro.y = (gyroRaw.y - gyroBias.y) * BMI2_GYR_SENSITIVITY_2000;
//         sensorData.gyro.z = (gyroRaw.z - gyroBias.z) * BMI2_GYR_SENSITIVITY_2000;

//         sensorData.acc.x = (accelRaw.x) * BMI2_ACC_SENSITIVITY_16G;
//         sensorData.acc.y = (accelRaw.y) * BMI2_ACC_SENSITIVITY_16G;
//         sensorData.acc.z = (accelRaw.z) * BMI2_ACC_SENSITIVITY_16G;

//         printf("\nDirection Test:\n");
//         printf("Tilt FORWARD  - Y should change\n");
//         printf("Tilt RIGHT   - X should change\n");
//         printf("Calibrated Accelerometer (m/s²) (x,y,z): %.2f, %.2f, %.2f\n", 
//                sensorData.acc.x, sensorData.acc.y, sensorData.acc.z);
//         printf("Calibrated Gyroscope (deg/s) (x,y,z): %.2f, %.2f, %.2f\n", 
//                sensorData.gyro.x, sensorData.gyro.y, sensorData.gyro.z);
        
//         // Add delay before filtering (500ms)
//         vTaskDelay(pdMS_TO_TICKS(2000));

//         // Apply low-pass filtering
//         applyAxis3fLpf((lpf2pData *)(&gyroLpf), &sensorData.gyro);
//         applyAxis3fLpf((lpf2pData *)(&accLpf), &sensorData.acc);

//         printf("\nFiltered Values:\n");
//         printf("Filtered Accelerometer (m/s²) (x,y,z): %.2f, %.2f, %.2f\n", 
//                sensorData.acc.x, sensorData.acc.y, sensorData.acc.z);
//         printf("Filtered Gyroscope (deg/s) (x,y,z): %.2f, %.2f, %.2f\n", 
//                sensorData.gyro.x, sensorData.gyro.y, sensorData.gyro.z);
        
//         // Add final delay before next reading (1 second)
//         vTaskDelay(pdMS_TO_TICKS(1000));
//         printf("\n--------------------------------\n");
//     }
//     else {
//         printf("Error reading sensor data, result: %d\n", rslt);
//         // Add delay on error (2 seconds)
//         vTaskDelay(pdMS_TO_TICKS(2000));
//     }
// }


static void bmi2_delay_us(uint32_t period, void *intf_ptr)
{
    esp_rom_delay_us(period);  // ESP-IDF's microsecond delay function
}


static int8_t bmi2_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    return i2cdevReadReg8(I2C0_DEV, BMI2_I2C_PRIM_ADDR, reg_addr, len, data) ? BMI2_OK : BMI2_E_COM_FAIL;
}

static int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    // Cast away const for i2cdevWriteReg8 as it's known to not modify the data
    return i2cdevWriteReg8(I2C0_DEV, BMI2_I2C_PRIM_ADDR, reg_addr, len, (uint8_t *)data) ? 
           BMI2_OK : BMI2_E_COM_FAIL;
}


static void calculateGyroBias(void) {
    struct bmi2_sens_data sensor_data = {0};
    int8_t rslt;
    uint32_t startTick = xTaskGetTickCount();
    uint32_t timeout = M2T(3000); // 3 seconds timeout

    while (xTaskGetTickCount() - startTick < timeout) {
        rslt = bmi2_get_sensor_data(&sensor_data, &bmi2_dev);
        if (rslt == BMI2_OK) {
            if (processGyroBias(sensor_data.gyr.x, sensor_data.gyr.y, sensor_data.gyr.z, &gyroBias)) {
                gyroBiasFound = true;
                DEBUG_PRINT("Gyro bias calculation [OK]\n");
                break;
            }
        }
        vTaskDelay(M2T(10));
    }

    if (!gyroBiasFound) {
        DEBUG_PRINT("Gyro bias calculation [FAIL]\n");
    }
}




static void sensorsDeviceInit(void)
{
    isMagnetometerPresent = false;
    isBarometerPresent = false;

    // Wait for sensors to startup
    while (xTaskGetTickCount() < 2000) {
        vTaskDelay(M2T(50));
    }

    i2cdevInit(I2C0_DEV);

    // Initialize BMI270 device structure
    bmi2_dev.read = bmi2_i2c_read;
    bmi2_dev.write = bmi2_i2c_write;
    bmi2_dev.delay_us = bmi2_delay_us;
    bmi2_dev.intf = BMI2_I2C_INTF;
    bmi2_dev.intf_ptr = NULL;
    bmi2_dev.read_write_len = 32;
    bmi2_dev.chip_id = BMI2_I2C_PRIM_ADDR;

    // Initialize BMI270
    int8_t rslt = bmi270_init(&bmi2_dev);
    if (rslt == BMI2_OK) {
        DEBUG_PRINTI("BMI270 I2C connection [OK].\n");
        
        // Enable the accelerometer and gyro sensors
        uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };
        rslt = bmi2_sensor_enable(sensor_list, 2, &bmi2_dev);
        
        if (rslt == BMI2_OK) {
            // Configure the sensors
            struct bmi2_sens_config config[2];
            
            // Accelerometer configuration
            config[0].type = BMI2_ACCEL;
            config[0].cfg.acc.odr = BMI2_ACC_ODR_200HZ;
            config[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
            config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
            config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

            // Gyroscope configuration
            config[1].type = BMI2_GYRO;
            config[1].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;
            config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
            config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
            config[1].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;
            config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

            rslt = bmi2_set_sensor_config(config, 2, &bmi2_dev);
            if (rslt != BMI2_OK) {
                DEBUG_PRINTE("BMI270 sensor configuration failed\n");
                assert(0);
            }
        } else {
            DEBUG_PRINTE("BMI270 sensor enable failed\n");
            assert(0);
        }
    } else {
        DEBUG_PRINTE("BMI270 I2C connection [FAIL].\n");
        assert(0);
    }

    // Initialize bias
    sensorsBiasObjInit(&gyroBiasRunning);

#ifdef SENSORS_ENABLE_MAG_HM5883L
    hmc5883lInit(I2C0_DEV);

    if (hmc5883lTestConnection() == true) {
        isMagnetometerPresent = true;
        hmc5883lSetMode(HMC5883L_MODE_CONTINUOUS); // 16bit 100Hz
        DEBUG_PRINTI("hmc5883l I2C connection [OK].\n");
    } else {
        DEBUG_PRINTW("hmc5883l I2C connection [FAIL].\n");
    }

#endif

#ifdef SENSORS_ENABLE_PRESSURE_BMP280
if (bmp280Init(I2C0_DEV)) {
    isBarometerPresent = true;
    DEBUG_PRINTI("BMP280 I2C connection [OK].\n");
} else {
    DEBUG_PRINTW("BMP280 I2C connection [FAIL].\n");
}
#endif




#ifdef SENSORS_ENABLE_PRESSURE_MS5611
    if (ms5611Init(I2C0_DEV)) {
        isBarometerPresent = true;
        DEBUG_PRINTI("MS5611 I2C connection [OK].\n");
    } else {
        //TODO: Should sensor test fail hard if no connection
        DEBUG_PRINTW("MS5611 I2C connection [FAIL].\n");
    }

#endif

#ifdef SENSORS_ENABLE_RANGE_VL53L1X
    zRanger2Init();

    if (zRanger2Test() == true) {
        isVl53l1xPresent = true;
        DEBUG_PRINTI("VL53L1X I2C connection [OK].\n");
    } else {
        //TODO: Should sensor test fail hard if no connection
        DEBUG_PRINTW("VL53L1X I2C connection [FAIL].\n");
    }

#endif

#ifdef SENSORS_ENABLE_RANGE_VL53L0X
    zRangerInit();

    if (zRangerTest() == true) {
        isVl53l0xPresent = true;
        DEBUG_PRINTI("VL53L0X I2C connection [OK].\n");
    } else {
        //TODO: Should sensor test fail hard if no connection
        DEBUG_PRINTW("VL53L0X I2C connection [FAIL].\n");
    }

#endif

#ifdef SENSORS_ENABLE_FLOW_PMW3901

    flowdeck2Init();

    if (flowdeck2Test() == true) {
        isPmw3901Present = true;
        setCommandermode(POSHOLD_MODE);
        DEBUG_PRINTI("PMW3901 SPI connection [OK].\n");
    } else {
        //TODO: Should sensor test fail hard if no connection
        DEBUG_PRINTW("PMW3901 SPI connection [FAIL].\n");
    }



#endif

    DEBUG_PRINTI("sensors init done");
    DEBUG_PRINTI("sensorsDeviceInit complete\n");
    /*
    *get calib angle from NVS
    */
    // cosPitch = cosf(configblockGetCalibPitch() * (float)M_PI / 180);
    // sinPitch = sinf(configblockGetCalibPitch() * (float)M_PI / 180);
    // cosRoll = cosf(configblockGetCalibRoll() * (float)M_PI / 180);
    // sinRoll = sinf(configblockGetCalibRoll() * (float)M_PI / 180);
    cosPitch = cosf(PITCH_CALIB * (float)M_PI / 180);
    sinPitch = sinf(PITCH_CALIB * (float)M_PI / 180);
    cosRoll = cosf(ROLL_CALIB * (float)M_PI / 180);
    sinRoll = sinf(ROLL_CALIB * (float)M_PI / 180);
    DEBUG_PRINTI("pitch_calib = %f,roll_calib = %f",PITCH_CALIB,ROLL_CALIB);
    //calculateGyroBias();
}









static void sensorsTaskInit(void)
{
  accelerometerDataQueue = STATIC_MEM_QUEUE_CREATE(accelerometerDataQueue);
  gyroDataQueue = STATIC_MEM_QUEUE_CREATE(gyroDataQueue);
  magnetometerDataQueue = STATIC_MEM_QUEUE_CREATE(magnetometerDataQueue);
  barometerDataQueue = STATIC_MEM_QUEUE_CREATE(barometerDataQueue);

  STATIC_MEM_TASK_CREATE(sensorsTask, sensorsTask, SENSORS_TASK_NAME, NULL, SENSORS_TASK_PRI);
  DEBUG_PRINTD("xTaskCreate sensorsTask \n");
}

// static void IRAM_ATTR sensors_inta_isr_handler(void *arg)
// {
//     portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
//     imuIntTimestamp = usecTimestamp(); //This function returns the number of microseconds since esp_timer was initialized
//     xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

//     if (xHigherPriorityTaskWoken) {
//         portYIELD_FROM_ISR();
//     }
// }

static void IRAM_ATTR sensors_isr_handler(void *arg)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    imuIntTimestamp = usecTimestamp();
    xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// static void sensorsInterruptInit(void)
// {
//     gpio_config_t io_conf = {
//         .intr_type = GPIO_INTR_POSEDGE,
//         .pin_bit_mask = (1ULL << CONFIG_BMI270_PIN_INT),
//         .mode = GPIO_MODE_INPUT,
//         .pull_down_en = 0,
//         .pull_up_en = 1,
//     };

//     gpio_config(&io_conf);
//     gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
//     gpio_isr_handler_add(CONFIG_BMI270_PIN_INT, sensors_isr_handler, NULL);
// }

static void sensorsInterruptInit(void)
{

    DEBUG_PRINTD("sensorsInterruptInit \n");
    gpio_config_t io_conf;
    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pins
    io_conf.pin_bit_mask = (1ULL << GPIO_INTA_MPU6050_IO);
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    sensorsDataReady = xSemaphoreCreateBinary();
    dataReady = xSemaphoreCreateBinary();
    gpio_config(&io_conf);
    //install gpio isr service
    //portDISABLE_INTERRUPTS();
    gpio_set_intr_type(GPIO_INTA_MPU6050_IO, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INTA_MPU6050_IO, sensors_isr_handler, (void *)GPIO_INTA_MPU6050_IO);
    //portENABLE_INTERRUPTS();
    DEBUG_PRINTD("sensorsInterruptInit done \n");

    //   FSYNC "shall not be floating, must be set high or low by the MCU"

}


// static void sensorsInterruptInit(void)
// {
//     gpio_config_t io_conf = {
//         .intr_type = GPIO_INTR_POSEDGE,
//         .pin_bit_mask = (1ULL << CONFIG_BMI270_INT1_PIN),
//         .mode = GPIO_MODE_INPUT,
//         .pull_down_en = 0,
//         .pull_up_en = 1,
//     };

//     gpio_config(&io_conf);
//     gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
//     gpio_isr_handler_add(CONFIG_BMI270_INT1_PIN, sensors_isr_handler, NULL);
// }

void sensorsMpu6050Hmc5883lMs5611Init(void)
{
    if (isInit) {
        return;
    }
    //i2cdevInit(I2C0_DEV);
    sensorsBiasObjInit(&gyroBiasRunning);
    sensorsDeviceInit();
    sensorsInterruptInit();
    sensorsTaskInit();
    isInit = true;
}


bool sensorsMpu6050Hmc5883lMs5611Test(void)
{
    bool testStatus = true;

    if (!isInit) {
        DEBUG_PRINTE("Error while initializing sensor task\r\n");
        testStatus = false;
    }

    // Try for 3 seconds so the quad has stabilized enough to pass the test
    for (int i = 0; i < 300; i++) {
        // Replace MPU6050 self-test with BMI270 self-test
        if (bmi270SelfTest() == true) {  // You'll need to implement this function based on BMI270 datasheet
            isBmi270TestPassed = true;    // Replace isMpu6050TestPassed with isBmi270TestPassed
            break;
        } else {
            vTaskDelay(M2T(10));
        }
    }

    testStatus &= isBmi270TestPassed;

#ifdef SENSORS_ENABLE_MAG_HM5883L
    testStatus &= isMagnetometerPresent;

    if (testStatus) {
        isHmc5883lTestPassed = hmc5883lSelfTest();
        testStatus &= isHmc5883lTestPassed;
    }
#endif

#ifdef SENSORS_ENABLE_PRESSURE_BMP280
    testStatus &= isBarometerPresent;

    if (testStatus) {
        // Add BMP280-specific self-test if required, or use a generic test
        isBmp280TestPassed = bmp280SelfTest();
        testStatus &= isBmp280TestPassed;
    }
#endif

#ifdef SENSORS_ENABLE_PRESSURE_MS5611
    testStatus &= isBarometerPresent;

    if (testStatus) {
        isMs5611TestPassed = ms5611SelfTest();
        testStatus &= isMs5611TestPassed;
    }
#endif

    return testStatus;
}


bool bmi270SelfTest(void)
{
    int8_t rslt;
    uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };
    struct bmi2_sens_data sensor_data = { 0 };

    // 1. Configure and enable the accelerometer and gyroscope
    rslt = bmi2_sensor_enable(sensor_list, 2, &bmi2_dev);
    if (rslt != BMI2_OK) {
        return false;
    }

    // 2. Configure the sensors
    struct bmi2_sens_config config[2];
    
    // Accelerometer configuration
    config[0].type = BMI2_ACCEL;
    config[0].cfg.acc.odr = BMI2_ACC_ODR_200HZ;
    config[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
    config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
    config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

    // Gyroscope configuration
    config[1].type = BMI2_GYRO;
    config[1].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;
    config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
    config[1].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;
    config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

    rslt = bmi2_set_sensor_config(config, 2, &bmi2_dev);
    if (rslt != BMI2_OK) {
        return false;
    }

    // 3. Wait for sensor to stabilize
    vTaskDelay(M2T(50));

    // 4. Perform accelerometer self-test
    rslt = bmi2_perform_accel_self_test(&bmi2_dev);
    if (rslt != BMI2_OK) {
        return false;
    }

    // 5. Get sensor data to verify
    rslt = bmi2_get_sensor_data(&sensor_data, &bmi2_dev);
    if (rslt != BMI2_OK) {
        return false;
    }

    // Self-test passed if we reach here
    return true;
}




/**
 * Calculates accelerometer scale out of SENSORS_ACC_SCALE_SAMPLES samples. Should be called when
 * platform is stable.
 */
static bool processAccScale(int16_t ax, int16_t ay, int16_t az)
{
    static bool accBiasFound = false;
    static uint32_t accScaleSumCount = 0;

    if (!accBiasFound) {
        accScaleSum += sqrtf(powf(ax * SENSORS_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_G_PER_LSB_CFG, 2) + powf(az * SENSORS_G_PER_LSB_CFG, 2));
        accScaleSumCount++;

        if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES) {
            accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
            accBiasFound = true;
        }
    }

    return accBiasFound;
}

// Gyro bias calculation implementations
// #ifdef GYRO_BIAS_LIGHT_WEIGHT

// Lightweight implementation without buffer
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
    sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

    if (!gyroBiasRunning.isBiasValueFound) {
        sensorsFindBiasValue(&gyroBiasRunning);

        if (gyroBiasRunning.isBiasValueFound) {
            DEBUG_PRINTI("Gyro bias found: x=%f, y=%f, z=%f\n", 
                (double)gyroBiasRunning.bias.x,
                (double)gyroBiasRunning.bias.y,
                (double)gyroBiasRunning.bias.z);
            
            soundSetEffect(SND_CALIB);
            ledseqRun(&seq_calibrated);
        }
    }

    gyroBiasOut->x = gyroBiasRunning.bias.x;
    gyroBiasOut->y = gyroBiasRunning.bias.y;
    gyroBiasOut->z = gyroBiasRunning.bias.z;

    return gyroBiasRunning.isBiasValueFound;
}

// #else

// Standard implementation with circular buffer
// static bool sensorsFindBiasValue(BiasObj *bias)
// {
//     static int32_t varianceSampleTime;
//     bool foundBias = false;

//     if (bias->isBufferFilled) {
//         sensorsCalculateVarianceAndMean(bias, &bias->variance, &bias->mean);

//         DEBUG_PRINTI("Variance: x=%f, y=%f, z=%f", 
//             (double)bias->variance.x,
//             (double)bias->variance.y,
//             (double)bias->variance.z);

//         if (bias->variance.x < GYRO_VARIANCE_THRESHOLD_X &&
//             bias->variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
//             bias->variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
//             (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount())) {
//             varianceSampleTime = xTaskGetTickCount();
//             bias->bias.x = bias->mean.x;
//             bias->bias.y = bias->mean.y;
//             bias->bias.z = bias->mean.z;
//             foundBias = true;
//             bias->isBiasValueFound = true;
//             DEBUG_PRINTI("Bias found!");
//         }
//     }

//     return foundBias;
// }


// Initialize bias object 
static void sensorsBiasObjInit(BiasObj *bias)
{
    bias->isBufferFilled = false;
    bias->bufHead = bias->buffer;
}


/**
 * Calculates the variance and mean for the bias buffer.
 */
static void sensorsCalculateVarianceAndMean(BiasObj *bias, Axis3f *varOut, Axis3f *meanOut)
{
    uint32_t i;
    int64_t sum[GYRO_NBR_OF_AXES] = {0};
    int64_t sumSq[GYRO_NBR_OF_AXES] = {0};

    for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++) {
        sum[0] += bias->buffer[i].x;
        sum[1] += bias->buffer[i].y;
        sum[2] += bias->buffer[i].z;
        sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
        sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
        sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
    }

    varOut->x = (sumSq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
    varOut->y = (sumSq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
    varOut->z = (sumSq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

    meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
    meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
    meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

/**
 * Calculates the mean for the bias buffer.
 */
static void __attribute__((used)) sensorsCalculateBiasMean(BiasObj *bias, Axis3i32 *meanOut)
{
    uint32_t i;
    int32_t sum[GYRO_NBR_OF_AXES] = {0};

    for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++) {
        sum[0] += bias->buffer[i].x;
        sum[1] += bias->buffer[i].y;
        sum[2] += bias->buffer[i].z;
    }

    meanOut->x = sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
    meanOut->y = sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
    meanOut->z = sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

/**
 * Adds a new value to the variance buffer and if it is full
 * replaces the oldest one. Thus a circular buffer.
 */
static void sensorsAddBiasValue(BiasObj *bias, int16_t x, int16_t y, int16_t z)
{
    bias->bufHead->x = x;
    bias->bufHead->y = y;
    bias->bufHead->z = z;
    bias->bufHead++;

    if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES]) {
        bias->bufHead = bias->buffer;
        bias->isBufferFilled = true;
    }
}

/**
 * Checks if the variances is below the predefined thresholds.
 * The bias value should have been added before calling this.
 * @param bias  The bias object
 */
// Keep only one version of this function
static bool sensorsFindBiasValue(BiasObj *bias)
{
    static int32_t varianceSampleTime;
    bool foundBias = false;

    if (bias->isBufferFilled) {
        sensorsCalculateVarianceAndMean(bias, &bias->variance, &bias->mean);

        DEBUG_PRINTI("Variance: x=%f, y=%f, z=%f", 
            (double)bias->variance.x,
            (double)bias->variance.y,
            (double)bias->variance.z);

        if (bias->variance.x < GYRO_VARIANCE_THRESHOLD_X &&
            bias->variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
            bias->variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
            (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount())) {
            varianceSampleTime = xTaskGetTickCount();
            bias->bias.x = bias->mean.x;
            bias->bias.y = bias->mean.y;
            bias->bias.z = bias->mean.z;
            foundBias = true;
            bias->isBiasValueFound = true;
            DEBUG_PRINTI("Bias found!");
        }
    }

    return foundBias;
}

bool sensorsMpu6050Hmc5883lMs5611ManufacturingTest(void)
{
    bool testStatus = false;
    struct bmi2_sens_data sensor_data = { 0 };
    float pitch, roll;
    uint32_t startTick = xTaskGetTickCount();

    // Perform self-test
    testStatus = bmi270SelfTest();

    if (testStatus) {
        sensorsBiasObjInit(&gyroBiasRunning);

        while (xTaskGetTickCount() - startTick < SENSORS_VARIANCE_MAN_TEST_TIMEOUT) {
            int8_t rslt = bmi2_get_sensor_data(&sensor_data, &bmi2_dev);
            
            if (rslt == BMI2_OK) {
                if (processGyroBias(sensor_data.gyr.x, sensor_data.gyr.y, sensor_data.gyr.z, &gyroBias)) {
                    gyroBiasFound = true;
                    DEBUG_PRINTI("Gyro variance test [OK]\n");
                    break;
                }
            }
            vTaskDelay(M2T(1));
        }

        if (gyroBiasFound) {
            // Calculate acceleration in g
            float acc_x = sensor_data.acc.x * SENSORS_G_PER_LSB_CFG;
            float acc_y = sensor_data.acc.y * SENSORS_G_PER_LSB_CFG;
            float acc_z = sensor_data.acc.z * SENSORS_G_PER_LSB_CFG;

            // Calculate pitch and roll based on accelerometer
            pitch = tanf(-acc_x / (sqrtf(acc_y * acc_y + acc_z * acc_z))) * 180 / (float)M_PI;
            roll = tanf(acc_y / acc_z) * 180 / (float)M_PI;

            if ((fabsf(roll) < SENSORS_MAN_TEST_LEVEL_MAX) && 
                (fabsf(pitch) < SENSORS_MAN_TEST_LEVEL_MAX)) {
                DEBUG_PRINTI("Acc level test [OK]\n");
                testStatus = true;
            } else {
                DEBUG_PRINTE("Acc level test Roll:%0.2f, Pitch:%0.2f [FAIL]\n", 
                            (double)roll, (double)pitch);
                testStatus = false;
            }
        } else {
            DEBUG_PRINTE("Gyro variance test [FAIL]\n");
            testStatus = false;
        }
    }

    return testStatus;
}

/**
 * Compensate for a miss-aligned accelerometer. It uses the trim
 * data gathered from the UI and written in the config-block to
 * rotate the accelerometer to be aligned with gravity.
 */
static void sensorsAccAlignToGravity(Axis3f *in, Axis3f *out)
{
    Axis3f rx;
    Axis3f ry;

    // Rotate around x-axis
    rx.x = in->x;
    rx.y = in->y * cosRoll - in->z * sinRoll;
    rx.z = in->y * sinRoll + in->z * cosRoll;

    // Rotate around y-axis
    ry.x = rx.x * cosPitch - rx.z * sinPitch;
    ry.y = rx.y;
    ry.z = -rx.x * sinPitch + rx.z * cosPitch;

    out->x = ry.x;
    out->y = ry.y;
    out->z = ry.z;
}

/** set different low pass filters in different environment
 *
 *
 */
void sensorsMpu6050Hmc5883lMs5611SetAccMode(accModes accMode)
{
    switch (accMode)
    {
    case ACC_MODE_PROPTEST:
        mpu6050SetRate(7);
        mpu6050SetDLPFMode(MPU6050_DLPF_BW_256);
        for (uint8_t i = 0; i < 3; i++)
        {
            lpf2pInit(&accLpf[i], 1000, 250);
        }
        break;
    case ACC_MODE_FLIGHT:
    default:
        mpu6050SetRate(0);
#ifdef CONFIG_TARGET_ESP32_S2_DRONE_V1_2
        mpu6050SetDLPFMode(MPU6050_DLPF_BW_42);
        for (uint8_t i = 0; i < 3; i++) {
        lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
        }
#else
        mpu6050SetDLPFMode(MPU6050_DLPF_BW_98);
        for (uint8_t i = 0; i < 3; i++) {
        lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
        }
#endif
        break;
    }
}

static void applyAxis3fLpf(lpf2pData *data, Axis3f *in)
{
    for (uint8_t i = 0; i < 3; i++) {
        in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
    }
}

#ifdef GYRO_ADD_RAW_AND_VARIANCE_LOG_VALUES
LOG_GROUP_START(gyro)
LOG_ADD(LOG_INT16, xRaw, &gyroRaw.x)
LOG_ADD(LOG_INT16, yRaw, &gyroRaw.y)
LOG_ADD(LOG_INT16, zRaw, &gyroRaw.z)
LOG_ADD(LOG_FLOAT, xVariance, &gyroBiasRunning.variance.x)
LOG_ADD(LOG_FLOAT, yVariance, &gyroBiasRunning.variance.y)
LOG_ADD(LOG_FLOAT, zVariance, &gyroBiasRunning.variance.z)
LOG_GROUP_STOP(gyro)
#endif

//TODO:
PARAM_GROUP_START(imu_sensors)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MS5611, &isBarometerPresent) // TODO: Rename MS5611 to LPS25H. Client needs to be updated at the same time.
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY , VL53L1X , &isVl53l1xPresent)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY , PMW3901 , &isPmw3901Present)
PARAM_GROUP_STOP(imu_sensors)

PARAM_GROUP_START(imu_tests)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, mpu6050, &isBmi270TestPassed)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MS5611, &isBarometerPresent) // TODO: Rename MS5611 to LPS25H. Client needs to be updated at the same time.
PARAM_GROUP_STOP(imu_tests)