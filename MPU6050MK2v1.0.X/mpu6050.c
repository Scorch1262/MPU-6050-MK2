/*
 * File:   gyro.c
 * Author: Torben
 *
 * Created on December 9, 2020, 7:06 PM
 */

// Gyroscope
#define FCY 16000000UL

#include "xc.h"
#include <assert.h>
#include <stdbool.h>
#include <libpic30.h>
#include "mcc_generated_files/system.h"
#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/i2c1.h"
#include "mpu6050.h"

#if defined(__dsPIC33E__)
	#include <p33exxxx.h>
#elif defined(__dsPIC33F__)
	#include <p33fxxxx.h>
#elif defined(__dsPIC30F__)
	#include <p30fxxxx.h>
#elif defined(__PIC24E__)
	#include <p24exxxx.h>
#elif defined(__PIC24H__)
	#include <p24hxxxx.h>
#elif defined(__PIC24F__) || defined(__PIC24FK__)
	#include <p24fxxxx.h>
#endif

#define MPU6050_ADDR        0x69 

#define XG_OFFS_TC          0x00
#define YG_OFFS_TC          0x01
#define ZG_OFFS_TC          0x02
#define X_FINE_GAIN         0x03
#define Y_FINE_GAIN         0x04
#define Z_FINE_GAIN         0x05
#define XA_OFFS_H           0x06 
#define XA_OFFS_L_TC        0x07
#define YA_OFFS_H           0x08 
#define YA_OFFS_L_TC        0x09
#define ZA_OFFS_H           0x0A 
#define ZA_OFFS_L_TC        0x0B
#define XG_OFFS_USRH        0x13
#define XG_OFFS_USRL        0x14
#define YG_OFFS_USRH        0x15
#define YG_OFFS_USRL        0x16
#define ZG_OFFS_USRH        0x17
#define ZG_OFFS_USRL        0x18
#define SMPLRT_DIV          0x19
#define CONFIG              0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define FF_THR              0x1D
#define FF_DUR              0x1E
#define MOT_THR             0x1F
#define MOT_DUR             0x20
#define ZRMOT_THR           0x21
#define ZRMOT_DUR           0x22
#define FIFO_EN             0x23
#define I2C_MST_CTRL        0x24
#define I2C_SLV0_ADDR       0x25
#define I2C_SLV0_REG        0x26
#define I2C_SLV0_CTRL       0x27
#define I2C_SLV1_ADDR       0x28
#define I2C_SLV1_REG        0x29
#define I2C_SLV1_CTRL       0x2A
#define I2C_SLV2_ADDR       0x2B
#define I2C_SLV2_REG        0x2C
#define I2C_SLV2_CTRL       0x2D
#define I2C_SLV3_ADDR       0x2E
#define I2C_SLV3_REG        0x2F
#define I2C_SLV3_CTRL       0x30
#define I2C_SLV4_ADDR       0x31
#define I2C_SLV4_REG        0x32
#define I2C_SLV4_DO         0x33
#define I2C_SLV4_CTRL       0x34
#define I2C_SLV4_DI         0x35
#define I2C_MST_STATUS      0x36
#define INT_PIN_CFG         0x37
#define INT_ENABLE          0x38
#define DMP_INT_STATUS      0x39
#define INT_STATUS          0x3A
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40
#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42
#define GYRO_XOUT_H         0x43
#define GYRO_XOUT_L         0x44
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48
#define EXT_SENS_DATA_00    0x49
#define EXT_SENS_DATA_01    0x4A
#define EXT_SENS_DATA_02    0x4B
#define EXT_SENS_DATA_03    0x4C
#define EXT_SENS_DATA_04    0x4D
#define EXT_SENS_DATA_05    0x4E
#define EXT_SENS_DATA_06    0x4F
#define EXT_SENS_DATA_07    0x50
#define EXT_SENS_DATA_08    0x51
#define EXT_SENS_DATA_09    0x52
#define EXT_SENS_DATA_10    0x53
#define EXT_SENS_DATA_11    0x54
#define EXT_SENS_DATA_12    0x55
#define EXT_SENS_DATA_13    0x56
#define EXT_SENS_DATA_14    0x57
#define EXT_SENS_DATA_15    0x58
#define EXT_SENS_DATA_16    0x59
#define EXT_SENS_DATA_17    0x5A
#define EXT_SENS_DATA_18    0x5B
#define EXT_SENS_DATA_19    0x5C
#define EXT_SENS_DATA_20    0x5D
#define EXT_SENS_DATA_21    0x5E
#define EXT_SENS_DATA_22    0x5F
#define EXT_SENS_DATA_23    0x60
#define MOT_DETECT_STATUS   0x61
#define I2C_SLV0_DO         0x63
#define I2C_SLV1_DO         0x64
#define I2C_SLV2_DO         0x65
#define I2C_SLV3_DO         0x66
#define I2C_MST_DELAY_CTRL  0x67
#define SIGNAL_PATH_RESET   0x68
#define MOT_DETECT_CTRL     0x69
#define USER_CTRL           0x6A
#define PWR_MGMT_1          0x6B
#define PWR_MGMT_2          0x6C
#define BANK_SEL            0x6D
#define MEM_START_ADDR      0x6E
#define MEM_R_W             0x6F
#define DMP_CFG_1           0x70
#define DMP_CFG_2           0x71
#define FIFO_COUNTH         0x72
#define FIFO_COUNTL         0x73
#define FIFO_R_W            0x74
#define WHO_AM_I            0x75

I2C1_MESSAGE_STATUS status;                                             // 

uint8_t  writeBuffer[3] = {0x00, 0x00, 0x00};                           // definiert "writeBuffer"
uint8_t  readBuffer[2] = {0x00, 0x00};                                  // definiert "readBuffer" 

uint8_t Slave_Address;                                                  // definiert "Slave_Address" 

uint16_t retryTimeOut1;                                                  // definiert "retryTimeOut1" 
uint16_t retryTimeOut2;                                                  // definiert "retryTimeOut2"  

uint8_t ACCEL_X_L = ACCEL_XOUT_L;                                       // definiert "ACCEL_X_L" 
uint8_t ACCEL_X_H = ACCEL_XOUT_H;                                       // definiert "ACCEL_X_H" 
uint8_t ACCEL_Y_L = ACCEL_YOUT_L;                                       // definiert "ACCEL_Y_L" 
uint8_t ACCEL_Y_H = ACCEL_YOUT_H;                                       // definiert "ACCEL_Y_H"
uint8_t ACCEL_Z_L = ACCEL_ZOUT_L;                                       // definiert "ACCEL_Z_L" 
uint8_t ACCEL_Z_H = ACCEL_ZOUT_H;                                       // definiert "ACCEL_Z_H"
uint8_t ACCEL_gX_L = GYRO_XOUT_L;                                       // definiert "ACCEL_gX_L" 
uint8_t ACCEL_gX_H = GYRO_XOUT_H;                                       // definiert "ACCEL_gX_H" 
uint8_t ACCEL_gY_L = GYRO_YOUT_L;                                       // definiert "ACCEL_gY_L" 
uint8_t ACCEL_gY_H = GYRO_YOUT_H;                                       // definiert "ACCEL_gY_H"
uint8_t ACCEL_gZ_L = GYRO_ZOUT_L;                                       // definiert "ACCEL_gZ_L" 
uint8_t ACCEL_gZ_H = GYRO_ZOUT_H;                                       // definiert "ACCEL_gZ_H"

void MPU6050_Init(void) {                                               // "MPU6050_Init"
    Slave_Address = MPU6050_ADDR;                                       // schreibt "MPU6050_ADDR" in "Slave_Address"

    writeBuffer[0] = SMPLRT_DIV;                                        // "writeBuffer[0]" gleich Wert von "SMPLRT_DIV"
    writeBuffer[1] = 0x07;                                              // "writeBuffer[1]" gleich inhalt von "0x07"
    I2C1_MasterWrite(&writeBuffer[0], 2, Slave_Address, &status);       // 1KHz sample rate
    i2c1_message_pending_100();                                         // ruft "i2c1_message_pending_100" auf 

    writeBuffer[0] = PWR_MGMT_1;                                        // "writeBuffer[0]" gleich Wert von "PWR_MGMT_1"
    writeBuffer[1] = 0x01;                                              // "writeBuffer[1]" gleich inhalt von "0x01"
    I2C1_MasterWrite(&writeBuffer[0], 2, Slave_Address, &status);       // X axis gyroscope reference frequency
    i2c1_message_pending_100();                                         // ruft "i2c1_message_pending_100" auf

    writeBuffer[0] = CONFIG;                                            // "writeBuffer[0]" gleich Wert von "CONFIG"
    writeBuffer[1] = 0x00;                                              // "writeBuffer[1]" gleich inhalt von "0x00"
    I2C1_MasterWrite(&writeBuffer[0], 2, Slave_Address, &status);       // Fs = 8KHz
    i2c1_message_pending_100();                                         // ruft "i2c1_message_pending_100" auf
    
    writeBuffer[0] = GYRO_CONFIG;                                       // "writeBuffer[0]" gleich Wert von "GYRO_CONFIG"
    writeBuffer[1] = 0x18;                                              // "writeBuffer[1]" gleich inhalt von "0x18"
    I2C1_MasterWrite(&writeBuffer[0], 2, Slave_Address, &status);       // Full scale range +/- 2000 degree/C
    i2c1_message_pending_100();                                         // ruft "i2c1_message_pending_100" auf

    writeBuffer[0] = INT_ENABLE;                                        // "writeBuffer[0]" gleich Wert von "INT_ENABLE"
    writeBuffer[1] = 0x01;                                              // "writeBuffer[1]" gleich inhalt von "0x01"
    I2C1_MasterWrite(&writeBuffer[0], 2, Slave_Address, &status);       // interrupt enable 
    i2c1_message_pending_100();                                         // ruft "i2c1_message_pending_100" auf
    __delay_ms(1000);                                                   // warte 1000ms
}                                                                       // 

uint16_t MPU6050_Get_X(void){                                           // "MPU6050_Get_X"
    uint16_t X_Data;                                                    // definiert "X_Data" 
    Slave_Address = MPU6050_ADDR;                                       // schreibt "MPU6050_ADDR" in "Slave_Address"

    I2C1_MasterWrite(&ACCEL_X_H, 1, Slave_Address, &status);            // schreibt 1 Bypte "ACCEL_X_H" in "Slave_Address"
    i2c1_message_pending_100();                                         // ruft "i2c1_message_pending_100" auf 
    I2C1_MasterRead(&readBuffer[0], 2, Slave_Address, &status);         // lieﬂt 2 Bypte von "Slave_Address" aus 
    i2c1_message_pending_100();                                         // ruft "i2c1_message_pending_100" auf
     
    X_Data = (readBuffer[0] << 8) + readBuffer[1];                      //
    return X_Data;                                                      // gibt "X_Data" zur¸ck
}                                                                       // 

uint16_t MPU6050_Get_Y(void){                                           // "MPU6050_Get_Y"
    uint16_t Y_Data;                                                    // definiert "Y_Data" 
    Slave_Address = MPU6050_ADDR;                                       // schreibt "MPU6050_ADDR" in "Slave_Address"
  
    I2C1_MasterWrite(&ACCEL_Y_H, 1, Slave_Address, &status);            // schreibt 1 Bypte "ACCEL_Y_H" in "Slave_Address"
    i2c1_message_pending_100();                                         // ruft "i2c1_message_pending_100" auf 
    I2C1_MasterRead(&readBuffer[0], 2, Slave_Address, &status);         // lieﬂt 2 Bypte von "Slave_Address" aus 
    i2c1_message_pending_100();                                         // ruft "i2c1_message_pending_100" auf
    
    Y_Data = (readBuffer[0] << 8) + readBuffer[1];                      //
    return Y_Data;                                                      // gibt "Y_Data" zur¸ck
}                                                                       // 

uint16_t MPU6050_Get_Z(void){                                           // "MPU6050_Get_Z"
    uint16_t Z_Data;                                                    // definiert "Z_Data" 
    Slave_Address = MPU6050_ADDR;                                       // schreibt "MPU6050_ADDR" in "Slave_Address"
  
    I2C1_MasterWrite(&ACCEL_Z_H, 1, Slave_Address, &status);            // schreibt 1 Bypte "ACCEL_Z_H" in "Slave_Address"
    i2c1_message_pending_100();                                         // ruft "i2c1_message_pending_100" auf 
    I2C1_MasterRead(&readBuffer[0], 2, Slave_Address, &status);         // lieﬂt 2 Bypte von "Slave_Address" aus 
    i2c1_message_pending_100();                                         // ruft "i2c1_message_pending_100" auf
    
    Z_Data = (readBuffer[0] << 8) + readBuffer[1];                      //
    return Z_Data;                                                      // gibt "Z_Data" zur¸ck
}                                                                       // 

uint16_t MPU6050_Get_gX(void){                                          // "MPU6050_Get_gX"
    uint16_t gX_Data;                                                   // definiert "gX_Data" 
    Slave_Address = MPU6050_ADDR;                                       // schreibt "MPU6050_ADDR" in "Slave_Address"
  
    I2C1_MasterWrite(&ACCEL_gX_H, 1, Slave_Address, &status);           // schreibt 1 Bypte "ACCEL_gX_H" in "Slave_Address"
    i2c1_message_pending_100();                                         // ruft "i2c1_message_pending_100" auf 
    I2C1_MasterRead(&readBuffer[0], 2, Slave_Address, &status);         // lieﬂt 2 Bypte von "Slave_Address" aus 
    i2c1_message_pending_100();                                         // ruft "i2c1_message_pending_100" auf
    
    gX_Data = (readBuffer[0] << 8) + readBuffer[1];                     //
    return gX_Data;                                                     // gibt "gX_Data" zur¸ck
}                                                                       // 

uint16_t MPU6050_Get_gY(void){                                          // "MPU6050_Get_gY"
    uint16_t gY_Data;                                                   // definiert "gY_Data" 
    Slave_Address = MPU6050_ADDR;                                       // schreibt "MPU6050_ADDR" in "Slave_Address"
  
    I2C1_MasterWrite(&ACCEL_gY_H, 1, Slave_Address, &status);           // schreibt 1 Bypte "ACCEL_gY_H" in "Slave_Address"
    i2c1_message_pending_100();                                         // ruft "i2c1_message_pending_100" auf 
    I2C1_MasterRead(&readBuffer[0], 2, Slave_Address, &status);         // lieﬂt 2 Bypte von "Slave_Address" aus 
    i2c1_message_pending_100();                                         // ruft "i2c1_message_pending_100" auf
    
    gY_Data = (readBuffer[0] << 8) + readBuffer[1];                     //
    return gY_Data;                                                     // gibt "gY_Data" zur¸ck
}                                                                       // 

uint16_t MPU6050_Get_gZ(void){                                          // "MPU6050_Get_gZ"
    uint16_t gZ_Data;                                                   // definiert "gX_Data" 
    Slave_Address = MPU6050_ADDR;                                       // schreibt "MPU6050_ADDR" in "Slave_Address"
  
    I2C1_MasterWrite(&ACCEL_gZ_H, 1, Slave_Address, &status);           // schreibt 1 Bypte "ACCEL_Z_H" in "Slave_Address"
    i2c1_message_pending_100();                                         // ruft "i2c1_message_pending_100" auf 
    I2C1_MasterRead(&readBuffer[0], 2, Slave_Address, &status);         // lieﬂt 2 Bypte von "Slave_Address" aus 
    i2c1_message_pending_100();                                         // ruft "i2c1_message_pending_100" auf
    
    gZ_Data = (readBuffer[0] << 8) + readBuffer[1];                     //
    return gZ_Data;                                                     // gibt "gZ_Data" zur¸ck
}                                                                       // 

void i2c1_message_pending_100(void){                                    // "i2c1_message_pending_100"
    retryTimeOut1 = 0;                                                  // setzt "retryTimeOut1" auf 0
    retryTimeOut2 = 0;                                                  // setzt "retryTimeOut2" auf 0
    while(status == I2C1_MESSAGE_PENDING){                              // solange "status" gleich "I2C1_MESSAGE_PENDING" ist
        if (retryTimeOut2 == 500){                                      // wenn "retryTimeOut2" gleich 100 ist
            break;                                                      // Abbruch der Schleife
        }else{                                                          // sonst
            retryTimeOut2++;                                            // "retryTimeOut2" +1
        }                                                               // 
    }                                                                   // 
    while(status != I2C1_MESSAGE_FAIL){                                 // solange "status" nicht gleich "I2C1_MESSAGE_FAIL" ist
        if (retryTimeOut1 == 500){                                      // wenn "retryTimeOut1" gleich 100 ist
            break;                                                      // Abbruch der Schleife
        }else{                                                          // sonst
            retryTimeOut1++;                                            // "retryTimeOut1" +1
        }                                                               // 
    }                                                                   // 
}                                                                       // 
