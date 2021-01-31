#define FCY 16000000UL 
#include <assert.h>
#include <stdbool.h>
#include "mcc_generated_files/system.h"
#include <stdio.h>
#include <stdlib.h>
#include <libpic30.h>
#include <xc.h>
#include <string.h>
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

uint16_t X_Anz;                                 // definiert "X_Anz"
uint16_t Y_Anz;                                 // definiert "Y_Anz"
uint16_t Z_Anz;                                 // definiert "Z_Anz"
uint16_t gX_Anz;                                // definiert "gX_Anz"
uint16_t gY_Anz;                                // definiert "gY_Anz"
uint16_t gZ_Anz;                                // definiert "gZ_Anz"

/*
                         Main application
 */
int main(void){                                 // "main"
    __delay_ms(1000);                           // warte 1000ms
    SYSTEM_Initialize();                        // ruft "SYSTEM_Initialize" auf
    MPU6050_Init();                             // Initialisierung von MPU6050
    __delay_ms(1000);                           // warte 1000ms
    
    while (1){                                  // Endloschleife
        X_Anz = MPU6050_Get_X();                // schreibt "MPU6050_Get_X" in "X_Anz"
        Y_Anz = MPU6050_Get_Y();                // schreibt "MPU6050_Get_Y" in "Y_Anz"
        Z_Anz = MPU6050_Get_Z();                // schreibt "MPU6050_Get_Z" in "Z_Anz"
        gX_Anz = MPU6050_Get_gX();              // schreibt "MPU6050_Get_gX" in "gX_Anz"
        gY_Anz = MPU6050_Get_gY();              // schreibt "MPU6050_Get_gY" in "gY_Anz"
        gZ_Anz = MPU6050_Get_gZ();              // schreibt "MPU6050_Get_gZ" in "gZ_Anz"
        printf("X= %d, Y= %d, Z= %d\r\n",X_Anz, Y_Anz, Z_Anz);  // schreibt den Inhalt von "X_Anz", "Y_Anz", "Z_Anz" an UART
        printf("gX= %d, gY= %d, gZ= %d\r\n",gX_Anz, gY_Anz, gZ_Anz);// schreibt den Inhalt von "gX_Anz", "gY_Anz", "gZ_Anz" an UART
        __delay_ms(500);                        // warte 500ms
    }                                           // 
}                                               // 

