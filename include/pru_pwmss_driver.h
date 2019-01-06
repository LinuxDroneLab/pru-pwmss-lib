/*
 * pru_pwmss_driver.h
 *
 *  Created on: 12 ago 2018
 *      Author: andrea
 */

#ifndef INCLUDE_PRU_PWMSS_DRIVER_H_
#define INCLUDE_PRU_PWMSS_DRIVER_H_

#include <stdint.h>


#define PRU_PWMSS_LIB_CMD_ID             0xF0
#define PRU_PWMSS_LIB_CMD_START          0x01
#define PRU_PWMSS_LIB_CMD_START_RSP      0x02
#define PRU_PWMSS_LIB_CMD_STOP           0x03
#define PRU_PWMSS_LIB_CMD_STOP_RSP       0x04
#define PRU_PWMSS_LIB_CMD_SET_DATA       0x05
#define PRU_PWMSS_LIB_CMD_SET_DATA_RSP   0x06
#define PRU_PWMSS_LIB_CMD_SET_DUTY       0x07
#define PRU_PWMSS_LIB_CMD_SET_DUTY_RSP   0x08
#define PRU_PWMSS_LIB_CMD_SET_PERIOD     0x09
#define PRU_PWMSS_LIB_CMD_SET_PERIOD_RSP 0x0a
#define PRU_PWMSS_LIB_CMD_DATA_BUFF_SIZE 0x40 /* 64 bytes: buffer for responses */
#define PRU_PWMSS_LIB_CMD_DATA_BUFF_OFF  0x2  /* offset for data buffer */

typedef struct {
    uint8_t (*onStart)(unsigned char* data, uint8_t dataBytes);
    uint8_t (*onStop)(unsigned char* data, uint8_t dataBytes);
    uint8_t (*onSetDuty)(unsigned char* data, uint8_t dataBytes);
    uint8_t (*onSetPeriod)(unsigned char* data, uint8_t dataBytes);
    uint8_t (*onSetData)(unsigned char* data, uint8_t dataBytes);
} PruDShotLibConfig;

uint8_t pru_pwmss_lib_IsConfigured(uint8_t pwmssDevice);
uint8_t pru_pwmss_lib_IsInitialized(uint8_t pwmssDevice);
uint8_t pru_pwmss_lib_IsRunning(uint8_t pwmssDevice);
uint8_t pru_pwmss_lib_Init(uint8_t pwmssDevice);
uint8_t pru_pwmss_lib_Pulse(uint8_t pwmssDevice);
uint8_t pru_pwmss_lib_Conf(PruDShotLibConfig* config);
uint8_t pru_pwmss_lib_SetData(uint8_t pwmssDevice, uint16_t period, uint16_t duA, uint16_t duB);
uint8_t pru_pwmss_lib_SetDuty(uint8_t pwmssDevice, uint16_t duA, uint16_t duB);
uint8_t pru_pwmss_lib_SetPeriod(uint8_t pwmssDevice, uint16_t period);
uint8_t pru_pwmss_lib_Start(uint8_t pwmssDevice);
uint8_t pru_pwmss_lib_Stop(uint8_t pwmssDevice);

uint8_t pru_pwmss_lib_IsCmdSupported(unsigned char* cmd, uint8_t numBytes);
uint8_t pru_pwmss_lib_ExecCmd(unsigned char* cmd, uint8_t numBytes);

#endif /* INCLUDE_PRU_PWMSS_DRIVER_H_ */
