#if !defined(__OPT3001_H__)
#define __OPT3001_H__

#include <Arduino.h>

#define OPT3001_ADDR  0x45

#define OPT3001_REG_RESULT          0x00
#define OPT3001_REG_CONFIG          0x01
#define OPT3001_REG_LOLIMIT         0x02
#define OPT3001_REG_HILIMIT         0x03
#define OPT3001_REG_MANUFACTUREID   0x7E
#define OPT3001_DEVICEID            0x7F
#define OPT3001_CMD_CONFIG_MSB      0xC6
#define OPT3001_CMD_CONFIG_LSB      0x10

extern uint16_t sens_light;
#endif
