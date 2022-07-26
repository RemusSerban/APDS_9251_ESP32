#ifndef _APDS_9251_REGS_H_
#define _APDS_9251_REGS_H_

#ifdef __cplusplus
extern "C" {
#endif

/** Register Addresss */
#define MAIN_CTRL                       0x00
#define LS_MEAS_RATE                    0x04
#define LS_GAIN                         0x05
#define PART_ID                         0x06
#define MAIN_STATUS                     0x07
#define LS_DATA_IR_0                    0x0A
#define LS_DATA_IR_1                    0x0B
#define LS_DATA_IR_2                    0x0C
#define LS_DATA_GREEN_0                 0x0D
#define LS_DATA_GREEN_1                 0x0E
#define LS_DATA_GREEN_2                 0x0F
#define LS_DATA_BLUE_0                  0x10
#define LS_DATA_BLUE_1                  0x11
#define LS_DATA_BLUE_2                  0x12
#define LS_DATA_RED_0                   0x13
#define LS_DATA_RED_1                   0x14
#define LS_DATA_RED_2                   0x15
#define INT_CFG                         0x19
#define INT_PERSISTANCE                 0x1A
#define LS_THRES_UP_0                   0x21
#define LS_THRES_UP_1                   0x22
#define LS_THRES_UP_2                   0x23
#define LS_THRES_LOW_0                  0x24
#define LS_THRES_LOW_1                  0x25
#define LS_THRES_LOW_2                  0x26
#define LS_THRES_VAR                    0x27

/** MAIN_CTRL */
#define SW_RESET                        0x10
#define LS_EN                           0x02
#define CS_ON                           0x04 //default ALS+IR

/** LS_MEAS_RATE */
#define LS_RESOLUTION_20B_400MS         0x00
#define LS_RESOLUTION_19B_200MS         0x10
#define LS_RESOLUTION_18B_100MS         0x20
#define LS_RESOLUTION_17B_50MS          0x30
#define LS_RESOLUTION_16B_25MS          0x40
#define LS_RESOLUTION_13B_3_125MS       0x50

#define LS_MES_RATE_25MS                0x00
#define LS_MES_RATE_50MS                0x01
#define LS_MES_RATE_100MS               0x02
#define LS_MES_RATE_200MS               0x03
#define LS_MES_RATE_500MS               0x04
#define LS_MES_RATE_1000MS              0x05
#define LS_MES_RATE_2001MS              0x06
#define LS_MES_RATE_2002MS              0x07

/** LS_GAIN */
#define LS_GAIN1                        0x00
#define LS_GAIN3                        0x01
#define LS_GAIN6                        0x02
#define LS_GAIN9                        0x03
#define LS_GAIN18                       0x04

/** MAIN_STATUS */
#define POWER_ON_STATUS                 0x20
#define LS_INTERR_STATUS                0x10
#define LS_DATA_STATUS                  0x08

/** INT_CFG */
#define LS_INT_IRCH                     0x00
#define LS_INT_ALS_GCH                  0x10
#define LS_INT_RCH                      0x20
#define LS_INT_BCH                      0x30
#define LS_VAR_MODE                     0x08
#define LS_INT_EN                       0x04

/** LS_PERSISTANCE */
#define LS_PERSIST_1                    0x00 
#define LS_PERSIST_2                    0x10 
#define LS_PERSIST_3                    0x20 
#define LS_PERSIST_4                    0x30  
#define LS_PERSIST_5                    0x40 
#define LS_PERSIST_6                    0x50  
#define LS_PERSIST_7                    0x60 
#define LS_PERSIST_8                    0x70  
#define LS_PERSIST_9                    0x80 
#define LS_PERSIST_10                   0x90  
#define LS_PERSIST_11                   0xA0 
#define LS_PERSIST_12                   0xB0  
#define LS_PERSIST_13                   0xC0 
#define LS_PERSIST_14                   0xD0  
#define LS_PERSIST_15                   0xE0 
#define LS_PERSIST_16                   0xF0 



#ifdef __cplusplus
}
#endif

#endif /*_APDS_9251_REGS_H_*/