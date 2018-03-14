#ifndef _INA226_H_
#define _INA226_H_

#ifdef __cplusplus
extern "C" {
#endif

#if defined HAVE_CONFIG_H
#include "sdkconfig.h"
#endif

#if ! defined BIT
#define BIT( n ) ( 1 << n )
#endif

#if defined CONFIG_INA226_USE_FP
typedef float ina_value;
#else
typedef int32_t ina_value;
#endif

typedef size_t ( *INAWriteBytes ) ( int Address, const uint8_t* Buffer, size_t BytesToWrite );
typedef size_t ( *INAReadBytes ) ( int Address, uint8_t* Buffer, size_t BufferMaxLen );

struct INA226_Device {
    ina_value ShuntVoltage_LSB;
    ina_value BusVoltage_LSB;

    ina_value CalibrationValue;
    ina_value Current_LSB;

    INAWriteBytes WriteBytesFn;
    INAReadBytes ReadBytesFn;

    int Address;
};

typedef enum {
    INA226_Reg_Cfg = 0x00,
    INA226_Reg_ShuntVoltage,
    INA226_Reg_BusVolage,
    INA226_Reg_Power,
    INA226_Reg_Current,
    INA226_Reg_Calibration,
    INA226_Reg_AlertMask,
    INA226_Reg_AlertLimit,
    INA226_Reg_ManufacturerId = 0xFE,
    INA226_Reg_DieId
} INA226_Reg;

typedef enum {
    INA226_Averages_1 = 0,
    INA226_Averages_4,
    INA226_Averages_16,
    INA226_Averages_64,
    INA226_Averages_128,
    INA226_Averages_256,
    INA226_Averages_512,
    INA226_Averages_1024,
    INA226_Num_Averages = 7
} INA226_AveragingMode;

typedef enum {
    INA226_ConversionTime_140us = 0,
    INA226_ConversionTime_204us,
    INA226_ConversionTime_332us,
    INA226_ConversionTime_588us,
    INA226_ConversionTime_1_1ms,
    INA226_ConversionTime_2_116ms,
    INA226_ConversionTime_4_156ms,
    INA226_ConversionTime_8_244ms,
    INA226_Num_ConversionTimes = 7
} INA226_ConversionTime;

typedef enum {
    INA226_Mode_Shutdown = 0,
    INA226_Mode_ShuntVoltage_Triggered,
    INA226_Mode_BusVoltage_Triggered,
    INA226_Mode_ShuntAndBus_Triggered,
    INA226_Mode_Shutdown2,
    INA226_Mode_ShuntVoltage_Continuous,
    INA226_Mode_BusVoltage_Continuous,
    INA226_Mode_ShuntAndBus_Continuous,
    INA226_Num_Modes = 7
} INA226_Mode;

typedef enum {
    INA226_Alert_ShuntOverVoltage = BIT( 15 ),
    INA226_Alert_ShuntUnderVoltage = BIT( 14 ),
    INA226_Alert_BusOverVoltage = BIT( 13 ),
    INA226_Alert_BusUnderVoltage = BIT( 12 ),
    INA226_Alert_PowerOverLimit = BIT( 11 ),
    INA226_Alert_ConversionReady = BIT( 10 ),
    INA226_Alert_AlertFunctionFlag = BIT( 4 ),
    INA226_Alert_ConversionReadyFlag = BIT( 3 ),
    INA226_Alert_MathOverflowFlag = BIT( 2 ),
    INA226_Alert_AlertPolarity = BIT( 1 ),
    INA226_Alert_AlertLatchEnable = BIT( 0 )
} INA226_Alert;

#define INA226_CFG_Reset BIT( 15 )

#define INA226_CFG_AveragingMask ( BIT( 9 ) | BIT( 10 ) | BIT( 11 ) )
#define INA226_CFG_AveragingOffset 9

#define INA226_CFG_BusVoltageTimeMask ( BIT( 6 ) | BIT( 7 ) | BIT( 8 ) )
#define INA226_CFG_BusVoltageTimeOffset 6

#define INA226_CFG_ShuntVoltageTimeMask ( BIT( 3 ) | BIT( 4 ) | BIT( 5 ) )
#define INA226_CFG_ShuntVoltageTimeOffset 3

#define INA226_CFG_ModeMask ( BIT( 0 ) | BIT( 1 ) | BIT( 2 ) )

bool INA226_WriteReg( struct INA226_Device* Device, INA226_Reg Register, uint16_t Value );
uint16_t INA226_ReadReg16( struct INA226_Device* Device, INA226_Reg Register );

uint16_t INA226_GetManufacturerId( struct INA226_Device* Device );
uint16_t INA226_GetDieId( struct INA226_Device* Device );

uint16_t INA226_ReadConfig( struct INA226_Device* Device );
void INA226_WriteConfig( struct INA226_Device* Device, uint16_t Config );

INA226_AveragingMode INA226_GetAveragingMode( struct INA226_Device* Device );
void INA226_SetAveragingMode( struct INA226_Device* Device, INA226_AveragingMode Mode );

INA226_ConversionTime INA226_GetBusVoltageConversionTime( struct INA226_Device* Device );
void INA226_SetBusVoltageConversionTime( struct INA226_Device* Device, INA226_ConversionTime ConversionTime );

INA226_ConversionTime INA226_GetShuntVoltageConversionTime( struct INA226_Device* Device );
void INA226_SetShuntVoltageConversionTime( struct INA226_Device* Device, INA226_ConversionTime ConversionTime );

INA226_Mode INA226_GetOperatingMode( struct INA226_Device* Device );
void INA226_SetOperatingMode( struct INA226_Device* Device, INA226_Mode Mode );

ina_value INA226_GetShuntVoltage( struct INA226_Device* Device );
ina_value INA226_GetBusVoltage( struct INA226_Device* Device );
ina_value INA226_GetCurrent( struct INA226_Device* Device );
ina_value INA226_GetPower( struct INA226_Device* Device );

bool INA226_Init( struct INA226_Device* Device, int I2CAddress, int RShuntInMilliOhms, int MaxCurrentInAmps, INAWriteBytes WriteBytesFn, INAReadBytes ReadBytesFn );
void INA226_Reset( struct INA226_Device* Device );
void INA226_Calibrate( struct INA226_Device* Device, int RShunt, int MaxCurrentInMilliamps );

ina_value INA226_SetAlertLimit_BusVoltage( struct INA226_Device* Device, ina_value BusVoltageInMV );

#ifdef __cplusplus
}
#endif

#endif
