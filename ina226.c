/**
 * Copyright (c) 2017 Tara Keeling
 * 
 * This software is released under the MIT License.
 * https://opensource.org/licenses/MIT
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <driver/i2c.h>

#include "ina226.h"

#define NullCheck( ptr, retexpr ) { \
    if ( ptr == NULL ) { \
        printf( "%s: %s == NULL\n", __FUNCTION__, #ptr ); \
        retexpr; \
    }; \
}

#define RangeCheck( value, min, max, retexpr ) { \
    if ( value < min || value > max ) { \
        printf( "ERROR %s: %s out of range. Got %d, expected [%d to %d]\n", __FUNCTION__, #value, value, min, max ); \
        retexpr; \
    } \
}

static bool INA226_SetRegisterPointer( struct INA226_Device* Device, INA226_Reg Register ) {
    NullCheck( Device, return false );
    NullCheck( Device->WriteBytesFn, return false );

    return ( Device->WriteBytesFn( Device->Address, ( const uint8_t* ) &Register, 1 ) == 1 ) ? true : false;
}

bool INA226_WriteReg( struct INA226_Device* Device, INA226_Reg Register, uint16_t Value ) {
    uint8_t Command[ ] = {
        ( uint8_t ) Register,
        Value >> 8,
        Value & 0xFF
    };

    NullCheck( Device, return false );
    NullCheck( Device->WriteBytesFn, return false );

    return ( Device->WriteBytesFn( Device->Address, ( const uint8_t* ) Command, sizeof( Command ) ) == sizeof( Command ) ) ? true : false;
}

uint16_t INA226_ReadReg16( struct INA226_Device* Device, INA226_Reg Register ) {
    uint16_t Value = 0;

    NullCheck( Device, return 0 );
    NullCheck( Device->WriteBytesFn, return 0 );
    NullCheck( Device->ReadBytesFn, return 0 );

    if ( INA226_SetRegisterPointer( Device, Register ) == true ) {
        if ( Device->ReadBytesFn( Device->Address, ( uint8_t* ) &Value, sizeof( uint16_t ) ) == sizeof( uint16_t ) ) {
            return ( Value >> 8 ) | ( Value << 8 );
        }
    }

    return 0;
}

uint16_t INA226_GetManufacturerId( struct INA226_Device* Device ) {
    return INA226_ReadReg16( Device, INA226_Reg_ManufacturerId );
}

uint16_t INA226_GetDieId( struct INA226_Device* Device ) {
    return INA226_ReadReg16( Device, INA226_Reg_DieId );
}

uint16_t INA226_ReadConfig( struct INA226_Device* Device ) {
    return INA226_ReadReg16( Device, INA226_Reg_Cfg );
}

void INA226_WriteConfig( struct INA226_Device* Device, uint16_t Config ) {
    INA226_WriteReg( Device, INA226_Reg_Cfg, Config );
}

INA226_AveragingMode INA226_GetAveragingMode( struct INA226_Device* Device ) {
    uint16_t CurrentConfig = 0;

    NullCheck( Device, return -1 );

    CurrentConfig = INA226_ReadConfig( Device );
    CurrentConfig>>= INA226_CFG_AveragingOffset;
    CurrentConfig &= 0x07;

    return ( INA226_AveragingMode ) CurrentConfig;
}

void INA226_SetAveragingMode( struct INA226_Device* Device, INA226_AveragingMode Mode ) {
    uint16_t CurrentConfig = 0;

    NullCheck( Device, return );
    RangeCheck( Mode, 0, INA226_Num_Averages, return );

    CurrentConfig = INA226_ReadConfig( Device );
    CurrentConfig &= ~INA226_CFG_AveragingMask;
    CurrentConfig |= ( Mode << INA226_CFG_AveragingOffset );

    INA226_WriteReg( Device, INA226_Reg_Cfg, CurrentConfig );
}

INA226_ConversionTime INA226_GetBusVoltageConversionTime( struct INA226_Device* Device ) {
    uint16_t CurrentConfig = 0;

    NullCheck( Device, return -1 );

    CurrentConfig = INA226_ReadConfig( Device );
    CurrentConfig>>= INA226_CFG_BusVoltageTimeOffset;
    CurrentConfig&= 0x07;

    return ( INA226_ConversionTime ) CurrentConfig;    
}

void INA226_SetBusVoltageConversionTime( struct INA226_Device* Device, INA226_ConversionTime ConversionTime ) {
    uint16_t CurrentConfig = 0;

    NullCheck( Device, return );
    RangeCheck( ConversionTime, 0, INA226_Num_ConversionTimes, return );

    CurrentConfig = INA226_ReadConfig( Device );
    CurrentConfig &= ~INA226_CFG_BusVoltageTimeMask;
    CurrentConfig |= ( ConversionTime << INA226_CFG_BusVoltageTimeOffset );

    INA226_WriteReg( Device, INA226_Reg_Cfg, CurrentConfig );
}

INA226_ConversionTime INA226_GetShuntVoltageConversionTime( struct INA226_Device* Device ) {
    uint16_t CurrentConfig = 0;

    NullCheck( Device, return -1 );

    CurrentConfig = INA226_ReadConfig( Device );
    CurrentConfig>>= INA226_CFG_ShuntVoltageTimeOffset;
    CurrentConfig&= 0x07;

    return ( INA226_ConversionTime ) CurrentConfig;     
}

void INA226_SetShuntVoltageConversionTime( struct INA226_Device* Device, INA226_ConversionTime ConversionTime ) {
    uint16_t CurrentConfig = 0;

    NullCheck( Device, return );
    RangeCheck( ConversionTime, 0, INA226_Num_ConversionTimes, return );

    CurrentConfig = INA226_ReadConfig( Device );
    CurrentConfig &= ~INA226_CFG_ShuntVoltageTimeMask;
    CurrentConfig |= ( ConversionTime << INA226_CFG_ShuntVoltageTimeOffset );

    INA226_WriteReg( Device, INA226_Reg_Cfg, CurrentConfig );
}

INA226_Mode INA226_GetOperatingMode( struct INA226_Device* Device ) {
    uint16_t CurrentConfig = 0;

    NullCheck( Device, return -1 );

    CurrentConfig = INA226_ReadConfig( Device );
    CurrentConfig&= 0x07;

    return ( INA226_Mode ) CurrentConfig;        
}

void INA226_SetOperatingMode( struct INA226_Device* Device, INA226_Mode Mode ) {
    uint16_t CurrentConfig = 0;

    NullCheck( Device, return );
    RangeCheck( Mode, 0, INA226_Num_Modes, return );

    CurrentConfig = INA226_ReadConfig( Device );
    CurrentConfig &= ~INA226_CFG_ModeMask;
    CurrentConfig |= Mode;

    INA226_WriteReg( Device, INA226_Reg_Cfg, CurrentConfig );    
}

/* Returns the shunt voltage in millivolts */
float INA226_GetShuntVoltage( struct INA226_Device* Device ) {
    float Result = 0.0f;

    Result = ( float ) ( ( int16_t ) INA226_ReadReg16( Device, INA226_Reg_ShuntVoltage ) );
    Result = Result * Device->ShuntVoltage_LSB;

    return Result;
}

/* Returns the voltage (in millivolts) of VBUS */
float INA226_GetBusVoltage( struct INA226_Device* Device ) {
    float Data = 0.0f;

    Data = ( float ) INA226_ReadReg16( Device, INA226_Reg_BusVolage );
    Data = Data * Device->BusVoltage_LSB;

    return Data;
}

/* Returns the current flowing in microamps */
float INA226_GetCurrent( struct INA226_Device* Device ) {
    float Data = 0.0f;

    Data = ( float ) ( ( int16_t ) INA226_ReadReg16( Device, INA226_Reg_Current ) );
    Data = Data * Device->Current_LSB;

    return Data;
}

float INA226_GetPower( struct INA226_Device* Device ) {
    float Data = 0.0f;

    Data = ( float ) ( ( int16_t ) INA226_ReadReg16( Device, INA226_Reg_Power ) );
    Data = Data * Device->Current_LSB;

    return Data * ( ( float ) 25 );
}

void INA226_Reset( struct INA226_Device* Device ) {
    NullCheck( Device, return );
    INA226_WriteConfig( Device, INA226_ReadConfig( Device ) | INA226_CFG_Reset );
}

static void INA226_Calibrate_FP( struct INA226_Device* Device, int RShuntInMilliOhms, int MaxCurrentInAmps ) {
    float RShunt = ( ( float ) RShuntInMilliOhms ) / 1000.0f;
    float Current_LSB = 0.0f;
    float Cal = 0.0f;

    /* Somehow converting amperes to microamperes makes this work.
     * At least at the current point in time my head is going to explode figuring this out
     * but for now "Just Works(tm)" is good enough.
     */
    Current_LSB = ( ( float ) MaxCurrentInAmps * 1000000 ) / 32768.0f;
    Cal = ( 0.00512f / ( Current_LSB * RShunt ) ) * 1000000;

    Device->Current_LSB = Current_LSB;
    Device->CalibrationValue = Cal;
    Device->ShuntVoltage_LSB = 2.5f;
    Device->BusVoltage_LSB = 1.25f;

    INA226_WriteReg( Device, INA226_Reg_Calibration, ( uint16_t ) Cal );
}

void INA226_Calibrate( struct INA226_Device* Device, int RShuntInMilliOhms, int MaxCurrentInAmps ) {
    NullCheck( Device, return );
    INA226_Calibrate_FP( Device, RShuntInMilliOhms, MaxCurrentInAmps );
}

bool INA226_Init( struct INA226_Device* Device, int I2CAddress, int RShuntInMilliOhms, int MaxCurrentInAmps, INAWriteBytes WriteBytesFn, INAReadBytes ReadBytesFn ) {
    const uint16_t ConfigRegisterAfterReset = 0x4127;

    NullCheck( WriteBytesFn, return false );
    NullCheck( ReadBytesFn, return false );
    NullCheck( Device, return false );
    
    memset( Device, 0, sizeof( struct INA226_Device ) );

    if ( I2CAddress > 0 ) {
        Device->WriteBytesFn = WriteBytesFn;
        Device->ReadBytesFn = ReadBytesFn;
        Device->Address = I2CAddress;

        INA226_Reset( Device );

        /* Check to see if we can actually talk to the device, if we can then the initial
         * value for the config register should be 0x4127 after a reset.
         */
        if ( INA226_ReadConfig( Device ) == ConfigRegisterAfterReset ) {
            INA226_Calibrate( Device, RShuntInMilliOhms, MaxCurrentInAmps );

            return true;
        }
    }

    return false;
}

static uint16_t INA226_SetAlertLimit( struct INA226_Device* Device, float Value ) {
    uint16_t Old = INA226_ReadReg16( Device, INA226_Reg_AlertLimit );

    NullCheck( Device, return 0 );
    INA226_WriteReg( Device, INA226_Reg_AlertLimit, Value );

    return Old;
}

float INA226_SetAlertLimit_BusVoltage( struct INA226_Device* Device, float BusVoltageInMV ) {
    float OldLimit = 0.0f;

    NullCheck( Device, return 0.0f );

    OldLimit = ( float ) INA226_SetAlertLimit( Device, BusVoltageInMV * Device->BusVoltage_LSB );
    OldLimit/= Device->BusVoltage_LSB;

    return OldLimit;
}
