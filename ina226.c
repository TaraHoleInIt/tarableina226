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

static const int USE_THIS_I2C_PORT = 1;

static bool INA226_SetRegisterPointer( struct INA226_Device* Device, INA226_Reg Register ) {
    i2c_cmd_handle_t CommandHandle = NULL;

    RangeCheck( Register, INA226_Reg_Cfg, INA226_Reg_DieId, return false );
    NullCheck( Device, return false );

    if ( ( CommandHandle = i2c_cmd_link_create( ) ) ) {
        ESP_ERROR_CHECK( i2c_master_start( CommandHandle ) );
            ESP_ERROR_CHECK( i2c_master_write_byte( CommandHandle, ( Device->Address << 1 ) | I2C_MASTER_WRITE, true ) );
            ESP_ERROR_CHECK( i2c_master_write_byte( CommandHandle, ( uint8_t ) Register, true ) );
        ESP_ERROR_CHECK( i2c_master_stop( CommandHandle ) );

        ESP_ERROR_CHECK( i2c_master_cmd_begin( USE_THIS_I2C_PORT, CommandHandle, pdMS_TO_TICKS( 1000 ) ) );
        i2c_cmd_link_delete( CommandHandle );

        return true;
    }

    return false;
}

bool INA226_WriteReg( struct INA226_Device* Device, INA226_Reg Register, uint16_t Value ) {
    i2c_cmd_handle_t CommandHandle = NULL;

    RangeCheck( Register, INA226_Reg_Cfg, INA226_Reg_DieId, return false );
    NullCheck( Device, return false );
    
    if ( ( CommandHandle = i2c_cmd_link_create( ) ) ) {
        ESP_ERROR_CHECK( i2c_master_start( CommandHandle ) );
            ESP_ERROR_CHECK( i2c_master_write_byte( CommandHandle, ( Device->Address << 1 ) | I2C_MASTER_WRITE, true ) );
            ESP_ERROR_CHECK( i2c_master_write_byte( CommandHandle, ( uint8_t ) Register, true ) );
            ESP_ERROR_CHECK( i2c_master_write_byte( CommandHandle, ( uint8_t ) ( Value >> 8 ), true ) );
            ESP_ERROR_CHECK( i2c_master_write_byte( CommandHandle, ( uint8_t ) Value, true ) );
        ESP_ERROR_CHECK( i2c_master_stop( CommandHandle ) );

        ESP_ERROR_CHECK( i2c_master_cmd_begin( USE_THIS_I2C_PORT, CommandHandle, pdMS_TO_TICKS( 1000 ) ) );
        i2c_cmd_link_delete( CommandHandle );

        return true;
    }

    return false;
}

uint16_t INA226_ReadReg16( struct INA226_Device* Device, INA226_Reg Register ) {
    i2c_cmd_handle_t CommandHandle = NULL;
    uint8_t Value_lo = 0;
    uint8_t Value_hi = 0;

    RangeCheck( Register, INA226_Reg_Cfg, INA226_Reg_DieId, return false );
    NullCheck( Device, return 0xBAAD );

    if ( INA226_SetRegisterPointer( Device, Register ) == true ) {
        vTaskDelay( pdMS_TO_TICKS( 1 ) );

        if ( ( CommandHandle = i2c_cmd_link_create( ) ) ) {
            ESP_ERROR_CHECK( i2c_master_start( CommandHandle ) );
                ESP_ERROR_CHECK( i2c_master_write_byte( CommandHandle, ( Device->Address << 1 ) | I2C_MASTER_READ, true ) );
                ESP_ERROR_CHECK( i2c_master_read_byte( CommandHandle, &Value_hi, false ) );
                ESP_ERROR_CHECK( i2c_master_read_byte( CommandHandle, &Value_lo, false ) );
            ESP_ERROR_CHECK( i2c_master_stop( CommandHandle ) );

            ESP_ERROR_CHECK( i2c_master_cmd_begin( USE_THIS_I2C_PORT, CommandHandle, pdMS_TO_TICKS( 1000 ) ) );
            i2c_cmd_link_delete( CommandHandle );

            return ( ( Value_hi << 8 ) | Value_lo );
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
ina_value INA226_GetShuntVoltage( struct INA226_Device* Device ) {
    ina_value Result = ( ina_value ) 0;

    Result = ( ina_value ) ( ( int16_t ) INA226_ReadReg16( Device, INA226_Reg_ShuntVoltage ) );
    Result = Result * Device->ShuntVoltage_LSB;

#if defined CONFIG_INA226_USE_FP
    return Result;
#else
    return Result / 10;
#endif
}

/* Returns the voltage (in millivolts) of VBUS */
ina_value INA226_GetBusVoltage( struct INA226_Device* Device ) {
    ina_value Data = ( ina_value ) 0;

    Data = ( ina_value ) INA226_ReadReg16( Device, INA226_Reg_BusVolage );
    Data = Data * Device->BusVoltage_LSB;

#if defined CONFIG_INA226_USE_FP
    return Data;
#else
    /* Scale the data back down to millivolts */
    return Data / 100;
#endif
}

/* Returns the current flowing in microamps */
ina_value INA226_GetCurrent( struct INA226_Device* Device ) {
    ina_value Data = ( ina_value ) 0;

    Data = ( ina_value ) ( ( int16_t ) INA226_ReadReg16( Device, INA226_Reg_Current ) );
    Data = Data * Device->Current_LSB;

    return Data;
}

ina_value INA226_GetPower( struct INA226_Device* Device ) {
    ina_value Data = ( ina_value ) 0;

    Data = ( ina_value ) ( ( int16_t ) INA226_ReadReg16( Device, INA226_Reg_Power ) );
    Data = Data * Device->Current_LSB;

    return Data * ( ( ina_value ) 25 );
}

void INA226_Reset( struct INA226_Device* Device ) {
    NullCheck( Device, return );
    INA226_WriteConfig( Device, INA226_ReadConfig( Device ) | INA226_CFG_Reset );
}

#ifdef CONFIG_INA226_USE_FP
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

    printf( "Current_LSB: %f\n", Current_LSB );
    printf( "Cal: %f\n", Cal );
    printf( "dCal: %d\n", ( uint16_t ) Cal );

    INA226_WriteReg( Device, INA226_Reg_Calibration, ( uint16_t ) Cal );
}
#else
static void INA226_Calibrate_INT( struct INA226_Device* Device, int RShuntInMilliOhms, int MaxCurrentInAmps ) {
    uint64_t InternalScaleValue = 0;
    uint64_t Current_LSB = 0;
    uint64_t Cal = 0;

    /* Example input:
     * RShuntInMilliOhms: 100 (0.1 ohms) - value of shunt resistor
     * MaxCurrentInAmps: 4 - maximum (expected) current
     * 
     * Calibration:
     * Current_LSB: Maximum current in amps / 2^15  (Current_LSB is microamps per bit?)
     * Calibration: 0.00512 / ( Current_LSB * Value of shunt resistor )
     */

    /* Convert amps into microamps and divide it by our scale range (2^15) 
     * This makes it a whole number and much easier to work with.
     */
    Current_LSB = ( ( MaxCurrentInAmps * 1000000 ) / 32768 ); // 122 uA/bit

    /* This is the fixed internal value of 0.00512 multiplied by 100,000
     * to make it a whole number.
     */
    InternalScaleValue = 512;

    /* MYSTERY AHEAD
     * BEWARE NON UNDERSTOOD MATH (by me)
     */
    Cal = Current_LSB * RShuntInMilliOhms;
    Cal = ( InternalScaleValue * 10000 ) / Cal;     /* Where does 10,000 come from and why does it fit? */

    Device->Current_LSB = ( int32_t ) Current_LSB;
    Device->CalibrationValue = ( uint16_t ) Cal;

    /* This is actually 1.25 millivolts but by multiplying by 100 we
    * get rid of the fractional component and remove the need for
    * floating point.
    */
    Device->BusVoltage_LSB = 125;

    /* 2.5 Microvolts multiplied by 10 to remove the fractional component */
    Device->ShuntVoltage_LSB = 25;

    INA226_WriteReg( Device, INA226_Reg_Calibration, Device->CalibrationValue );

    printf( "lsb: %d\ncal: %d\n", ( int ) Current_LSB, ( int ) Cal );
}
#endif

void INA226_Calibrate( struct INA226_Device* Device, int RShuntInMilliOhms, int MaxCurrentInAmps ) {
    NullCheck( Device, return );

#ifdef CONFIG_INA226_USE_FP
    INA226_Calibrate_FP( Device, RShuntInMilliOhms, MaxCurrentInAmps );
#else
    INA226_Calibrate_INT( Device, RShuntInMilliOhms, MaxCurrentInAmps );
#endif    
}

bool INA226_Init( struct INA226_Device* Device, int I2CAddress, int RShuntInMilliOhms, int MaxCurrentInAmps ) {
    const uint16_t ConfigRegisterAfterReset = 0x4127;

    NullCheck( Device, return false );
    
    memset( Device, 0, sizeof( struct INA226_Device ) );

    if ( I2CAddress > 0 ) {
        Device->Address = I2CAddress;

        /* Check to see if we can actually talk to the device, if we can then the initial
         * value for the config register should be 0x4127 after a reset.
         */
        if ( INA226_ReadConfig( Device ) == ConfigRegisterAfterReset ) {
            INA226_Reset( Device );
            INA226_Calibrate( Device, RShuntInMilliOhms, MaxCurrentInAmps );

            return true;
        }
    }

    return false;
}
