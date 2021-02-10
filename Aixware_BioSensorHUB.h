#ifndef _AIXWARE_BIOSENSORHUB_LIBRARY_H_
#define _AIXWARE_BIOSENSORHUB_LIBRARY_H_

#include <Wire.h>
#include <Arduino.h>

class Aixware_BioSensorHUB
{
  public:  

    Aixware_BioSensorHUB(uint16_t, uint16_t); 

    uint8_t begin( TwoWire &wirePort = Wire);

    uint8_t read_status_HUB();
 
    uint8_t config_HR_sensor();
  
    void read_HR_sensor(uint8_t array[24]);

    uint8_t read_samples_HR_sensor();

    uint8_t read_address_HUB();

    uint8_t SetSpO2coefficients_C();

    uint8_t readDeviceMode();

    uint8_t readPartID();

    uint8_t readsensormode();

    uint8_t readdacc();

    //Temp sensor
    uint8_t gen_call_temp();

    uint8_t enable_eeprom_temp();

    uint8_t calculate_constants_temp_sensor();

    float read_obj_temp();

    float read_amb_temp();
    
    
  private:   

    uint8_t _reset_pin;
    uint8_t _mfio_pin;
    uint8_t _address_hr_sensor;
    uint8_t _address_temp_sensor;

    float const_k=0;
    float temp_M_float=0;

    float ambient_temperature=0;
    float object_temperature=0;

    uint32_t temp_M=0;
    uint32_t temp_PTAT25=0;
    uint32_t lookup=0;
    uint32_t temp_U0=0;
    uint32_t temp_Tobj1=0;

    TwoWire *_i2cPort;
 
    uint8_t write_byte_i2c(uint8_t, uint8_t, uint8_t);
    uint8_t write_byte_i2c(uint8_t, uint8_t, uint8_t, uint8_t);
    uint8_t write_byte2_i2c(uint8_t, uint8_t, uint8_t, uint8_t);
    uint8_t write_byte3_i2c(uint8_t , uint8_t ,uint8_t , uint8_t ,uint8_t );
    uint8_t write_byte_i2c(uint8_t, uint8_t, uint8_t, uint16_t);
    uint8_t read_byte_i2c(uint8_t, uint8_t); 
    uint8_t read_byte_i2c(uint8_t, uint8_t, uint8_t); 
    uint8_t read_byte_i2c_temp(uint8_t); 

    uint8_t* ReadFifo(uint8_t, uint8_t, uint8_t array[]);
};
#endif
