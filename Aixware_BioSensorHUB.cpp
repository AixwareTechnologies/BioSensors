#include "Aixware_BioSensorHUB.h"

Aixware_BioSensorHUB::Aixware_BioSensorHUB(uint16_t resetPin, uint16_t mfioPin) { 
  
  _reset_pin = resetPin; 
  _mfio_pin = mfioPin;
  _address_hr_sensor = 0x55;    //0xAA write adrress, 0xAB read address
  _address_temp_sensor = 0x0C;  //0x19 write address, 0x18 read address  

  pinMode(_mfio_pin, OUTPUT); 
  pinMode(_reset_pin, OUTPUT); // Set these pins as output
  
}

uint8_t Aixware_BioSensorHUB::begin( TwoWire &wirePort ) {

  _i2cPort = &wirePort;

  digitalWrite(_mfio_pin, HIGH); 
  digitalWrite(_reset_pin, LOW); 
  delay(200); 
  digitalWrite(_reset_pin, HIGH); 
  delay(2000);

  //Read device mode
  uint8_t responseByte = read_byte_i2c(0x02, 0x00); // 0x00 only possible Index Byte.
  return responseByte; 
}

uint8_t Aixware_BioSensorHUB::read_status_HUB(){
  
  uint8_t hubstatus;
  uint8_t status;

  digitalWrite(_mfio_pin, LOW);
  delay(50);

  _i2cPort->beginTransmission(_address_hr_sensor);
  _i2cPort->write(0x00);    
  _i2cPort->write(0x00);    
  _i2cPort->endTransmission();
  delay(10);
  
  _i2cPort->requestFrom(_address_hr_sensor, 2); 
  status = _i2cPort->read();

  hubstatus = _i2cPort->read(); 

  digitalWrite(_mfio_pin, HIGH);
  delay(50);

  return hubstatus; 

}

uint8_t Aixware_BioSensorHUB::config_HR_sensor(){

  uint8_t cont_tries=0, status = 0;
  Serial.println ("Configuring...");

//--------------------------------------------------------------------------------------
//AGC Control algorithm
//   cont_tries=0;
//   do {
//     status = SetSpO2coefficients_C();
//     Serial.print("Set coe: ");
//     Serial.println(status);
//     //delay(200);
//     cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );

//   cont_tries=0;
//   //Set output mode to sensor data 
//   do {
//     status = writeByte(0x10, 0x00, 0x03);
//     Serial.print("output mode: ");
//     Serial.println(status);
//     //if (status != SUCCESS) return status;
//     //delay(200);
//     cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );

//   cont_tries=0;
//   //Set sensor hub interrupt threshold
//   do {
//   status = writeByte(0x10, 0x01, 0x05);
//   Serial.print("threshold: ");
//   Serial.println(status);
//   //if (status != SUCCESS) return status;
//   //delay(200);
//    cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );

//   cont_tries=0;
//   // report rate (Error 1: command invalid)
//   do { 
//     status = writeByte(0x10, 0x02, 0x01);
//     Serial.print("report rate: ");
//     Serial.println(status);
//     //if (status != SUCCESS) return status;
//     //delay(200);
//     cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );

// cont_tries=0;
//   // report rate (Error 1: command invalid)
//   do { 
//     status = writeByte2(0x44, 0x04, 0x01, 0x00);
//     Serial.print("report rate: ");
//     Serial.println(status);
//     //if (status != SUCCESS) return status;
//     //delay(200);
//     cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );

//   cont_tries=0;
//   // Algorithm operation (Error 2)
//   do {
//     status = writeByte2(0x50, 0x07, 0x0A, 0x00);
//     Serial.print("Alg op mode: ");
//     Serial.println(status);
//     //if (status != SUCCESS) return status;
//     //delay(200);
//     cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );

//   cont_tries=0;
//   // Enable AEC (Error 2)
//   do {
//     status = writeByte2(0x50, 0x07, 0x0B, 0x01);
//     Serial.print("AEC: ");
//     Serial.println(status);
//     //if (status != SUCCESS) return status;
//     //delay(200);
//     cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );

//   cont_tries=0;
//   //Auto PD (Error 2)
//   do {
//     status = writeByte2(0x50, 0x07, 0x12, 0x00);
//     Serial.print("Auto PD: ");
//     Serial.println(status);
//     //if (status != SUCCESS) return status;
//     //delay(200);
//     cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );

//   cont_tries=0;
//   //Enable SCD (error 2)
//   do {
//     status = writeByte2(0x50, 0x07, 0x0C, 0x00);
//     Serial.print("SCD: ");
//     Serial.println(status);
//     //if (status != SUCCESS) return status;
//     //delay(300);
//     cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );

//   cont_tries=0;
//   //Enable Set AGC Target 
//   do{
//     status = writeByte3(0x50, 0x07, 0x11, 0x00, 0x64);
//     Serial.print("Set Target: ");
//     Serial.println(status);
//     //if (status != SUCCESS) return status;
//     //delay(300);
//     cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );

//   cont_tries=0;
//   //Enable algorithm (error 2)
//   do {
//     status = writeByte(0x52, 0x07, 0x01);
//     Serial.print("Enable alg: ");
//     Serial.println(status);
//     //if (status != SUCCESS) return status;
//     //delay(200);
//     cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );


//--------------------------------------------------------------------------------------
//AEC MDOE

  //Read device mode
  status = read_byte_i2c(0x02, 0x00); 
  if (status == 0x00) Serial.println("app mode selected.");
  else if (status == 0x02) Serial.println("reset mode selected.");
  else if (status == 0x08) Serial.println("boot mode selected.");

  cont_tries=0;
  do {
    status = SetSpO2coefficients_C();
    Serial.print("Set coe: ");
    Serial.println(status);
    if (status != 0x00) return status;
    cont_tries++;
  } while (status != 0x00 && cont_tries<3 );

  cont_tries=0;
  //Set output mode to sensor data 
  do {
    status = write_byte_i2c(0x10, 0x00, 0x02);
    Serial.print("output mode: ");
    Serial.println(status);
    if (status != 0x00) return status;
    cont_tries++;
  } while (status != 0x00 && cont_tries<3 );

  cont_tries=0;
  //Set sensor hub interrupt threshold
  do {
  status = write_byte_i2c(0x10, 0x01, 0x01);
  Serial.print("threshold: ");
  Serial.println(status);
  if (status != 0x00) return status;
   cont_tries++;
  } while (status != 0x00 && cont_tries<3 );

  cont_tries=0;
  // report rate 
  do { 
   status = write_byte_i2c(0x10, 0x02, 0x10); 
    Serial.print("report rate: ");
    Serial.println(status);
    if (status != 0x00) return status;
    cont_tries++;
  } while (status != 0x00 && cont_tries<3 );

  cont_tries=0;
  // Enable accelerometer
  do {
    status = write_byte2_i2c(0x44, 0x04, 0x01, 0x00);
    Serial.print("acc: ");
    Serial.println(status);
    if (status != 0x00) return status;
    cont_tries++;
  } while (status != 0x00 && cont_tries<3 );

  cont_tries=0;
  // Algorithm operation 
  do {
    status = write_byte2_i2c(0x50, 0x07, 0x0A, 0x00);
    Serial.print("Alg op mode: ");
    Serial.println(status);
    if (status != 0x00) return status;
    cont_tries++;
  } while (status != 0x00 && cont_tries<3 );

  cont_tries=0;
  // Enable AEC 
  do {
    status = write_byte2_i2c(0x50, 0x07, 0x0B, 0x01);
    Serial.print("AEC: ");
    Serial.println(status);
    if (status != 0x00) return status;
    cont_tries++;
  } while (status != 0x00 && cont_tries<3 );

  cont_tries=0;
  //Auto PD 
  do {
    status = write_byte2_i2c(0x50, 0x07, 0x12, 0x01);
    Serial.print("Auto PD: ");
    Serial.println(status);
    if (status != 0x00) return status;
    cont_tries++;
  } while (status != 0x00 && cont_tries<3 );

  cont_tries=0;
  //Enable SCD (error 2)
  do {
    status = write_byte2_i2c(0x50, 0x07, 0x0C, 0x01);
    Serial.print("SCD: ");
    Serial.println(status);
    if (status != 0x00) return status;
    cont_tries++;
  } while (status != 0x00 && cont_tries<3 );

  cont_tries=0;
  //Enable algorithm (error 2)
  do {

    status = write_byte_i2c(0x52, 0x07, 0x01);
    Serial.print("Enable alg: ");
    Serial.println(status);
    if (status != 0x00) return status;
    cont_tries++;
  } while (status != 0x00 && cont_tries<3 );
 
//----------------------------------------------------------------------------------------------------
//RAW DATA 
//   cont_tries=0;
//   //Set output mode to sensor data 
//   do {
//     status = writeByte(0x10, 0x00, 0x01);
//     Serial.print("output mode: ");
//     Serial.println(status);
//     if (status != SUCCESS) return status;
//     cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );

//   cont_tries=0;
//   //Set output mode to sensor data 
//   do {
//     status = writeByte(0x10, 0x01, 0x08);
//     Serial.print("output mode: ");
//     Serial.println(status);
//     if (status != SUCCESS) return status;
//     cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );


//    cont_tries=0;
//   //Set output mode to sensor data 
//   do {
//     status = writeByte2(0x44, 0x00, 0x01, 0x00);
//     Serial.print("output mode: ");
//     Serial.println(status);
//     if (status != SUCCESS) return status;
//     cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );

//   cont_tries=0;
//   //Set sensor hub interrupt threshold
//   do {
//   status = writeByte2(0x44, 0x04, 0x01, 0x00);
//   Serial.print("threshold: ");
//   Serial.println(status);
//   if (status != SUCCESS) return status;
//    cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );

//   cont_tries=0;
//   // report rate (Error 1: command invalid)
//   do { 
//     status = writeByte2(0x40, 0x00, 0x11, 0x3F);
//     Serial.print("report rate: ");
//     Serial.println(status);
//     if (status != SUCCESS) return status;
//     cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );

//   cont_tries=0;
//   // Algorithm operation (Error 2)
//   do {
//     status = writeByte2(0x40, 0x00, 0x12, 0x18);
//     Serial.print("Alg op mode: ");
//     Serial.println(status);
//     if (status != SUCCESS) return status;
//     cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );

//   cont_tries=0;
//   // Enable AEC (Error 2)
//   do {
//     status = writeByte2(0x40, 0x00, 0x20, 0x21);
//     Serial.print("AEC: ");
//     Serial.println(status);
//     if (status != SUCCESS) return status;
//     cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );

//   cont_tries=0;
//   //Auto PD (Error 2)
//   do {
//     status = writeByte2(0x40, 0x00, 0x2A, 0x3C);
//     Serial.print("Auto PD: ");
//     Serial.println(status);
//     if (status != SUCCESS) return status;
//     cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );

//   cont_tries=0;
//   //Enable SCD (error 2)
//   do {
//     status = writeByte2(0x40, 0x00, 0x21, 0x03);
//     Serial.print("SCD: ");
//     Serial.println(status);
//     if (status != SUCCESS) return status;
//     cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );

// cont_tries=0;
//   //Enable SCD (error 2)
//   do {
//     status = writeByte2(0x40, 0x00, 0x22, 0x00);
//     Serial.print("SCD: ");
//     Serial.println(status);
//     if (status != SUCCESS) return status;
//     cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );

//   cont_tries=0;
//   //Enable SCD (error 2)
//   do {
//     status = writeByte2(0x40, 0x00, 0x23, 0x30);
//     Serial.print("SCD: ");
//     Serial.println(status);
//     if (status != SUCCESS) return status;
//     cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );

//    cont_tries=0;
//   //Enable SCD (error 2)
//   do {
//     status = writeByte2(0x40, 0x00, 0x24, 0x00);
//     Serial.print("SCD: ");
//     Serial.println(status);
//     if (status != SUCCESS) return status;
//     cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );

//    cont_tries=0;
//   //Enable SCD (error 2)
//   do {
//     status = writeByte2(0x40, 0x00, 0x25, 0x00);
//     Serial.print("SCD: ");
//     Serial.println(status);
//     if (status != SUCCESS) return status;
//     cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );

//    cont_tries=0;
//   //Enable SCD (error 2)
//   do {
//     status = writeByte2(0x40, 0x00, 0x2A, 0x3F);
//     Serial.print("SCD: ");
//     Serial.println(status);
//     if (status != SUCCESS) return status;
//     cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );

//   cont_tries=0;
//   //Enable SCD (error 2)
//   do {
//     status = writeByte2(0x40, 0x00, 0x09,0x7C);
//     Serial.print("SCD: ");
//     Serial.println(status);
//     if (status != SUCCESS) return status;
//     cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );

//   cont_tries=0;
//   //Enable SCD (error 2)
//   do {
//     status = writeByte2(0x40, 0x00, 0x0A,0x0E);
//     Serial.print("SCD: ");
//     Serial.println(status);
//     if (status != SUCCESS) return status;
//     cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );

//   cont_tries=0;
//   //Enable SCD (error 2)
//   do {
//     status = writeByte2(0x40, 0x00, 0x02,0xC6);
//     Serial.print("SCD: ");
//     Serial.println(status);
//     if (status != SUCCESS) return status;
//     cont_tries++;
//   } while (status != SUCCESS && cont_tries<3 );


  status = read_byte_i2c(0x51, 0x07, 0x0A); 
  if (status== 0x00)  Serial.println("Continuos HRM, Continuos SPO2");
  else if (status== 0x01)  Serial.println("Continuos HRM, one shot SPO2");
  else if (status== 0x02)  Serial.println("Continuos HRM");
  else if (status== 0x03)  Serial.println("Sampled HRM");
  else if (status== 0x04)  Serial.println("Sampled HRM, one shot SPO2");
  else if (status== 0x05)  Serial.println("Activity tracking");
  else if (status== 0x06)  Serial.println("SPO2 calibration");

  Serial.print("Succes!!");
  return 0x00; 
}

uint8_t Aixware_BioSensorHUB::readdacc() {

  digitalWrite(_mfio_pin, LOW);
  delay(10); 

  _i2cPort->beginTransmission(_address_hr_sensor);
  _i2cPort->write(0x41);    
  _i2cPort->write(0x04);    
  _i2cPort->write(0x0F);   
  _i2cPort->endTransmission();
  delay(50); 

  _i2cPort->requestFrom(_address_hr_sensor, 3); 
  //Primero se lee el estatus
  uint8_t statusByte = _i2cPort->read();

  //Despues se lee el valor
  uint8_t accdata = _i2cPort->read();

  uint8_t whois = _i2cPort->read();

  Serial.print("ACCEL:");
  Serial.print(accdata);
  Serial.print(", ");
  Serial.println(whois);

  digitalWrite(_mfio_pin, HIGH);
  delay(10); 

  return statusByte;
}


uint8_t Aixware_BioSensorHUB::readPartID(){

  digitalWrite(_mfio_pin, LOW);
  delay(100); 

  _i2cPort->beginTransmission(_address_hr_sensor);
  _i2cPort->write(0x41);    
  _i2cPort->write(0x00);    
  _i2cPort->write(0xFF);   
  _i2cPort->endTransmission();
  delay(50); 

  _i2cPort->requestFrom(_address_hr_sensor, 2); 

  //Primero se lee el estatus
  uint8_t status = _i2cPort->read();
  
  //Despues se lee el valor
  uint8_t Partid = _i2cPort->read();

  Serial.print(status);
  Serial.print(" ");
  Serial.print(Partid);
  Serial.print(" : ");
  if (Partid == 0x25)  Serial.println("MAX86141");
  else if (Partid == 0x24)  Serial.println("MAX86140");
  else if (Partid == 0x36)  Serial.println("MAXM86161");

  digitalWrite(_mfio_pin, HIGH);
  delay(100); 

  return status;

}

uint8_t Aixware_BioSensorHUB::readsensormode() {


    digitalWrite(_mfio_pin, LOW);
    delay(10); 

    _i2cPort->beginTransmission(_address_hr_sensor);
    _i2cPort->write(0x45);    
    _i2cPort->write(0x00);   
    _i2cPort->endTransmission();
    delay(50); 

    _i2cPort->requestFrom(_address_hr_sensor, 1); 

    //Primero se lee el estatus
    uint8_t statusByte = _i2cPort->read();

    Serial.print("Sensor mode: ");
    Serial.println(statusByte);
    
    digitalWrite(_mfio_pin, HIGH);
    delay(10); 

  return statusByte;  

  }

  
void Aixware_BioSensorHUB::read_HR_sensor(uint8_t array[24]){

  //Verificamos cuantos samples hay
  uint8_t samples =  read_samples_HR_sensor(); 

  //Leemos los 24 bytes del paquete de sensor que contiene puros datos del algoritmo
  ReadFifo(0x12, 0x01, 49, array); 

}
 
uint8_t Aixware_BioSensorHUB::read_samples_HR_sensor() {


  digitalWrite(_mfio_pin, LOW);
  delay(10); 

  _i2cPort->beginTransmission(_address_hr_sensor);
  _i2cPort->write(0x12);    
  _i2cPort->write(0x00);    
  _i2cPort->endTransmission();
  delay(50); 

  _i2cPort->requestFrom(_address_hr_sensor, 2); 
  //Primero se lee el estatus
  uint8_t status = _i2cPort->read();

  //Despues se lee el valor
  uint8_t samples2 = _i2cPort->read();

  Serial.print( "Samples: ");
  Serial.print(samples2);
  Serial.print( ", ");

  digitalWrite(_mfio_pin, HIGH);
  delay(10); 
  return samples2;
}

uint8_t Aixware_BioSensorHUB::read_address_HUB()
{

  //Read HUB version
  //Lectura
  digitalWrite(_mfio_pin, LOW);
  delay(10); 

  _i2cPort->beginTransmission(_address_hr_sensor);
  _i2cPort->write(0xFF);    
  _i2cPort->write(0x03);    
  _i2cPort->endTransmission();
  delay(50); 

  _i2cPort->requestFrom(_address_hr_sensor, 4); 
  //Primero se lee el estatus
  uint8_t statusByte = _i2cPort->read();

  //Despues se lee el valor
  uint8_t addressreaded1 = _i2cPort->read();
  uint8_t addressreaded2 = _i2cPort->read();
  uint8_t addressreaded3 = _i2cPort->read();

  Serial.print( "Address HUB: ");
  Serial.print(addressreaded1);
  Serial.print( "." );
  Serial.print(addressreaded2);
  Serial.print( "." );
  Serial.print(addressreaded3);
  Serial.print( " , " );
  Serial.print( "Estatus: ");
  Serial.println( statusByte );

  digitalWrite(_mfio_pin, HIGH);
  delay(10); 

  return 0x00; 
}


uint8_t Aixware_BioSensorHUB::SetSpO2coefficients_C() {


  digitalWrite(_mfio_pin, LOW);
  delay(10); 

  _i2cPort->beginTransmission(_address_hr_sensor);
  _i2cPort->write(0x50);    
  _i2cPort->write(0x07);    
  _i2cPort->write(0x00);    
  _i2cPort->write(0x00);    
  _i2cPort->write(0x00);    
  _i2cPort->write(0x00);    
  _i2cPort->write(0x00);    
  _i2cPort->write(0xFF);    
  _i2cPort->write(0xD7);    
  _i2cPort->write(0xFB);    
  _i2cPort->write(0xDD);    
  _i2cPort->write(0x00);    
  _i2cPort->write(0xAB);    
  _i2cPort->write(0x61);    
  _i2cPort->write(0xFE);    
  _i2cPort->endTransmission();
  delay(100); 

  _i2cPort->requestFrom(_address_hr_sensor, 1); 
  uint8_t status= _i2cPort->read();


  digitalWrite(_mfio_pin, HIGH);
  delay(10); 

  return status; 

}

uint8_t Aixware_BioSensorHUB::write_byte_i2c(uint8_t familyByte, uint8_t indexByte, uint8_t writeByte)
{

  digitalWrite(_mfio_pin, LOW);
  delay(10); 

  _i2cPort->beginTransmission(_address_hr_sensor);     
  _i2cPort->write(familyByte);    
  _i2cPort->write(indexByte);    
  _i2cPort->write(writeByte); 
  _i2cPort->endTransmission(); 

  if (familyByte == 0x52) delay(250); 
  else delay(50);

  _i2cPort->requestFrom(_address_hr_sensor, 1); 
  uint8_t status = _i2cPort->read(); 

  digitalWrite(_mfio_pin, HIGH);
  delay(10); 

  return status; 

}

uint8_t Aixware_BioSensorHUB::write_byte3_i2c(uint8_t familyByte, uint8_t indexByte,uint8_t valByte, uint8_t writeByte, uint8_t val1 )
{


  digitalWrite(_mfio_pin, LOW);
  delay(10); 

  _i2cPort->beginTransmission(_address_hr_sensor);     
  _i2cPort->write(familyByte);    
  _i2cPort->write(indexByte);
  _i2cPort->write(valByte);        
  _i2cPort->write(writeByte); 
  _i2cPort->write(val1); 
  _i2cPort->endTransmission(); 
  delay(0x10); 

  _i2cPort->requestFrom(_address_hr_sensor, 1); 
  uint8_t status = _i2cPort->read(); 


  digitalWrite(_mfio_pin, HIGH);
  delay(10); 

  return status; 

}

uint8_t Aixware_BioSensorHUB::write_byte2_i2c(uint8_t familyByte, uint8_t indexByte,uint8_t valByte, uint8_t writeByte)
{

  digitalWrite(_mfio_pin, LOW);
  delay(10); 

  _i2cPort->beginTransmission(_address_hr_sensor);     
  _i2cPort->write(familyByte);    
  _i2cPort->write(indexByte);
  _i2cPort->write(valByte);        
  _i2cPort->write(writeByte); 
  _i2cPort->endTransmission(); 
  delay(0x50); 

  _i2cPort->requestFrom(_address_hr_sensor, 1); 
  uint8_t statusByte = _i2cPort->read(); 

  digitalWrite(_mfio_pin, HIGH);
  delay(10); 

  return statusByte; 

}

uint8_t Aixware_BioSensorHUB::write_byte_i2c(uint8_t familyByte, uint8_t indexByte,uint8_t writeByte, uint8_t writeVal)
{

  digitalWrite(_mfio_pin, LOW);
  delay(10); 

  _i2cPort->beginTransmission(_address_hr_sensor);     
  _i2cPort->write(familyByte);    
  _i2cPort->write(indexByte);    
  _i2cPort->write(writeByte);    
  _i2cPort->write(writeVal);    
  _i2cPort->endTransmission(); 
  delay(0x50); 

  _i2cPort->requestFrom(_address_hr_sensor, 1); 
  uint8_t statusByte = _i2cPort->read(); 

  digitalWrite(_mfio_pin, HIGH);
  delay(10); 

  return statusByte; 

}

uint8_t Aixware_BioSensorHUB::read_byte_i2c(uint8_t familyByte, uint8_t indexByte )
{

  uint8_t returnByte;
  uint8_t status;

  digitalWrite(_mfio_pin, LOW);
  delay(10);

  _i2cPort->beginTransmission(_address_hr_sensor);
  _i2cPort->write(familyByte);    
  _i2cPort->write(indexByte);    
  _i2cPort->endTransmission();
  delay(0x10);
  
  _i2cPort->requestFrom(_address_hr_sensor, 2); 
  status = _i2cPort->read();

  if( status ) return status; 

  returnByte = _i2cPort->read(); 

  digitalWrite(_mfio_pin, HIGH);
  delay(10);

  return returnByte;
}

uint8_t  Aixware_BioSensorHUB::read_byte_i2c(uint8_t _familyByte, uint8_t _indexByte,uint8_t _writeByte)
{

  uint8_t returnByte;
  uint8_t statusByte;

  digitalWrite(_mfio_pin, LOW);
  delay(10);

  _i2cPort->beginTransmission(_address_hr_sensor);
  _i2cPort->write(_familyByte);    
  _i2cPort->write(_indexByte);    
  _i2cPort->write(_writeByte);    
  _i2cPort->endTransmission();
  delay(0x10); 

  _i2cPort->requestFrom(_address_hr_sensor, 2); 
  statusByte = _i2cPort->read();
  if( statusByte )
    return statusByte; 

  returnByte = _i2cPort->read(); 

  digitalWrite(_mfio_pin, HIGH);
delay(10);

  return returnByte; 

}

uint8_t  Aixware_BioSensorHUB::read_byte_i2c_temp(uint8_t _registerbyte)
{

  uint8_t statusByte;

  _i2cPort->beginTransmission(_address_temp_sensor);
  _i2cPort->write(_registerbyte);    
  _i2cPort->endTransmission();

  _i2cPort->requestFrom(_address_temp_sensor,1); 
  statusByte = _i2cPort->read();

  return statusByte; 

}

uint8_t* Aixware_BioSensorHUB::ReadFifo(uint8_t _familyByte, uint8_t _indexByte, uint8_t arraySize, uint8_t array[24] )
{

  uint8_t statusByte;
  digitalWrite(_mfio_pin, LOW);
  delay(10); 

  _i2cPort->beginTransmission(_address_hr_sensor);
  _i2cPort->write(0x12);    
  _i2cPort->write(0x01);     
  _i2cPort->endTransmission();
  delay(200); 

  _i2cPort->requestFrom(_address_hr_sensor, 25);
  statusByte = _i2cPort->read(); 

  for(uint8_t i = 0; i < 24; i++) {
    array[i] = (uint8_t) _i2cPort->read(); 
  }


  digitalWrite(_mfio_pin, HIGH);
   delay(10); 
}

//----------------------------------------------------------------------------------------------------------------------------------
//Temperature sensor 
uint8_t Aixware_BioSensorHUB::gen_call_temp(){

  _i2cPort->beginTransmission(0x00);
  _i2cPort->write(0x04);    
  _i2cPort->write(0x00);     
  _i2cPort->endTransmission();
  delay(10); 


 }

 uint8_t Aixware_BioSensorHUB::enable_eeprom_temp(){

  _i2cPort->beginTransmission(0x0C);
  _i2cPort->write(0x1F);    
  _i2cPort->write(0x80);     
  _i2cPort->endTransmission();
  delay(10); 

 }


uint8_t Aixware_BioSensorHUB::calculate_constants_temp_sensor(){
    
   uint32_t byte_read1=0, byte_read2=0, byte_read3=0;
    float rest_float1=0, rest_float2=0;
    float f_Tobj1=0;
    float f_25_273=0;
    uint32_t temp_Uout1=0;
    uint64_t temp_Uout2=0;
    uint16_t dummy1=0, dummy2=0;
    String str; 
    
    byte_read1 = read_byte_i2c_temp(0x2C); //Register 44
    byte_read2 = read_byte_i2c_temp(0x2D); //Register 45
    temp_M = byte_read1<<8; 
    temp_M = temp_M | byte_read2;
    temp_M_float = temp_M /100.00;

    //Solo para debugear
    // Serial.print("temp_M_float: "); 
    // Serial.println(temp_M_float); 

    byte_read1 = read_byte_i2c_temp(0x2A); //Register 42
    byte_read2 = read_byte_i2c_temp(0x2B); //Register 43
    temp_PTAT25 = byte_read1<<8; 
    temp_PTAT25 = temp_PTAT25 | byte_read2; 

    //Solo para debugear
    // Serial.print("temp_PTAT25: "); 
    // Serial.print(temp_PTAT25); 
    // Serial.println(""); 
    
    byte_read1 = read_byte_i2c_temp(0x30); //Register 48
    byte_read2 = read_byte_i2c_temp(0x31); //Register 49
    temp_Uout1 = byte_read1<<8; 
    temp_Uout1 = temp_Uout1 | byte_read2;
    temp_Uout1 = temp_Uout1 * 2;

    //Solo para debugear
    // Serial.print("temp_Uout1: "); 
    // Serial.println(temp_Uout1); 
    
    byte_read1 = read_byte_i2c_temp(0x2E); //Register 46
    byte_read2 = read_byte_i2c_temp(0x2F); //Register 47
    temp_U0 = byte_read1<<8; 
    temp_U0 = temp_U0 | byte_read2; 
    temp_U0 = temp_U0 +32768;

    //Solo para debugear
    // Serial.print("temp_U0: "); 
    // Serial.println(temp_U0); 
    
    byte_read1 = read_byte_i2c_temp(0x32); //Register 50
    temp_Tobj1 = byte_read1; 

    //Solo para debugear
    // Serial.print("temp_Tobj1: "); 
    // Serial.println(temp_Tobj1); 

    byte_read1 = read_byte_i2c_temp(0x29); //Register 41
    lookup = byte_read1; 

    //Solo para debugear
    // Serial.print("lookup: "); 
    // Serial.println(lookup); 
    
    f_Tobj1 = powf(temp_Tobj1+273.15, 3.8f);
    f_25_273 = powf(25+273.15, 3.8f);

    //Solo para debugear
    // Serial.print("f_Tobj1: "); 
    // Serial.println(f_Tobj1); 
    // Serial.print("f_25_273: "); 
    // Serial.println(f_25_273); 
    
    rest_float1 = (float)(temp_Uout1 - temp_U0);
    rest_float2 = (float) (f_Tobj1-f_25_273);
    const_k =  (float) rest_float1 / rest_float2;
    
    //Solo para debugear
    // Serial.print("rest_float1: "); 
    // Serial.println(rest_float1); 
    // Serial.print("rest_float2: "); 
    // Serial.println(rest_float2); 
    // Serial.print("const_k: "); 
    // Serial.println(const_k); 
    
}


float Aixware_BioSensorHUB::read_amb_temp(void){
    
    uint32_t byte_read1=0, byte_read2=0, TP_ambient=0, i=0;
    float temp_amb=0;    
    
    while ((byte_read1 == 0 || byte_read2 ==0) && i<3) {
        
        byte_read1 = read_byte_i2c_temp(0x03); //Register 3
        byte_read2 = read_byte_i2c_temp(0x04); //Register 4
        i++;
    }

    TP_ambient = ( (byte_read1 & 0x7F) << 8) | byte_read2 ; 

    temp_amb = 25 + 273.15 + ((TP_ambient-temp_PTAT25)*(1.0f/temp_M_float));
    
    ambient_temperature = temp_amb;

    temp_amb = temp_amb-273.15;

    //Solo para debugear
    // Serial.print("temp_amb: "); 
    // Serial.println(temp_amb); 

    
    return temp_amb;
}


float Aixware_BioSensorHUB::read_obj_temp(void){
    
    uint32_t byte_read1=0, byte_read2=0, byte_read3=0, i=0;
    float temp_obj=0, rest_float1=0,  f_Tamb=0, F_inverse=0, temp_U0_float=0;
    int32_t TP_object=0;
    
    byte_read1 = read_byte_i2c_temp(0x01); //Register 1
    byte_read2 = read_byte_i2c_temp(0x02); //Register 2
    byte_read3 = read_byte_i2c_temp(0x03); //Register 3 (Only seventh bit)
    TP_object = (( byte_read1 << 16) | (byte_read2 << 8) | (byte_read3 & 0x80)) >> 7; 
    
    f_Tamb = powf(ambient_temperature, 3.8f);
    
    //Solo para debugear
    // Serial.print("TP_object: "); 
    // Serial.println(TP_object); 

    temp_U0_float =temp_U0;
    
    rest_float1 = (float)(TP_object - temp_U0_float);
    
    //Solo para debugear
    // Serial.print("rest_float1 antes: "); 
    // Serial.println(rest_float1); 

    rest_float1 = rest_float1/const_k;
    F_inverse = (float)(rest_float1 + f_Tamb);
    
    //Solo para debugear
    // Serial.print("temp_U0_float: "); 
    // Serial.println(temp_U0_float); 
    // Serial.print("rest_float1: "); 
    // Serial.println(rest_float1); 
    // Serial.print("F_inverse: "); 
    // Serial.println(F_inverse); 
    // Serial.print("f_Tamb: "); 
    // Serial.println(f_Tamb); 
    
    temp_obj = powf(F_inverse, 0.2631578947f);
    temp_obj = temp_obj-273.15;

    //Solo para debugear
    // Serial.print("temp_obj: "); 
    // Serial.println(temp_obj); 

    return temp_obj;
}