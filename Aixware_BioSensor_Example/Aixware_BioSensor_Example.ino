 /*
Ejemplo creado para la placa creada para mediciones de heart rate con el sensor MAX86141 y MAX32664C y mediciones de temperatura de piel con el sensor TpiS 1S 1385 

Creado por Aixware Technologies
Autor: legzez@aixwaretechnologies.com

Este codigo tiene la intencion de servir como guia para desarrollar sobre los sensores MAX86141 y MAX32664C que son sensores para calcular el heart rate del fabricante Maxim Integrated, asi como tambien el sensor de temperatura sin 
contacto TpiS 1S 1385 de Excelitas Technologies. 

Se ha probado el codigo sobre un Arduino UNO y los pines a usar son: 

MFIO Pin = Pin 5
Reset Pin = Pin 4
SDA = A4
SCL = A5
VDD = 3.3 v
GND = GND

Sugerencias de uso:
Una ves puesto el sensor sobre la muñeca o sobre el dedo, hemos visto que es recomendable dejar que se estabilicen las lecturas por algunos segundosya que internamente se hacen procesamientos en los IC. Hay que destacar que las 
lecturas no han pasado por ningun tipo de post procesamiento, lo que se despliega en consola es el valor que arroja los sensores, haciendo los calculos y ajustes necesarios dictados por el datasheet. Es responsabilidad del usuario 
validar las lecturas y si es necesario hacer una calibracion (Sugerida por el datasheet). 

En Aixware nos apasiona la investigación y la tecnología, te invitamos a darte una vuelta por nuestra página web y redes sociales. 
https://www.aixwaretechnologies.com/
*/

#include <Aixware_BioSensorHUB.h>
#include <Wire.h>

//Reset pin, MFIO pin
int resPin = 4;   //Pin 4 de
int mfioPin = 5; 
//A4 => SDA
//A5 => SCL

 uint8_t array_hr_sensor[24];
 float temperatura_ambiente=0;    
 float temperatura_objeto=0;  

 //Estas variables son las tipicas que se podrian usar, aunque el sensor te entrega mas datos, para mas informacion de los datos 
 //que maneja en el paquete revisar el pdf "AN6924_MAX32664C Quick Start Guide_30.13.0_201031.pdf"
 uint32_t op_mode=0;
 uint32_t heartrate=0;  
 uint32_t heartrate_confidence=0; 
 uint32_t RR=0;
 uint32_t RR_confidence=0;
 uint32_t activity_class=0;
 uint32_t R=0;
 uint32_t SpO2_confidence=0;
 uint32_t SpO2=0;
 uint32_t scd_state=0; 
 
Aixware_BioSensorHUB BioSensor(resPin, mfioPin); 

void setup(){

  Serial.begin(115200);
  Serial.println("Iniciando...");
  Wire.begin();
 
  int result;

  //Inicializando sensor. 
  result = BioSensor.begin();
  if (result == 0) Serial.println("Sensor iniciado!");  //No errors. 0x00
  else {
    Serial.print("No se pudo comunicar con el sensor...");
    Serial.println(result);
    if (result ==0x02) Serial.println("Reset mode.");
    else if (result == 0x08) Serial.println("Bootloader mode");

  }

  //Configuracion del sensor
  Serial.println("Configurando Sensor...."); 
  int error; 
  error = BioSensor.config_HR_sensor(); // Configuring just the BPM settings. 
  if(error == 0){ Serial.println("Sensor configured."); //No errors. 0x00
  } else {
    Serial.print("Error al configurar el sensor: ");
    Serial.println(error); 
  }

  //Configuracion sensor de temperatura
  BioSensor.gen_call_temp();
  BioSensor.enable_eeprom_temp();
  BioSensor.calculate_constants_temp_sensor();
}

void loop(){

    //Algunas funciones que te ayudaran a verificar que todo este funcionando bien 
    //SensorHR.readDeviceMode();  //Este metodo te regresa el modo en el que esta, por default debe de ser 0x00 (Continuos)
    //SensorHR.read_address_HUB();  //Este metodo te regresa el numero de version del HUB, por default este metodo debe de desplegar 3.13.0
    //SensorHR.readPartID();      //Este metodo despliega el numero de parte del MAX86141 (0x25)
    //SensorHR.readsensormode();  //Dice el modo en el que esta el MAX86141(debe de ser 0x01 para habilitado)
    //SensorHR.readdacc();        //Despliega si esta conectado el acelerometro (debe de desplegar 0x1B)

    //Hay que leer el estatus del HUB para verificar que haya alguna lectura valida (Lectura entrante = 0x08)
    int hubstt= BioSensor.read_status_HUB();
    while ((hubstt != 0x08)) { 
      hubstt= BioSensor.read_status_HUB();
    }

  
    Serial.println(" ");
    Serial.print("--------------------------------------------------------------------------------"); 
    Serial.println(" ");
    
    Serial.print("HUB Status:");
    Serial.print(hubstt);
    Serial.print(", ");

    //Lectura de sensores
    BioSensor.read_HR_sensor(array_hr_sensor);
    temperatura_ambiente = BioSensor.read_amb_temp();
    temperatura_objeto = BioSensor.read_obj_temp();

    //Desplegamos todos los bytes del sensor
     Serial.print("array: ");
    Serial.print(array_hr_sensor[0], HEX);  Serial.print(" "); 
    Serial.print(array_hr_sensor[1], HEX);  Serial.print(" ");  
    Serial.print(array_hr_sensor[2], HEX);  Serial.print(" ");  
    Serial.print(array_hr_sensor[3], HEX);  Serial.print(" ");   
    Serial.print(array_hr_sensor[4], HEX);  Serial.print(" "); 
    Serial.print(array_hr_sensor[5], HEX);  Serial.print(" "); 
    Serial.print(array_hr_sensor[6], HEX);  Serial.print(" "); 
    Serial.print(array_hr_sensor[7], HEX);  Serial.print(" "); 
    Serial.print(array_hr_sensor[8], HEX);  Serial.print(" "); 
    Serial.print(array_hr_sensor[9], HEX);  Serial.print(" "); 
    Serial.print(array_hr_sensor[10], HEX);  Serial.print(" "); 
    Serial.print(array_hr_sensor[11], HEX);  Serial.print(" "); 
    Serial.print(array_hr_sensor[12], HEX);  Serial.print(" "); 
    Serial.print(array_hr_sensor[13], HEX);  Serial.print(" "); 
    Serial.print(array_hr_sensor[14], HEX);  Serial.print(" "); 
    Serial.print(array_hr_sensor[15], HEX);  Serial.print(" "); 
    Serial.print(array_hr_sensor[16], HEX);  Serial.print(" "); 
    Serial.print(array_hr_sensor[17], HEX);  Serial.print(" "); 
    Serial.print(array_hr_sensor[18], HEX);  Serial.print(" "); 
    Serial.print(array_hr_sensor[19], HEX);  Serial.print(" "); 
    Serial.print(array_hr_sensor[20], HEX);  Serial.print(" "); 
    Serial.print(array_hr_sensor[21], HEX);  Serial.print(" "); 
    Serial.print(array_hr_sensor[22], HEX);  Serial.print(" "); 
    Serial.println(array_hr_sensor[23], HEX); 

    op_mode = array_hr_sensor[0];
    
    heartrate = array_hr_sensor[1] <<8;
    heartrate += array_hr_sensor[2];
    heartrate = heartrate/10;

    heartrate_confidence = array_hr_sensor[3];

    RR = array_hr_sensor[4] <<8;
    RR += array_hr_sensor[5];
    RR = RR/10;

    RR_confidence =array_hr_sensor[6];

    activity_class =array_hr_sensor[7];

    R = array_hr_sensor[8] <<8;
    R += array_hr_sensor[9];
    R = R/1000;

    SpO2_confidence = array_hr_sensor[10];

    SpO2 = array_hr_sensor[11] <<8;
    SpO2 += array_hr_sensor[12];
    SpO2 = SpO2/10;
    
    scd_state = array_hr_sensor[19];

    //Desplegar variables para el usuario
    Serial.print("Op mode: "); 
    Serial.println(op_mode); 
    Serial.print("Heartrate: "); 
    Serial.println(heartrate); 
    Serial.print("Heartrate confidence: "); 
    Serial.println(heartrate_confidence);
    Serial.print("RR: "); 
    Serial.println(RR);
    Serial.print("RR confidence: "); 
    Serial.println(RR_confidence);
    Serial.print("Activity class: ");
    if(activity_class==0x00) Serial.println("Rest. ");
    if(activity_class==0x01) Serial.println("Other. ");
    if(activity_class==0x02) Serial.println("Walk. ");
    if(activity_class==0x03) Serial.println("Run. ");
    if(activity_class==0x04) Serial.println("Bike. ");
    Serial.print("R: "); 
    Serial.println(R);
    Serial.print("SpO2 confidence: "); 
    Serial.println(SpO2_confidence);
    Serial.print("SpO2: "); 
    Serial.println(SpO2);
    Serial.print("State: ");
    if(scd_state==0x00) Serial.println("undetected. ");
    if(scd_state==0x01) Serial.println("off skin. ");
    if(scd_state==0x02) Serial.println("on some subject. ");
    if(scd_state==0x03) Serial.println("on skin. ");
    Serial.print("Temperatura ambiente: "); 
    Serial.println(temperatura_ambiente);
    Serial.print("Temperatura objeto: "); 
    Serial.println(temperatura_objeto);

    //Limpiamos variables
    op_mode=0; 
    heartrate=0; 
    heartrate_confidence=0; 
    RR=0; 
    RR_confidence=0;
    activity_class=0; 
    R=0; 
    SpO2_confidence=0; 
    SpO2=0; 
    scd_state=0; 
    temperatura_ambiente=0; 
    temperatura_objeto=0; 
}
