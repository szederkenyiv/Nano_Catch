#include "Arduino_BMI270_BMM150.h"
#include <ArduinoBLE.h>
#include <Arduino_HS300x.h>
#include <Arduino_LPS22HB.h>
#include <avr/dtostrf.h>
#include <mbed.h>



volatile int bite_counter=0;
volatile int temperature_counter=93;
volatile int led_counter=0;
volatile int a=0;
volatile int red=255;
volatile int green=0;
volatile int blue=0;
volatile float sensitvity=1.05;
float result=-1;
volatile float temperature=0;
volatile float pressure=0;
char temparatureToString[6];
char pressureToString[6];
char barometricInfo[13];
volatile float samplingAvg=0;
volatile uint16_t counter=0;
bool timeToSendBarInfo=true; 
bool timeToAlarm=false;
bool isTimer=true;
bool isAlarmTimer=true;
float x, y, z;
String bite;

mbed::Ticker timer;
mbed::Ticker led_timer;
mbed::Watchdog &watchdog=mbed::Watchdog::get_instance();

//setting up ble
  BLEService biteAlarmService("4fafc201-1fb5-459e-8fcc-c5c9c331914b"); //Service uuid
  BLEStringCharacteristic alarmCharacteristic("ca73b3ba-39f6-4ab3-91ae-186dc9577d99", BLEWrite|BLERead|BLENotify,4); //Bite alarm characteristic
  BLEStringCharacteristic temperatureCharacteristic("c6daa4c4-e942-450d-b2df-872d918278cc",BLERead,13);
  BLEStringCharacteristic setAlarmCharacteristic("c560e5c8-a1fd-4d0e-8fe4-33b39fd28e8a",BLEWrite,1); 
  BLEStringCharacteristic changeLedColorCharacteristic("70ba83c6-6311-4790-bdc8-9e9245e79232",BLEWrite,11);
  BLEStringCharacteristic changeSensitvityCharacteristic("808cea06-6c56-4dfc-a2bf-2cf415a8cf57",BLEWrite,4);






class MyBoschSensor: public BoschSensorClass {

  public:
    MyBoschSensor(TwoWire& wire = Wire) : BoschSensorClass(wire) {};

  protected:
    virtual int8_t configure_sensor(struct bmi2_dev *dev)
    {
      int8_t rslt;
      uint8_t sens_list[1] = { BMI2_ACCEL };
      
      

      struct bmi2_int_pin_config int_pin_cfg;
      int_pin_cfg.pin_type = BMI2_INT1;
      int_pin_cfg.int_latch = BMI2_INT_NON_LATCH;
      int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
      int_pin_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
      int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
      int_pin_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;

      struct bmi2_sens_config sens_cfg[1];
      sens_cfg[0].type = BMI2_ACCEL;
      sens_cfg[0].cfg.acc.bwp = BMI2_ACC_OSR4_AVG1;
      sens_cfg[0].cfg.acc.odr = BMI2_ACC_ODR_6_25HZ;
      sens_cfg[0].cfg.acc.filter_perf = BMI2_POWER_OPT_MODE;
      sens_cfg[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
     

      rslt = bmi2_set_int_pin_config(&int_pin_cfg, dev);
      if (rslt != BMI2_OK)
        return rslt;

      rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, dev);
      if (rslt != BMI2_OK)
        return rslt;

      rslt = bmi2_set_sensor_config(sens_cfg, 1, dev);
      if (rslt != BMI2_OK)
        return rslt;

      rslt = bmi2_sensor_enable(sens_list, 1, dev);
      if (rslt != BMI2_OK)
        return rslt;
     
     
     
      return rslt;
    }
};

MyBoschSensor myIMU(Wire1);

void isBite() {
  // we can also read accelerometer / gyro data here!
  
  
    
 
    
  if (myIMU.accelerationAvailable()) {
   
    myIMU.readAcceleration(x, y, z);
    
    if(x<0){
      x=x*-1;
    }
    if (bite_counter==0){
        result=x;
    }
    if(result>sensitvity*x){
      Serial.println(sensitvity);
      timeToAlarm=true;
    }
    else{
      // set alarm false
      timeToAlarm=false;
    }
    bite_counter=bite_counter+1;

    if(bite_counter==2){
      bite_counter=0;
    }
    
    
  }

}
void sendBarometricInfo()
{
  temperature = HS300x.readTemperature();
  pressure = BARO.readPressure();
  dtostrf(temperature, 6, 2, temparatureToString);
  strcpy(barometricInfo,temparatureToString);
  strcat(barometricInfo,"&");

  dtostrf(pressure,6,2,pressureToString);
  strcat(barometricInfo,pressureToString);
  temperatureCharacteristic.setValue(barometricInfo);
}
void isTimeToSendBarInfo()
{
  timeToSendBarInfo=true;
}
void led_display(){
  led_counter=led_counter+1;
}
void setLedColor(){
    String ledstate=changeLedColorCharacteristic.value();
    red=getValue(ledstate,',',0).toInt();
    blue=getValue(ledstate,',',1).toInt();
    green=getValue(ledstate,',',2).toInt(); 
}
void setLEDON(){
  analogWrite(A5,red);
  analogWrite(A1,green);
  analogWrite(A3,blue);
}
void setLedOFF(){
  analogWrite(A5,0);
  analogWrite(A1,0);
  analogWrite(A3,0);
}
void setSensivity(){
  String sensState=changeSensitvityCharacteristic.value();
  sensitvity=sensState.toFloat();
}
String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); 
  myIMU.debug(Serial);
  myIMU.onInterrupt(isBite);
  myIMU.begin();
  uint8_t sens_list2[1] = { BMI2_GYRO };
  struct bmi2_dev *dev;
  bmi2_sensor_disable(sens_list2,1,dev);

  
  struct bmm150_settings setting;
  setting.pwr_mode=BMM150_POWERMODE_SUSPEND;

 
  
// setting up Temp sensor
 if (!HS300x.begin()) {
    Serial.println("Failed to initialize humidity temperature sensor!");
    while (1);
  }
// setting up Pressure sensor
 if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
    while (1);
  }
// setting up ble
  if(!BLE.begin()){
    Serial.println("starting Bluetooth® Low Energy failed!");
    while (1);
  }
  BLE.setLocalName("Kapasjelzo");
  BLE.setAdvertisedService(biteAlarmService);

  biteAlarmService.addCharacteristic(alarmCharacteristic);
  biteAlarmService.addCharacteristic(temperatureCharacteristic);
  biteAlarmService.addCharacteristic(setAlarmCharacteristic);
  biteAlarmService.addCharacteristic(changeLedColorCharacteristic);
  biteAlarmService.addCharacteristic(changeSensitvityCharacteristic);


  BLE.addService(biteAlarmService);

  BLE.advertise();
  
  watchdog.start();
  pinMode(A1, OUTPUT);  // sets the pin as output
  pinMode(A3, OUTPUT);
  pinMode(A5, OUTPUT);
}
int flip=0;
void loop() {
  
   BLEDevice central = BLE.central();
 
while (central.connected()){
  
  String a=setAlarmCharacteristic.value();
  if(changeSensitvityCharacteristic.written()){
    setSensivity();
  }
 if(isTimer){
  timer.attach(&isTimeToSendBarInfo,294.0f);
  isTimer=false;
 }
 
 
  if(timeToSendBarInfo){
    sendBarometricInfo();
    timeToSendBarInfo=false;
  }
  if(a=="1")
  {
    
  
  if(timeToAlarm){
 
    if(changeLedColorCharacteristic.written()){
        setLedColor();
    }
    setLEDON();
      alarmCharacteristic.setValue("true");

    if(isAlarmTimer){
      led_timer.attach(led_display,0.01f);
      isAlarmTimer=false;
    }
    led_counter=0;
    while(led_counter<2);
    setLedOFF();
    
   
    alarmCharacteristic.setValue("fals");
    
  }
  }
   
  watchdog.kick();
}
watchdog.kick();
timer.detach();
led_timer.detach();
isTimer=true;
isAlarmTimer=true;
setAlarmCharacteristic.setValue("0");

}
