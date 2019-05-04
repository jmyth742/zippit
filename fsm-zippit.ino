#include "Fsm.h"
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEBattery.h"
#include <Servo.h>
#include "BluefruitConfig.h"

#include <Adafruit_BLE.h>
#include <Adafruit_BLEGatt.h>

#include "FPS_GT511C3.h"


#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif
//state events
    #define EV_SETUP                    0
    #define EV_WAIT                     1
    #define EV_ADD                      2
    #define EV_REMOVE                   3
    #define EV_ACTIVE                   4
    #define EV_OPEN                     5
    #define EV_ERROR                    6
//ble related defines
    #define FACTORYRESET_ENABLE          1
    #define MINIMUM_FIRMWARE_VERSION    "0.7.0"
    //#define MODE_LED_BEHAVIOUR          "MODE"
    #define OPEN                        "open"
    #define CLOSE                       "close"
    #define MAGIC_NUMBER                0xC0FFEE
    #define MAGIC_STRING                __DATE__

//GATT RELATED INFOR
uint8_t fpsServiceUUID[] = {0x10,0x52,0x5F,0x60,0xCF,0x73,0x11,0xE6,0x95,0x98,0xF8,0xE2,0x25,0x1C,0x9A,0x69};
uint8_t fpsAddFingerUUID[] = {0x10,0x52,0x5F,0x61,0xCF,0x73,0x11,0xE6,0x95,0x98,0xF8,0xE2,0x25,0x1C,0x9A,0x61};
uint8_t fpsReturnFingerUUID[] = {0x10,0x52,0x5F,0x61,0xCF,0x73,0x11,0xE6,0x95,0x98,0xF8,0xE2,0x25,0x1C,0x9A,0x62};
uint8_t fpsActiveFingerUUID[] = {0x10,0x52,0x5F,0x61,0xCF,0x73,0x11,0xE6,0x95,0x98,0xF8,0xE2,0x25,0x1C,0x9A,0x63};
uint8_t fpsDeleteFingerUUID[] = {0x10,0x52,0x5F,0x61,0xCF,0x73,0x11,0xE6,0x95,0x98,0xF8,0xE2,0x25,0x1C,0x9A,0x64};
uint8_t fpsUnlockFingerUUID[] = {0x10,0x52,0x5F,0x61,0xCF,0x73,0x11,0xE6,0x95,0x98,0xF8,0xE2,0x25,0x1C,0x9A,0x65};

//return ids
int32_t fpsServiceId;
int32_t fpsAddFingerChar;
int32_t fpsReturnFingerChar;
int32_t fpsActiveFingerChar;
int32_t fpsDeleteFingerChar;
int32_t fpsUnlockFingerChar;


//objects for BLE
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_BLEGatt gatt(ble);

State st_init(&enter_init, &check_init, &exit_init);
State st_setup(&enter_setup, &setup_zippit, &exit_setup);
State st_wait(&enter_wait, &wait, &exit_wait); 
State st_add(&enter_add, &add, &exit_add);
State st_remove(&enter_remove, &remove_id, &exit_remove);
//State st_active(&enter_active, &active, &exit_active);

Fsm fsm(&st_init);

//for serial comms with 32u4
FPS_GT511C3 fps(4, 5);
/*
 * Information handling structs
 */

struct add_user {
  uint8_t data[];
  bool success;
};

add_user add_user;

/*
 * BLE related setup functions and callbacks
 */
// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void connected(void)
{
  Serial.println( F("Connected") );
}

void disconnected(void)
{
  Serial.println( F("Disconnected") );
}

void BleGattRX(int32_t chars_id, uint8_t data[], uint16_t len)
{
  
  //Serial.print( F("[BLE GATT RX] (" ) );
  Serial.println("Callback");
  Serial.print(chars_id);
  Serial.println();
  Serial.write(data, sizeof(data));
  Serial.println();
//  char *rxMsg[sizeof(data)];
//  Serial.print("data is  ");
//  for (int i = 0;i++;i<sizeof(data)){
//    Serial.println(data[i]);
//    rxMsg[i]=data[i];
//  }
//  Serial.println();
  
  //char *cData = (char *) &data;
  //Serial.print("convert to char typ : ");
  //Serial.println(cData);

  if(chars_id == fpsAddFingerChar){
    Serial.write(add_user.data, sizeof(data));
    fsm.trigger(EV_ADD);
  }else if(chars_id == fpsActiveFingerChar){
    fsm.trigger(EV_ACTIVE);
  }else if(chars_id == fpsReturnFingerChar){
    //fsm.trigger(EV_RETURN);
  }else if(chars_id == fpsDeleteFingerChar){
    fsm.trigger(EV_REMOVE);
  }else if(chars_id == fpsUnlockFingerChar){
    fsm.trigger(EV_OPEN);
  }
} 

/*
 * GATT profile setups
 */

void setupGATT(){
  /* Service ID should be 1 */
  fpsServiceId = gatt.addService(fpsServiceUUID);
  
  fpsAddFingerChar = gatt.addCharacteristic(fpsAddFingerUUID,
                     GATT_CHARS_PROPERTIES_WRITE,
                        sizeof(byte), sizeof(double), BLE_DATATYPE_INTEGER);
  fpsReturnFingerChar = gatt.addCharacteristic(fpsReturnFingerUUID,
                        GATT_CHARS_PROPERTIES_NOTIFY,
                        sizeof(byte), sizeof(float), BLE_DATATYPE_BYTEARRAY);
  fpsActiveFingerChar = gatt.addCharacteristic(fpsActiveFingerUUID,
                        GATT_CHARS_PROPERTIES_WRITE,
                        sizeof(byte), sizeof(float), BLE_DATATYPE_BYTEARRAY);
  
  fpsDeleteFingerChar = gatt.addCharacteristic(fpsDeleteFingerUUID,
                        GATT_CHARS_PROPERTIES_WRITE,
                        sizeof(byte), sizeof(float), BLE_DATATYPE_BYTEARRAY);
  
  fpsUnlockFingerChar = gatt.addCharacteristic(fpsUnlockFingerUUID,
                        GATT_CHARS_PROPERTIES_WRITE,
                        sizeof(byte), sizeof(float), BLE_DATATYPE_BYTEARRAY);
  
  ble.reset();
  /* Set callbacks */

   /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Zippit info:");
  /* Print Bluefruit information */

  ble.setConnectCallback(connected);
  ble.setDisconnectCallback(disconnected);
  
  ble.setBleGattRxCallback(fpsAddFingerChar, BleGattRX);
  ble.setBleGattRxCallback(fpsReturnFingerChar, BleGattRX);
  ble.setBleGattRxCallback(fpsDeleteFingerChar, BleGattRX);
  ble.setBleGattRxCallback(fpsUnlockFingerChar, BleGattRX);
}


/*
 * grouped for the initiliasing FSM transitions 
 */
void enter_init()
{
    Serial.println("check initial state - enter function");
}
 
void check_init()
{
  Serial.println("init state - ev setup - to setup state");
  if(firstBoot()){
  fsm.trigger(EV_SETUP);
  }else
  fsm.trigger(EV_WAIT);
}

void exit_init()
{
  Serial.println("exciting init state");
}


/*
 * FSM for the setup states and transitions
 */
void enter_setup()
{
  Serial.println("enter setup function ");
}

void setup_zippit(){
  //Serial.println("setup state - ev wait - to wait state");
  fsm.trigger(EV_WAIT);
}

void exit_setup()
{
  Serial.println("exiting setup function");
}

/*
 * FSM for the wait transitions
 */
void wait()
{ 
  delay(1000); 
  Serial.println("wait state - waiting on event from BLE");
  //Serial.println("wait state - ev add - to add state");
  //fsm.trigger(EV_ADD);
}

void enter_wait()
{
  Serial.println("enter wait state");
}

void exit_wait()
{
  Serial.println("exiting wait state");
}

/*
 * FSM for the add transitions
 */

void enter_add()
{
  Serial.println("enter add state");
  Serial.println("get data first");
  Serial.println(add_user.data);
}

void add()
{
  Serial.println("add state - ev wait - to wait state");
  fsm.trigger(EV_WAIT);
}

void exit_add()
{
  Serial.println("exiting add state");
}

/*
 * FSM transitions for remove user
 */

void enter_remove()
{
  Serial.println("exiting remove state");
}

void remove_id()
{
  Serial.println("remove state - ev wait - to wait state");

  fsm.trigger(EV_WAIT);
}
void exit_remove()
{
  Serial.println("exiting remove state");
}

bool firstBoot(){
  int32_t magic_number;
  ble.readNVM(0, &magic_number);
  if ( magic_number != MAGIC_NUMBER )
  {
    Serial.println("First Boot, writing magic");
    ble.writeNVM(0 , MAGIC_NUMBER);
    ble.writeNVM(16, MAGIC_STRING);
    return true;
  }else
  {
    Serial.println("MAgic FOUND - not first boot");
    return false;
    }
}


void setup() {
  while (!Serial); // required for Flora & Micro
  delay(500);
  
  Serial.begin(115200);
  Serial.println("initialising device and setup");
  fsm.add_transition(&st_init,&st_setup,EV_SETUP,NULL);
  fsm.add_transition(&st_setup,&st_wait,EV_WAIT,NULL);
  fsm.add_transition(&st_wait,&st_add,EV_ADD,NULL);
  fsm.add_transition(&st_add,&st_wait,EV_WAIT,NULL);
  fsm.add_transition(&st_init,&st_wait,EV_WAIT,NULL);
  fsm.add_transition(&st_wait,&st_remove,EV_REMOVE,NULL);
  fsm.add_transition(&st_remove,&st_wait,EV_WAIT,NULL);
 
 // fsm.add_transition(&st_wait,&st_open,EV_OPEN,NULL);
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );
  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }  
  }
  ble.verbose(false); 
  
  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=Zippit")) ) {
    error(F("Could not set device name?"));
  }

  setupGATT();
  //ble.reset();  
 
//  fps.Open();         //send serial command to initialize fps
//  fps.SetLED(true);   //turn on LED so fps can see fingerprint

  Serial.println("setup done");
    

}

void loop() {
     ble.update();
     fsm.run_machine();
}
