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


State st_init(&check_initial_state, EV_SETUP,&wait_or_setup);
State st_setup(&enter_setup_mode,NULL, &wait_msg);
State st_wait(&wait_msg, NULL, &wait_msg);
State st_open(&open_lock, NULL, &wait_msg);
State st_add(&add_user,NULL,&wait_msg);
State st_remove(&remove_user,NULL,&wait_msg);
State st_activity(&set_activity, NULL, &wait_msg);

Fsm fsm(&st_init);
Adafruit_BLEGatt gatt(ble);

union float_bytes {
  int value;
  uint8_t bytes[sizeof(float)];
};

void connected(void)
{
  Serial.println( F("Connected") );
}

void disconnected(void)
{
  Serial.println( F("Disconnected") );
}


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void BleGattRX(int32_t chars_id, uint8_t data[], uint16_t len)
{
  //Serial.print( F("[BLE GATT RX] (" ) );
  Serial.println("Callback");
  Serial.print(chars_id);
  Serial.println();
  Serial.write(data, sizeof(data));
}  
void setupGATT(){
  /* Service ID should be 1 */
  fpsServiceId = gatt.addService(fpsServiceUUID);
  
  fpsAddFingerChar = gatt.addCharacteristic(fpsAddFingerUUID,
                     GATT_CHARS_PROPERTIES_WRITE,
                        sizeof(byte), sizeof(float), BLE_DATATYPE_INTEGER);
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

  //ble.setBleUartRxCallback(BleGattRX);
  //ble.setBleGattRxCallback(test2, BleGattRX);
  //ble.setBleGattRxCallback(charid_number, BleGattRX);
}

void check_initial_state()
{
  int no=1;
  Serial.println("init state");
  if(no=1){
    //gatt.getChar()
    fsm.trigger(EV_WAIT);
    }
}
  
void wait_or_setup()
{
  Serial.println("wait or setup state");
}

void enter_setup_mode()
{
  Serial.println("setup state");
}

void wait_msg()
{
  Serial.println("wait state");
   static union float_bytes addFinger; //= { .value = random(0,10) };
   gatt.getChar(fpsAddFingerChar, addFinger.bytes, sizeof(addFinger));
   Serial.println(addFinger.value);
   ble.update();
   delay(2000);
}

void open_lock()
{
  Serial.println("open state");
}

void add_user()
{
  Serial.println("add state");
}

void remove_user()
{
  Serial.println("remove state");
}

void set_activity()
{
  Serial.println("active state");
}

void setup() {
  while (!Serial); // required for Flora & Micro
  delay(500);
  
  Serial.begin(115200);
  Serial.println("initialising device and setup");
  fsm.add_transition(&st_init,&st_setup,EV_SETUP,NULL);
  fsm.add_transition(&st_init,&st_wait,EV_WAIT,NULL);
  fsm.add_transition(&st_setup,&st_wait,EV_WAIT,NULL);
  fsm.add_transition(&st_wait,&st_open,EV_OPEN,NULL);
  fsm.add_transition(&st_wait,&st_add,EV_ADD,NULL);
  fsm.add_transition(&st_wait,&st_remove,EV_REMOVE,NULL);
  fsm.add_transition(&st_wait,&st_activity,EV_ACTIVE,NULL);
  
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
  Serial.println("setup done");
    

}

void loop() {
//   static union float_bytes addFinger; //= { .value = random(0,10) };
//   gatt.getChar(fpsAddFingerChar, addFinger.bytes, sizeof(addFinger));
//   Serial.println(addFinger.value);
     ble.update(200);
  // put your main code here, to run repeatedly:
  //Serial.println("run FSM");
  //fsm.run_machine();
  //fsm.trigger(EV_SETUP);
}
