/*              NRF24L01 PINS
CE    -> 9
CSN   -> 10 (Hardware SPI SS)
MOSI  -> 11 (Hardware SPI MOSI)
MISO  -> 12 (Hardware SPI MISO)
SCK   -> 13 (Hardware SPI SCK)
IRQ   -> No connection in this example
VCC   -> No more than 3.6 volts
GND   -> GND
                JOYSTICK PINS
throttle_pin   -> A0
aileron_pin    -> A1
elevator_pin   -> A2
rudder_pin     -> A3
                POTENTIOMETER PINS    // NOT EXIST IN ARDUINO UNO
throttle_multiplier       -> A4
aileron_multiplier_pin    -> A5
elevator_multiplier_pin   -> A6
rudder_multiplier_pin     -> A7
                EXTERNAL INTERRUPT
fp_an_active      -> 2
*/
/*
//  LIBRARY
*/
#include <SPI.h>
#include <NRFLite.h>
#include "TimerOne.h"
/*
//  RADIO 
*/
const static uint8_t RADIO_ID = 0;                  // Our radio's id.
const static uint8_t DESTINATION_RADIO_ID = 1;      // Id of the radio we will transmit to.
const static uint8_t PIN_RADIO_CE = 9;
const static uint8_t PIN_RADIO_CSN = 10;

struct RadioPacket // Any packet up to 32 bytes can be sent.
{
    uint8_t FromRadioId;
    uint8_t throttle;
    uint8_t aileron_angle;
    uint8_t elevator_angle;
    uint8_t rudder_angle;
    uint8_t flap;
};

NRFLite _radio;
RadioPacket _radioData;
/*
//  FLAP
*/
const static int flap_pin = 9;
uint8_t flap_input = 0;
/*
//  JOYSTICK INPUT VARIABLE
*/
uint8_t throttle_input = 0; 
uint8_t aileron_input = 0;
uint8_t elevator_input = 0;
uint8_t rudder_input = 0;
/*
//  CALIBRATION VARIABLE
*/
uint8_t throttle_MIN_cal = 0;
uint8_t throttle_MAX_cal = 255;
uint8_t aileron_MIN_cal = 0;
uint8_t aileron_MAX_cal = 255;
uint8_t elevator_MIN_cal = 0;
uint8_t elevator_MAX_cal = 225;
uint8_t rudder_MIN_cal = 0;
uint8_t rudder_MAX_cal = 255;
/*
//  MULTIPLIER VARIABLE
*/
volatile float throttle_multiplier = 1;
volatile float aileron_multiplier = 1;
volatile float elevator_multiplier = 1;
volatile float rudder_multiplier = 1;
/*
//  SERVO ANGLE AND THROTTLE
*/
/*  ALL DEFINED IN 'RadioPacket'
uint8_t throttle = 0;
uint8_t aileron_angle = 90;
uint8_t elevator_angle = 90;
uint8_t rudder_angle = 90;
*/
/* 
//  SERVO MAX-MIN ANGLE
*/
const uint8_t throttle_MIN = 0;
const uint8_t throttle_MAX = 255;
const uint8_t aileron_MIN_angle = 60;
const uint8_t aileron_MAX_angle = 120;
const uint8_t elevator_MIN_angle = 70;
const uint8_t elevator_MAX_angle = 110;
const uint8_t rudder_MIN_angle = 50;
const uint8_t rudder_MAX_angle = 130;

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/*
//  EXTERNAL INTERRUPT 0
//  
*/
volatile boolean fp_an_active = LOW;  // fp_an_active = HIGH -> FLAP AND AILERON WORKS TOGETHER, OTHERWISE WORKS SEPARATELY
void ext_int_zero(){
    fp_an_active != fp_an_active;
}
/*
//  INTERNAL INTERRUPT
*/
void internal_interrupt(){     // MULTIPLIER MUST BE BETWEEN 0-1
    Serial.println("interrupt");
    throttle_multiplier = mapfloat(analogRead(A4), 0, 1024, 0, 1);
    aileron_multiplier = mapfloat(analogRead(A5), 0, 1024, 0, 1);
    elevator_multiplier = mapfloat(analogRead(A6), 0, 1024, 0, 1);
    rudder_multiplier = mapfloat(analogRead(A7), 0, 1024, 0, 1);  
/*  
    Serial.println(String("throttle_multiplier") + throttle_multiplier);
    Serial.println(String("aileron_multiplier") + aileron_multiplier);
    Serial.println(String("elevator_multiplier") + elevator_multiplier);
    Serial.println(String("rudder_multiplier") + rudder_multiplier);
    Serial.println("--------------------------------------------------------");  
*/
}
/*
//  BEFORE FLIGHT CALIBRATION
*/
void calibration(){  /*   Find min and max values of JOYSTICK    */

}

void setup(){
    Serial.begin(9600);
    
    if (!_radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN)){
        Serial.println("Cannot communicate with radio");
        while (1) {} 
    }  
    _radioData.FromRadioId = RADIO_ID;

    calibration();

    pinMode(flap_pin, INPUT);

    Timer1.initialize();            
    Timer1.attachInterrupt(internal_interrupt);  
}

void loop(){

    throttle_input = map(analogRead(A0), 0, 1024, throttle_MIN_cal, throttle_MAX_cal);
    aileron_input = map(analogRead(A1), 0, 1024, aileron_MIN_cal, aileron_MAX_cal);
    elevator_input = map(analogRead(A2), 0, 1024, elevator_MIN_cal, elevator_MAX_cal);
    rudder_input = map(analogRead(A3), 0, 1024, rudder_MIN_cal, rudder_MAX_cal);
    flap_input = digitalRead(flap_pin);
/*  
    Serial.println("SCALED: 0-255");
    Serial.println(String("throttle_input") + throttle_input);
    Serial.println(String("aileron_input") + aileron_input);
    Serial.println(String("elevator_input") + elevator_input);
    Serial.println(String("rudder_input") + rudder_input);
    Serial.println(String("flap_input") + flap_input);   
    Serial.println("--------------------------------------------------------");
*/
    throttle_input = map(throttle_input, 0, 1024, throttle_MIN, throttle_MAX);
    aileron_input = map(aileron_input, 0, 1024, aileron_MIN_angle, aileron_MAX_angle);
    elevator_input = map(elevator_input, 0, 1024, elevator_MIN_angle, elevator_MAX_angle);
    rudder_input = map(rudder_input, 0, 1024, rudder_MIN_angle, rudder_MAX_angle);
/*
    Serial.println("SCALED: MIN-MAX");
    Serial.println(String("throttle_input") + throttle_input);
    Serial.println(String("aileron_input") + aileron_input);
    Serial.println(String("elevator_input") + elevator_input);
    Serial.println(String("rudder_input") + rudder_input);
    Serial.println(String("flap_input") + flap_input);   
    Serial.println("--------------------------------------------------------");
*/
    _radioData.throttle = throttle_input * throttle_multiplier;
    _radioData.aileron_angle = aileron_input * aileron_multiplier;
    _radioData.elevator_angle = elevator_input * elevator_multiplier;
    _radioData.rudder_angle = rudder_input * rudder_multiplier;
    if(fp_an_active == HIGH){
        _radioData.flap = _radioData.aileron_angle;
    }
    else{
        _radioData.flap = flap_input;
    }
/*
    Serial.println(String("_radioData.throttle") + _radioData.throttle);
    Serial.println(String("_radioData.aileron_angle") + _radioData.aileron_angle);
    Serial.println(String("_radioData.elevator_angle") + _radioData.elevator_angle);
    Serial.println(String("_radioData.rudder_angle") + _radioData.rudder_angle);
    Serial.println(String("_radioData.flap") + _radioData.flap);
    Serial.println("--------------------------------------------------------");
*/
    _radio.send(DESTINATION_RADIO_ID, &_radioData, sizeof(_radioData));


}


    /*
    By default, 'send' transmits data and waits for an acknowledgement.
    You can also send without requesting an acknowledgement as shown below.
    _radio.send(DESTINATION_RADIO_ID, &_radioData, sizeof(_radioData), NRFLite::NO_ACK)
    _radio.send(DESTINATION_RADIO_ID, &_radioData, sizeof(_radioData), NRFLite::REQUIRE_ACK) // THE DEFAULT
    */
