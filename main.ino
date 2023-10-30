/*
 * Code for the B0 and shim/gradient powIer supply. Uses an AD7789 ADC for current readback
 * and an LTC1595 based circuit for setting the output current. Uses a modified version of
 * the Analog Devices CN0216 circuit from the lab example for reading the ADC.
*/

#include <Arduino.h>
#include <SPI.h>
#include "AD7791.h"

// Do all the setup stuff
#define AD7791_SS 10             // CS pin for the AD7791
#define LD 3                     // LD pin for the LTC1595
#define OT 2                     // Overtemp LED pin
const int NTC = A0;              // Use 328P ADC for reading temperature
float temp = 0.0;                  // NTC temperature reading
const double beta = 3435.0;        // Beta value for NTC
double adc_max_code = pow(2,24) - 1; // Max code for 24 bit ADC
double dac_max_code = pow(2,16) - 1; // Max code for 16 bit DAC
double R1 = 10.0;                // Current shunt resistor value
double Rdac = 7e3;                // DAC feedback resistor value
double vref = 2.5;               // ADC reference voltage
uint32_t ui32Adcdata = 0;        // Stores code from ADC
unsigned int code = 0;           // Code for DAC current setpoint
byte command = 0;                // Incoming command from serial
float I = 0.0;                   // Current set value
float ADC_reading;               // store the value for the ADC reading
unsigned long myTime = 0;

// ADC Calibration Values
double scale = 2.5/adc_max_code/R1; // Sets the scale factor for ADC reading
double offset = 0.0;                // Sets offset for ADC reading
char cal_date[] = "NOV_1_2023";     // Date that current source was last calibrated

void setup(){
	
	// open digital communication protocols
	Serial.begin(9600);
	SPI.begin();

  // initialize pins
  pinMode(OT, OUTPUT);
  digitalWrite(OT, LOW);
	pinMode(AD7791_SS, OUTPUT);	        
	digitalWrite(AD7791_SS, HIGH);		  
  pinMode(LD, OUTPUT);                
  digitalWrite(LD, HIGH);             
  writeDAC(code);
	
	// initialize AD7791
	Ad7791INIT();

  temp = readTemp();
  myTime = millis();
}

void loop(){

  if( myTime + 500 >= millis() ){
    temp = readTemp();
    if( temp >= 90.0 ){
      writeDAC(0);
      digitalWrite(OT,HIGH);
    }
  }

  if( Serial.available() ){
    command = Serial.read();
    //Serial.print("Recieved command ");
    //Serial.println(command);
    switch(command){
      case 'W':
        I = Serial.parseFloat();
        code = I_to_code(I);
//        Serial.println(code);
        writeDAC(code);
        break;       
      case 'R':
        ADC_reading = read_ADC();
        Serial.println(scale * static_cast<double>(ADC_reading) + offset, 6);  
        break;    
      case 'I':
        Serial.println("Current Source");
        break;
      case 'T':
        temp = readTemp();
        Serial.println(temp);
        break;
      case 'D':
        Serial.println(cal_date);
        break;
      case 'M':
        Serial.println("250 mA");
        break;
      default:
        Serial.println("Invalid command");
    }
  }
}

uint64_t read_ADC(){
  
  uint64_t reading = 0;
  uint32_t ui32status = 0;
  
  for(int i=0; i<10; i++){
    
    ui32status = 0;
    
    do{
      ui32status = AD7791.readAd7791(STATUS_READ);
    }while (ui32status & 0x80);
    
    reading += AD7791.readAd7791(DATA_READ);
  }
  
  reading /= 10.0;
  return reading;
}

float readTemp(){
  int sensorValue = analogRead(NTC);
  float RNTC = 10e3 / (1023.0 / sensorValue - 1);
  float T = 1 / ( 1 / 298.15 + (1 / beta) * log(RNTC / 10e3) ) - 273.25; // Gives the temperature in celsius
  return T;
}

void writeDAC(unsigned int code){
  SPI.beginTransaction( SPISettings(1000000, MSBFIRST, SPI_MODE0) );
  SPI.transfer16(code);
  SPI.endTransaction();
  delayMicroseconds(100);
  digitalWrite(LD,LOW);
  delayMicroseconds(50);
  digitalWrite(LD,HIGH); 
}

unsigned int I_to_code(float I){
  float code = (dac_max_code * I * R1 *(1 + R1/Rdac))/2.5;
  if (code >= 65535)
    code = 65535;
//  unsigned long temp = static_cast<unsigned long>(code);
//  Serial.println(code, HEX);
//  Serial.println(static_cast<unsigned int>(code), HEX);
  return static_cast<unsigned int>(code);
}

void Ad7791INIT(void){
	AD7791.writeAd7791 (RESET, 0xFF);		            // Resets the part for initial use
	delay(1000);
	AD7791.writeAd7791 (MODE_WRITE, 0x04);	        // Mode register value (single conversion, unipolar input, buffered mode)
//	AD7791.writeAd7791 (FILTER_WRITE, 0x07);	      // Filter register value (clock not divided down, 9.5 Hz update rate)
}
