#include <SoftwareSerial.h>
// https://wiki.dfrobot.com/Gravity__Analog_pH_Sensor_Meter_Kit_V2_SKU_SEN0161-V2
#include "DFRobot_PH.h" // 
#include <EEPROM.h>
// https://www.devicemart.co.kr/goods/view?no=1066983
// https://github.com/PaulStoffregen/OneWire
#include <OneWire.h> // 수온
// https://www.kocoafab.cc/tutorial/view/72
// https://www.devicemart.co.kr/goods/view?no=1330500#goods_file
// https://github.com/practicalarduino/SHT1x
#include <SHT1x.h> // 온습도
// https://kocoafab.cc/tutorial/view/689
#include <LiquidCrystal_I2C.h>
#define DEBUG true
#define D8 8
#define dataPin 10
#define clockPin 11
#define PH_PIN A1
#define SOIL_PIN A3
#define ONEWIRE_PIN 4
#define TIMEOUT 10000 // mS
// ph 측정
#define SensorPin A1            //pH meter Analog output to Arduino Analog Input 0
#define Offset 0.00            //deviation compensate
#define LED 13
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0;
// wifi
#define BT_RXD 2
#define BT_TXD 3
String SSID = "PHONGKHACHDHBL";
String PASS = "dhbl2006";
String DST_IP = "192.168.43.223";
String PORT = "3000";

// Lcd 조정
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
// 온습도 측정
SHT1x sht1x(dataPin, clockPin);
//Temperature chip i/o - 온도 측정
OneWire ds(ONEWIRE_PIN);  // on digital pin 2
// ph 농도 측정
DFRobot_PH ph;
// Wifi
SoftwareSerial ESP_wifi(BT_RXD, BT_TXD);

void setup()
{
  pinMode(D8, OUTPUT);

  Serial.begin(9600);  
  ESP_wifi.begin(9600);
  ESP_wifi.setTimeout(10000);

  sendData("AT+RST\r\n", 5000, DEBUG);
  sendData("AT+CWJAP=\"" + SSID + "\",\"" + PASS + "\"\r\n", 15000, DEBUG);

  sendData("AT+CWMODE=3\r\n", 11000, DEBUG); // STA+AP
  sendData("AT+CIFSR\r\n", 7000, DEBUG);
//  sendData("AT+CIPMUX=1\r\n", 7000, DEBUG);
  DST_IP = "anjfodgoatnrhkd.pythonanywhere.com";
  PORT = "80";
  sendData("AT+CIPSTART=\"TCP\",\""+ DST_IP + "\"," + PORT + "\r\n", 5000, DEBUG);

//  String data = "POST /a?id=35 "; // Actual Server Test
//  String data = "POST /quiz?id=35 "; // My Server Test
  String data = "GET /quiz ";
  data += "HTTP/1.1\r\n";
  data += "Host: " + DST_IP + "\r\n";
  data += "\r\n";
  data += "\r\n";

  String cipsend = "AT+CIPSEND=";
  cipsend += String(data.length() + 2);
  cipsend += "\r\n";

  sendData(cipsend, 5000, DEBUG);
  sendData(data, 10000, DEBUG);
  sendData("\r\n", 5000, DEBUG);
  
  String closeCommand = "AT+CIPCLOSE";
  closeCommand += "\r\n";
//  sendData(closeCommand, 3000, DEBUG);  

  lcd.init(); // Start LCD
  lcd.backlight(); // backlight On
  lcd.print("hello vetnam!");
  Serial.println("Hello Arduino!");

  ph.begin();
  
  delay(1000);
}

const int AirValue = 675;   // Value to Air
const int WaterValue = 360;  // Value to Water // ???
int intervals = (AirValue - WaterValue) / 3;
int soilMoistureValue = 0;
int waterPercent = 0;

float voltage,phValue,temperature = 25;

String result;
int state = 1;
void loop()
{
  if(Serial.available()) ESP_wifi.print((char)Serial.read());
  if(ESP_wifi.available()) Serial.print((char)ESP_wifi.read());
  
  lcd.clear();
  
//  if (state == 0)
//  {
//    // 토양 습도 센서
//    soilMoistureValue = analogRead(SOIL_PIN); // read the analog in value
//
//    // Serial.println(); // Serial Output Funtion
//    result = "Soil : ";
//  
////    if ((WaterValue + intervals) > soilMoistureValue && soilMoistureValue > WaterValue) result += "Very Wet";
////    else if ((AirValue - intervals) > soilMoistureValue && soilMoistureValue > (WaterValue + intervals)) result += "Wet";
////    else if (AirValue > soilMoistureValue && soilMoistureValue > (AirValue - intervals)) result += "Dry";
////    else result += "None"; // Unknown State
//  
//    if (soilMoistureValue > AirValue - intervals) result += "Dry";
//    else if (soilMoistureValue > WaterValue + intervals) result += "Wet";
//    else result += "Very Wet";
//    
//    Serial.print(result + ", ");
//    lcd.print(result);
//  
//    waterPercent = map(soilMoistureValue, AirValue, WaterValue, 0, 100);
//    lcd.setCursor(0, 1);
//    result = "Value : " + String(soilMoistureValue);
////    result = "Percent : " + String(waterPercent) + "%";
//    Serial.println(result);
//    lcd.print(result);
//  }
//  else if (state == 1)
//  {
//    // 온습도 센서 - 28C 이상 시에 스프링클러가 작동해야함
//    float temp_c = sht1x.readTemperatureC();
//    float humidity = sht1x.readHumidity();
//
//    Serial.println("Temperature : " + String(temp_c) + "C, Humidity : " + String(humidity) + "%");
//    lcd.print("Temper : " + String(temp_c) + "C"); // Temperature
//    lcd.setCursor(0, 1);
//    lcd.print("Humidi : " + String(humidity) + "%"); // Humidi
//  }
//  else if (state == 2)
//  {
//    // 수온 센서
//     Serial.print("Temperature : " + String(getTemp()) + "C, ");
//     lcd.print("Temper : " + String(getTemp()) + "C");
//  
//    // ph 농도 센서
////    {
////      static unsigned long timepoint = millis();
////      if(millis() - timepoint > 1000U)  //time interval: 1s
////      {
////        timepoint = millis();
////        voltage = analogRead(PH_PIN) / 1024.0 * 5000;  // read the voltage
////        temperature = 30;  // 임시 - 수온을 측정한 것이 들어가야함
////        phValue = ph.readPH(voltage,temperature);  // convert voltage to pH with temperature compensation
////        Serial.print("PH : ");
////        Serial.println(phValue, 2);
////        lcd.setCursor(0, 1);
////        lcd.print("PH : " + String(phValue));
////      }
////      ph.calibration(voltage,temperature);  // calibration process by Serail CMD
////    }
//
//    {
//      static unsigned long samplingTime = millis();
//      static unsigned long printTime = millis();
//      static float phValue,voltage;
//      if(millis()-samplingTime > samplingInterval)
//      {
//          pHArray[pHArrayIndex++]=analogRead(SensorPin);
//          if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
//          voltage = avergearray(pHArray, ArrayLenth)*5.0/1024;
//          phValue = 3.5*voltage+Offset;
//          samplingTime=millis();
//      }
//      if (millis() - printTime > printInterval) //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
//      {
//        Serial.print("Voltage : " + String(voltage));
//        Serial.println(", ph value : " + String(phValue));
//        
//        digitalWrite(LED,digitalRead(LED)^1);
//        printTime=millis();
//
//        lcd.setCursor(0, 1);
//        lcd.print("PH : " + String(phValue) + "(" + String(voltage) + ")");
//      }
//    }
//
//    state = 0;
//  }
//  
//  state++;

//  delay(1000);
}

float getTemp()
{
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if (!ds.search(addr))
  {
      // no more sensors on chain, reset search
      ds.reset_search();
      Serial.println("address don't exist.");
      return -1000;
  }

  if (OneWire::crc8(addr, 7) != addr[7])
  {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if (addr[0] != 0x10 && addr[0] != 0x28)
  {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad

  
  for (int i = 0; i < 9; i++) // we need 9 bytes
  {
    data[i] = ds.read();
  }
  
  ds.reset_search();
  
  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
  
  return TemperatureSum;
}

double avergearray(int* arr, int number)
{
  int max, min;
  double avg;
  long amount = 0;
  if (number <= 0)
  {
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  
  if (number < 5) //less than 5, calculated directly statistics
  {
    for (int i = 0; i < number; i++)
    {
      amount += arr[i];
    }
    
    avg = amount / number;
    
    return avg;
  }
  else
  {
    if (arr[0] < arr[1])
    {
      min = arr[0];
      max = arr[1];
    }
    else
    {
      min=arr[1];
      max=arr[0];
    }
    
    for(int i = 2; i < number; i++)
    {
      if(arr[i] < min)
      {
        amount += min;
        min = arr[i];
      }
      else
      {
        if(arr[i] > max)
        {
          amount += max;
          max = arr[i];
        }
        else
        {
          amount += arr[i]; //min <= arr <= max
        }
      }
    }
    
    avg = (double)amount/(number-2);
  }
  
  return avg;
}

String sendData(String command, const int timeout, boolean debug)
{
  String response = "";
  ESP_wifi.print(command); // send the read character to the esp8266
  long int time = millis();
  while ( (time + timeout) > millis()) {
    while (ESP_wifi.available()) {
      // output to the serial window
      char c = ESP_wifi.read(); // read the next character.
      response += c;
    }
  }
  if (debug) {
    Serial.print(response);
  }
  return response;
}
