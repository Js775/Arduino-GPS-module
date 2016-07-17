/* Arduino based GPS Module for USDL(PNU Drone) 
 * 
 *    Main Function : GPS Data Send(RS232) and Logging
 * 
 *    Author : J.S. Jeong
 *    Date : 2016. 7. 18.
 * 
 */

#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <SD.h>


SoftwareSerial mySerial(6, 7); // RX, TX
TinyGPS gps;

File myFile;
Sd2Card card;

  char path[4];
  unsigned int NoSV, DOP;
  unsigned long date, time;
  bool savedate = true, sdcard = true;
  long lat, lon, alt;
  int speedU;

void gpsdump(TinyGPS &gps);
void gpssave();
void gpssend();
 
void setup()  
{
  Serial.begin(57600);  // set the data rate for LTE board
  mySerial.begin(9600);  // set the data rate for the SoftwareSerial port
  SD.begin(10); // Initializes the SD library and card

  if (!card.init(SPI_HALF_SPEED, 10))
    { sdcard = false;} 
  else {
    // Check the existed files and Adjust the file path
    int i = 0;
    itoa(i,path,10);
    while(SD.exists(path)){
      i++;
      itoa(i,path,10);
    }

    // Open the data logging file and write the headline    
    myFile = SD.open(path,FILE_WRITE);
      myFile.println("PNU USDL - GPS data Logging");
    myFile.close();
  }

  delay(1000);
}
 
void loop() 
{
  bool newdata = false;
  unsigned long start = millis();
 
  // GPS 데이터 갱신을 대기
  while (millis() - start < 100) { // 최소 100ms 이상으로 잡을것
    if (mySerial.available()) {
      char c = mySerial.read();
      if (gps.encode(c)) {
        newdata = true;
    //  break;  // write new data immediately!
      }
    }
  }

  // newdata가 가용할 경우 GPS 정보 수신, 전송, 저장
  if (newdata) {
    gpsdump(gps); // 수신된 GPS 정보를 전역 변수에 저장

    if(sdcard) { // sdcard가 사용가능할 경우 데이터를 기록 
      if(savedate){ // 파일 첫줄에 날짜, 시간 기록 
        myFile = SD.open(path,FILE_WRITE);
          myFile.print("Date : ");  myFile.print(date);
          myFile.print(" / Time : "); myFile.print(time); myFile.println(" + 9:00");
        myFile.close();
        savedate = false;
      }
    
      gpssave(); // GPS Data를 SD카드에 기록
    }
    
    gpssend(); // GPS Data를 시리얼로 전송
   }

  // delay(5000);
}
 
void gpsdump(TinyGPS &gps)
{  // On Arduino, GPS characters may be lost during lengthy Serial.write()

  unsigned long age; 

  gps.get_position(&lat, &lon, &age);
  lat *= 100;
  lon *= 100;
  alt = gps.altitude();  
  NoSV = gps.satellites();
  DOP = gps.hdop()*0.1;
  speedU = gps.f_speed_mps();
  gps.get_datetime(&date, &time, &age);
}

void gpssave()
{   
  myFile = SD.open(path,FILE_WRITE);    
    myFile.print(lat);
    myFile.print("\t");
    myFile.print(lon);
    myFile.print("\t");
    myFile.print(alt);
    myFile.print("\t");
    myFile.print(NoSV);
    myFile.print("\t");
    myFile.println(DOP);
  myFile.close();
}

void gpssend()
{

  unsigned int type, msgID, msgLength, msgChksum;
  
// Start MSG ID 1
  type = 0;   msgID = 1;   msgLength = 45;  msgChksum = 0;
  
// Header 0~2
  Serial.write(0xFD);  Serial.write(0xFE);  Serial.write(0xFF);

// Msg Type, Msg ID, Msg Length
  Serial.write(type);  Serial.write(msgID);  Serial.write(msgLength);
  msgChksum = msgChksum + type + msgID + msgLength;

  // Payload
  Serial.write(0x61); //FCC mode
  msgChksum += 0x61;

  Serial.write(lowByte(lat)); // Latitude 1
    msgChksum += lowByte(lat);
    lat = lat >> 8;
  Serial.write(lowByte(lat)); // Latitude 2
    msgChksum += lowByte(lat);
    lat = lat >> 8;
  Serial.write(lowByte(lat)); // Latitude 3
    msgChksum += lowByte(lat);
    lat = lat >> 8;
  Serial.write(lowByte(lat)); // Latitude 4
    msgChksum += lowByte(lat);
  
  Serial.write(lowByte(lon)); // Longitude 1
    msgChksum += lowByte(lon);
    lon = lon >> 8;
  Serial.write(lowByte(lon)); // Longitude 2
    msgChksum += lowByte(lon);
    lon = lon >> 8;
  Serial.write(lowByte(lon)); // Longitude 3
    msgChksum += lowByte(lon);
    lon = lon >> 8;
  Serial.write(lowByte(lon)); // Longitude 4
    msgChksum +=  lowByte(lon);

  Serial.write(lowByte(alt)); // Altitude 1
    msgChksum += lowByte(alt);
    alt = alt >> 8;
  Serial.write(lowByte(alt)); // Altitude 2
    msgChksum += lowByte(alt);
    alt = alt >> 8;
  Serial.write(lowByte(alt)); // Altitude 3
    msgChksum += lowByte(alt);
    alt = alt >> 8;
  Serial.write(lowByte(alt)); // Altitude 4
    msgChksum += lowByte(alt);

  Serial.write(lowByte(speedU)); // U velocity 1
    msgChksum += lowByte(speedU);
  Serial.write(highByte(speedU)); // U velocity 2
    msgChksum += highByte(speedU);

  for (int j = 0;j < 14;j++){
    Serial.write(0x00); // V, D velocity / Main, FCC voltage / Roll Pitch Yaw
  }

  Serial.write(0x01); // State Flag 1
    msgChksum += 0x01;
  Serial.write(0x02); // State Flag 2
    msgChksum += 0x02;

  Serial.write(NoSV); // NoSV
    msgChksum += NoSV;
  Serial.write(DOP); // DOP
    msgChksum += DOP;

  for (int k = 0;k < 12; k++){
    Serial.write(0x00); // Servo ch.1~6 
  }  

  Serial.write(lowByte(msgChksum)); // Msg ID 1 Modular Checksum 

}
