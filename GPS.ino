#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(2,3);  //시리얼 통신을 위한 객체선언
SoftwareSerial GPSSerial(4,5);

Adafruit_GPS GPS(&GPSSerial);

#define GPSECHO false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;

void setup()  
{
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  BTSerial.begin(9600);
  GPS.begin(115200);
  Serial.println("Adafruit GPS library basic test!");

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

  delay(1000);

  GPSSerial.println(PMTK_Q_RELEASE);
}

uint32_t timer = millis();

void loop()                     // run over and over again
{
  if (BTSerial.available()) {       
    Serial.write(BTSerial.read());  //블루투스측 내용을 시리얼모니터에 출력
  }
  if (Serial.available()) {         
    BTSerial.write(Serial.read());  //시리얼 모니터 내용을 블루추스 측에 WRITE
  }
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
      if (c) Serial.print(c);
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
   BTSerial.println(GPS.lastNMEA());
   Serial.println(GPS.lastNMEA());
   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
    BTSerial.print("nTime: ");
    BTSerial.print(GPS.hour, DEC); BTSerial.print(':');
    BTSerial.print(GPS.minute, DEC); BTSerial.print(':');
    BTSerial.print(GPS.seconds, DEC); BTSerial.print('.');
    BTSerial.println(GPS.milliseconds);
    BTSerial.print("Date: ");
    BTSerial.print(GPS.day, DEC); BTSerial.print('/');
    BTSerial.print(GPS.month, DEC); BTSerial.print("/20");
    BTSerial.println(GPS.year, DEC);
    BTSerial.print("Fix: "); BTSerial.print((int)GPS.fix);
    BTSerial.print(" quality: "); BTSerial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      BTSerial.print("Location: ");
      BTSerial.print(GPS.latitude, 4); BTSerial.print(GPS.lat);
      BTSerial.print(", "); 
      BTSerial.print(GPS.longitude, 4); BTSerial.println(GPS.lon);
      
      BTSerial.print("Speed (knots): "); BTSerial.println(GPS.speed);
      BTSerial.print("Angle: "); BTSerial.println(GPS.angle);
      BTSerial.print("Altitude: "); BTSerial.println(GPS.altitude);
      BTSerial.print("Satellites: "); BTSerial.println((int)GPS.satellites);
    }
  }
}
