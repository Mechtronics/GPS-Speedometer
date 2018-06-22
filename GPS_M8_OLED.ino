/***********************************
Written By: Md Reazul Islam
E-Mail: reazbuet99@yahoo.com
*/
#include "U8g2lib.h" 
#include <TinyGPS++.h> 
#include "DHT.h"


#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif


TinyGPSPlus tinyGPS; 

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

#define GPS_BAUD 9600 // GPS module baud rate. GP3906 defaults to 9600.

#define DHTPIN 8 // what digital pin DHT SENSOR connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

#include <SoftwareSerial.h>
#define ARDUINO_GPS_RX 7 // GPS TX, Arduino RX pin
#define ARDUINO_GPS_TX 6 // GPS RX, Arduino TX pin
SoftwareSerial ssGPS(ARDUINO_GPS_TX, ARDUINO_GPS_RX); // Create a SoftwareSerial

// Set gpsPort to either ssGPS if using SoftwareSerial or Serial1 if using an
// Arduino with a dedicated hardware serial port
#define gpsPort ssGPS  // Alternatively, use Serial1 on the Leonardo

#define SerialMonitor Serial

DHT dht(DHTPIN, DHTTYPE);

unsigned long previousMillis = 0;

void setup()
{
  SerialMonitor.begin(115200);
  gpsPort.begin(GPS_BAUD);
//  sensors.begin();
  dht.begin();
  u8g2.begin();
  u8g2.setFontMode(0);    // enable transparent mode, which is faster
  // flip screen, if required
  //u8g.setRot180();
/*
  // assign default color value
  if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {
    u8g.setColorIndex(255);     // white
  }
  else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
    u8g.setColorIndex(3);         // max intensity
  }
  else if ( u8g.getMode() == U8G_MODE_BW ) {
    u8g.setColorIndex(1);         // pixel on
  }
  else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    u8g.setHiColorByRGB(255, 255, 255);
  }
*/
  //u8g.setFont(u8g_font_fub30);
  //u8g.setFont(u8g_font_profont11);
  //u8g.setFont(u8g_font_unifont);
  //u8g.setFont(u8g_font_osb21);
  //u8g.drawStr( 0, 32, "u8g library");
}

const  int timeOffset = 8; //Perth

const int fontSize = 37;
int carSpeed = -1;
int carCourse = 0;
int homeCourse = 0;
String homeDistance = "";
int insideTemp = 0;
int insideHumidity = 0;
int noOfSat = 0;
String firstLine = "";
String secondLine = "";

static const double HOME_LAT = 51.508131, HOME_LON = -0.128002; //london

void loop()
{
 
  String gpsDate = "";
  String gpsTime = "";

  noOfSat = tinyGPS.satellites.value();
  unsigned long fix_age = tinyGPS.location.age();
  carCourse = (int)tinyGPS.course.deg();
  homeDistance = "";
  unsigned long distanceMToHome = (unsigned long)TinyGPSPlus::distanceBetween( tinyGPS.location.lat(), tinyGPS.location.lng(), HOME_LAT, HOME_LON);
  double courseToHome = TinyGPSPlus::courseTo(tinyGPS.location.lat(), tinyGPS.location.lng(), HOME_LAT, HOME_LON);


  SerialMonitor.print F("Sats: "); SerialMonitor.println(noOfSat);
  SerialMonitor.print F("Lat: "); SerialMonitor.println(tinyGPS.location.lat(), 6);
  SerialMonitor.print("Long: "); SerialMonitor.println(tinyGPS.location.lng(), 6);
  //SerialMonitor.print("Alt: "); SerialMonitor.println(tinyGPS.altitude.feet());
  //SerialMonitor.print("Course: "); SerialMonitor.println(tinyGPS.course.deg());
  SerialMonitor.print("Speed: "); SerialMonitor.println(tinyGPS.speed.kmph());
  SerialMonitor.print("Age: "); SerialMonitor.println(fix_age);
  SerialMonitor.println("Home Angle = " + String(courseToHome));


  if ( !tinyGPS.location.isValid())
  {
    Serial.println("No fix detected");
    carSpeed = -1;
  } else  if (fix_age > 1500)
  {
    Serial.println("Warning: possible stale data!");
    carSpeed = -1; //String ((int)tinyGPS.speed.kmph());
  } else
  {
    carSpeed = (int)tinyGPS.speed.kmph();
    if (carSpeed > 8) carSpeed = carSpeed + 2;
    //carCourse = (int)tinyGPS.course.deg();

    SerialMonitor.print("Course: "); SerialMonitor.println(carCourse);

    homeCourse = (int)courseToHome;
    if (distanceMToHome > 1000)
    {
      homeDistance = String(distanceMToHome / 1000) + "km";
    }
    else
    {
      homeDistance = String(distanceMToHome) + "m";
    }
  }


  gpsDate = String(String(tinyGPS.date.year()) + "/" + String(tinyGPS.date.month()) + "/" + tinyGPS.date.day());

  int timeHour = (int)tinyGPS.time.hour();
  timeHour = timeHour + timeOffset;
  if (timeHour >= 24) timeHour = timeHour - 24;

  gpsTime =  String( timeHour) + ":" + String(tinyGPS.time.minute()) ;//+ ":" + String(tinyGPS.time.second());

  SerialMonitor.println("Home Distance = " + homeDistance);
  SerialMonitor.println("Date: " + gpsDate);
  SerialMonitor.println("Time: " + gpsTime);
  SerialMonitor.println();


  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 2000) {
    // save the last time
    previousMillis = currentMillis;

    float h = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t))
    {
      //Serial.println("Failed to read from DHT sensor!");
      //return;

    } else
    {
      insideTemp =  (int) t;
      insideHumidity = (int) h;
    }
  }

  firstLine = String(insideTemp) + String((char)176) + "C " + String(insideHumidity) + "% Home:" + homeDistance;
  secondLine =   gpsDate + " " + gpsTime    + " S-" + String(noOfSat);
  u8g2.firstPage();
  do {
    printGPSInfo()  ;
  } while ( u8g2.nextPage() );

  // iii = iii + 1;
  smartDelay(500);
}


void printGPSInfo()
{

/*
  u8g2.clearBuffer();         // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  u8g2.drawStr(0,10,"Hello World!");  // write something to the internal memory
  u8g2.sendBuffer();          // transfer internal memory to the display
 */
  u8g2.setFont(u8g2_font_fur35_tn); //(u8g2_font_fur35n);

  //carSpeed = iii;
  if (carSpeed < 0)
  {
    u8g2.print("");
  } else if (carSpeed < 10)
  {
    //u8g2.setPrintPos( 60, 36);
     u8g2.setCursor(60, 36);
    u8g2.print(carSpeed);
  } else if (carSpeed < 100)
  {
   // u8g2.setPrintPos( 33, 36);
    u8g2.setCursor(33, 36);
    u8g2.print(carSpeed);
  } else
  {
    //u8g2.setPrintPos( 5, 36);
     u8g2.setCursor(5, 36);
    u8g2.print(carSpeed);
  }

  //u8g.drawLine(0, 0, 0, 50);
  // u8g.drawLine(0, 0, 100, 0);
  u8g2.drawLine(0, 38, 98, 38);
  u8g2.drawLine(98, 0, 98, 38);
  //u8g.drawLine(127, 0, 127, 50);
  // u8g.drawLine(100, 25, 128, 25);

  drawCourse(114, 9, 360 - carCourse);
  drawCourse(114, 29, homeCourse - carCourse);
  /*
      sensors.requestTemperatures();
      int temp = (int) sensors.getTempCByIndex(0);
  */
  u8g2.setFont(u8g_font_6x13);
  //u8g2.setPrintPos( 0, 51);
   u8g2.setCursor(0, 51);
  u8g2.print(firstLine);
  //u8g2.setPrintPos( 0, 64);
   u8g2.setCursor(0, 64);
  u8g2.print(secondLine);

}

void drawCourse(int centerX, int centerY, int m)
{

  float handRadius = 8;
  m = m + 90;
  float x1, y1;
  x1 = handRadius * cos(m * 0.0175);
  y1 = handRadius * sin(m * 0.0175);

  u8g2.drawLine(centerX, centerY, centerX - x1, centerY - y1);
  u8g2.drawCircle(centerX, centerY, handRadius + 1,U8G2_DRAW_ALL);
}


// This custom version of delay() ensures that the tinyGPS object
// is being "fed". From the TinyGPS++ examples.
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    // If data has come in from the GPS module
    while (gpsPort.available())
      tinyGPS.encode(gpsPort.read()); // Send it to the encode function
    // tinyGPS.encode(char) continues to "load" the tinGPS object with new
    // data coming in from the GPS module. As full NMEA strings begin to come in
    // the tinyGPS library will be able to start parsing them for pertinent info
  } while (millis() - start < ms);
}


