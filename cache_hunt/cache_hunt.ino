/*
 * cache_hunt.ino - Arduino sketch for a multi-location 'reverse geocache' hunt. Based on the parsing example of the Adafruit Ultimate GPS library. Locations are stored in Degrees/Decimal Minutes
 * 
 * Created by Ryan Twilley, June 2013
 */

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LatLng.h>

#include <Servo.h>

#define CHECK_MEMORY_USE 1

#if CHECK_MEMORY_USE
// for debugging only
#include <MemoryFree.h>
#endif

/*
 * Arduino Pins
 * 2 - GPS TX
 * 3 - GPS RX
 * 7 - Servo?
 * 8 - Speaker
 * 9-13 - OLED
 */
#define GPS_TX 3
#define GPS_RX 2
#define SERVO_CNTRL 7
#define AUDIO_OUT 8
#define OLED_MOSI 9
#define OLED_CLK 10
#define OLED_DC 11
#define OLED_CS 12
#define OLED_RESET 13

Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

// If using software serial, keep these lines enabled
// (you can change the pin numbers to match your wiring):
SoftwareSerial mySerial(GPS_TX, GPS_RX);
Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  false
// set to false to stop reporting current status for serial monitor
#define REPORT_SERIAL false

const int UPDATE_PERIOD = 3500;

// distance from target
float range = 3000.0;
int targetsFound = 0;
int currentTarget = 0;

#define NUM_TARGETS 2
#define FOUND_DISTANCE 250.0

const char* TARGET_HINT[NUM_TARGETS] PROGMEM = {
  "TEST1",
  "Meet and Meter"
  //"Dream Team Court",
  //"Used to be fly",
  //"Prep Walks",
  //"And so it begins"
};
// todo: add all locations in order
LatLng* TARGETS[] = {
  new LatLng(3013.286, 'N', 9202.661, 'W'),	// abdalla
  new LatLng(3012.683, 'N', 9200.939, 'W'),	// angelle practice field
  //new LatLng(3012.75, 'N', 9202.284, 'W'),	// bourgeois hall
  //new LatLng(3012.491, 'N', 9159.594, 'W'),	// airport
  //new LatLng(3013.913, 'N', 9159.184, 'W'),	// catholic diocese
  //new LatLng(3010.436, 'N', 9203.675, 'W')	// church
  // new LatLng(3011.247, 'N', 9204.2597, 'W')	// home
};

  // flags to switch what gets shown on oled
boolean showGpsPosition = false;
boolean showCalculatedPosition = true;

// this keeps track of whether we're using the interrupt
// off by default!
// todo: try interrupt
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

// -------- audio part -----------------

#define NOTE_G2 98
#define NOTE_A2 110
#define NOTE_C3 131

#define MELODY_LENGTH 7

// notes in the melody (can i kick it?)
int melody[] = {
  NOTE_G2, 0, NOTE_G2, 0, NOTE_A2, 0, NOTE_C3};

// note durations
int noteDurations[] = { 
  2, 4, 4, 8, 2, 4, 4 };

void playMelody();
// -------- end audio part -----------------

// ---- servo stuff ---------
int servoPwm = 0;
int lockedPosition = 90;    // servo angle for locked box
int openPosition = 180;	 // servo angle for unlocked box

void setServoAngle(int);
// ---- end servo stuff -----

void setup()  
{

  if(REPORT_SERIAL) {
    // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
    // also spit it out
    Serial.begin(115200);
    Serial.println(F("Adafruit GPS library basic test!"));
  }

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC);
  // init done

  // todo: add instructions?
  // display.display(); // show splashscreen

  display.setTextSize(1);
  display.setTextColor(WHITE);

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  // pinMode(SERVO_CNTRL, OUTPUT);

  delay(1000);
  
  playMelody();

  // lock box
  // setServoAngle(lockedPosition);    

  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();

  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
  // writing direct to UDR0 is much much faster than Serial.print 
  // but only one character can be written at a time. 
#endif
}

// todo: hardwire this?
void useInterrupt(boolean v) {
  if (v) {

    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0x1F;

    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } 
  else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

// play a short tune to indicate that a location has been discovered
void playMelody () {

  for (int thisNote = 0; thisNote < MELODY_LENGTH; thisNote++) {

    // to calculate the note duration, take one second 
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];

    tone(AUDIO_OUT, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);

    // stop the tone playing:
    noTone(AUDIO_OUT);
  }
}

void setServoAngle(int changeAngle) {
  for (int angle = 0; changeAngle <= 140; angle += 5)  {
    servoPwm = (angle*11) + 500;      // Convert angle to microseconds
    digitalWrite(SERVO_CNTRL, HIGH);
    delayMicroseconds(servoPwm);
    digitalWrite(SERVO_CNTRL, LOW);
    delay(50);                   // Refresh cycle of servo
  }
}

uint32_t timer = millis();
// string used to print to oled monitor
char locString[16];
// string used to format latitude
char latString[7];
// string used to format longitude
char lonString[7];

void loop()
{
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO && REPORT_SERIAL)
      if (c) Serial.print(c);
  }

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {

    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > UPDATE_PERIOD) { 
    timer = millis(); // reset the timer

    if(REPORT_SERIAL) {
      Serial.print(F("\nTime: "));
      Serial.print(GPS.hour, DEC); 
      Serial.print(':');
      Serial.print(GPS.minute, DEC); 
      Serial.print(':');
      Serial.print(GPS.seconds, DEC); 
      Serial.print('.');
      Serial.println(GPS.milliseconds);
      Serial.print(F("Date: "));
      Serial.print(GPS.day, DEC); 
      Serial.print('/');
      Serial.print(GPS.month, DEC); 
      Serial.print("/20");
      Serial.println(GPS.year, DEC);
      Serial.print(F("Fix: ")); 
      Serial.print((int)GPS.fix);
      Serial.print(F(" quality: ")); 
      Serial.println((int)GPS.fixquality); 
    }
    // if we have good info and we haven't reached the end of our search, keep checking
    if (GPS.fix && (targetsFound != NUM_TARGETS)) {

      if(REPORT_SERIAL) {
        Serial.print(F("Location: "));
        Serial.print(GPS.latitude, 4); 
        Serial.print(GPS.lat);
        Serial.print(F(", ")); 
        Serial.print(GPS.longitude, 4); 
        Serial.println(GPS.lon);

        Serial.print(F("Speed (knots): ")); 
        Serial.println(GPS.speed);
        Serial.print(F("Angle: ")); 
        Serial.println(GPS.angle);
        Serial.print(F("Altitude: ")); 
        Serial.println(GPS.altitude);
        Serial.print(F("Satellites: ")); 
        Serial.println((int)GPS.satellites);
      }

      LatLng* current = new LatLng(GPS.latitude, GPS.lat, GPS.longitude, GPS.lon);
      LatLng* trgt = TARGETS[currentTarget];

      if(showGpsPosition) {
        display.clearDisplay();
        display.setCursor(0, 0);

        dtostrf(current->getLatitude(), 5, 2, latString);
        dtostrf(current->getLongitude(), 5, 2, lonString);
        sprintf(locString, "%s: %s%c %s%c", "H", latString, current->getLatDir(), lonString, current->getLngDir());
        display.println(locString);

        dtostrf(trgt->getLatitude(), 5, 2, latString);
        dtostrf(trgt->getLongitude(), 5, 2, lonString);
        sprintf(locString, "%s: %s%c %s%c", "H", latString, trgt->getLatDir(), lonString, trgt->getLngDir());
        display.println(locString);

        display.display();
      }

      range = current->gcDistanceFrom(trgt);
      if(range < FOUND_DISTANCE) {

        playMelody();

        targetsFound++;
        currentTarget++;

        if(targetsFound == NUM_TARGETS) {

          display.clearDisplay();
          display.setCursor(0, 0);
          display.println(F("Congratulations!"));
          display.println(F("Happy Early Anniversary!"));
          display.display();

          setServoAngle(openPosition);    

          // todo: play winning sound

          // todo: open the box
        }
      } 
      else {
        if(showCalculatedPosition) {
          display.clearDisplay();
          display.setCursor(0, 0);
          display.println(TARGET_HINT[currentTarget]);	// line 1
          display.print(F("Looking for "));
          display.print(currentTarget + 1);
          display.print(F(" of "));
          display.println(NUM_TARGETS); // line 2

          display.print(F("Range: ")); 
          display.println(range); // line 3

#if CHECK_MEMORY_USE
          display.print(F("Mem=")); // line 4?
          display.println(freeMemory());
#endif

          display.display();
        }
      }

      free(current);
      free(trgt);
    }
    else {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println(F("Take me outside!"));
      
#if CHECK_MEMORY_USE
      display.print(F("Mem=")); // line 2?
      display.println(freeMemory());
#endif

      // todo: show antenna level? num satellites?

      display.display();
    }
  }
}

