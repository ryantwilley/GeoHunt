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
#error(F("Height incorrect, please fix Adafruit_SSD1306.h!"));
#endif

// If using software serial, keep these lines enabled
// (you can change the pin numbers to match your wiring):
SoftwareSerial mySerial(GPS_TX, GPS_RX);
Adafruit_GPS GPS(&mySerial);

#define UPDATE_PERIOD 3500

float range = 3000.0;
byte targetsFound = 0;
int currentTarget = 0;

#define NUM_TARGETS 4
#define FOUND_DISTANCE 200.0

// todo: figure out how to fit all locations in memory
LatLng* TARGETS[] = {
  new LatLng(3011.822, 'N', 9204.486, 'W'),    // c.c.'s
  new LatLng(3010.901, 'N', 9204.856, 'W'),    // dance
  new LatLng(3010.655, 'N', 9204.769, 'W'),    // ivy (petsmart?)
  new LatLng(3010.436, 'N', 9203.675, 'W')    // church
};

  // flags to switch what gets shown on oled
#define SHOW_GPS_POSITION 0

// -------- audio part -----------------

#define NOTE_G2 98
#define NOTE_A2 110
#define NOTE_C3 131

#define MELODY_LENGTH 7

// notes in the melody (can i kick it?)
int melody[] PROGMEM = {  NOTE_G2, 0, NOTE_G2, 0, NOTE_A2, 0, NOTE_C3};
// note durations
int noteDurations[] PROGMEM = { 2, 4, 4, 8, 2, 4, 3 };

void playMelody();
// -------- end audio part -----------------

// ---- servo stuff ---------
int servoPwm = 0;
byte lockedPosition = 90;    // servo angle for locked box
byte openPosition = 180;	 // servo angle for unlocked box

void setServoAngle(byte);
// ---- end servo stuff -----

void setup()  
{

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC);
  // init done

  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("Ok, here we go!"));
  display.println(F("Find 5 places "));
  display.println(F("from our past to"));
  display.println(F("open this box..."));
  display.display();

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

  // just force the conversion every loop
  TIMSK0 &= ~_BV(OCIE0A);

  pinMode(SERVO_CNTRL, OUTPUT);
  
  // lock box
  setServoAngle(lockedPosition);

  delay(2500);

  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
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

void setServoAngle(byte changeAngle) {
  for (int angle = 0; angle <= changeAngle; angle += 5)  {
    servoPwm = (angle*11) + 500;      // Convert angle to microseconds
    digitalWrite(SERVO_CNTRL, HIGH);
    delayMicroseconds(servoPwm);
    digitalWrite(SERVO_CNTRL, LOW);
    delay(50);                   // Refresh cycle of servo
  }
}

uint32_t timer = millis();

void loop()
{
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  // read data from the GPS in the 'main loop'
  char c = GPS.read();

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) 
  {

    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > UPDATE_PERIOD) 
  { 
    timer = millis(); // reset the timer

    // if we have good info and we haven't reached the end of our search, keep checking
    if (GPS.fix && (targetsFound < NUM_TARGETS)) 
    {
      LatLng* current = new LatLng(GPS.latitude, GPS.lat, GPS.longitude, GPS.lon);
  
#if SHOW_GPS_POSITION
      display.clearDisplay();
      display.setCursor(0, 0);
  
      display.print(current->getLatitude());
      display.print(current->getLatDir());
      display.print(F(" "));
      display.print(current->getLongitude());
      display.println(current->getLngDir());
      
      display.print(TARGETS[currentTarget]->getLatitude());
      display.print(TARGETS[currentTarget]->getLatDir());
      display.print(F(" "));
      display.print(TARGETS[currentTarget]->getLongitude());
      display.println(TARGETS[currentTarget]->getLngDir());
      
#if CHECK_MEMORY_USE
      display.print(F("Mem=")); // line 2?
      display.println(freeMemory());
#endif
  
      display.display();
#else
  
      range = current->gcDistanceFrom(TARGETS[currentTarget]);
      
      if(range > 0 && range < FOUND_DISTANCE) 
      {
  
        playMelody();
  
        targetsFound++;
        currentTarget++;
  
        if(targetsFound == NUM_TARGETS) 
        {
          display.clearDisplay();
          display.setCursor(0, 0);
          display.println(F("Congratulations!"));
          display.println(F("Here's to a great"));
          display.println(F("2 years!"));
          display.println(F("ANR so far..."));
          display.display();
  
          setServoAngle(openPosition);    
  
          // todo: play winning sound
        }
      } 
      else 
      {
          display.clearDisplay();
          display.setCursor(0, 0);
          display.print(F("Looking for "));
          display.print(currentTarget + 1);
          display.print(F(" of "));
          display.println(NUM_TARGETS); // line 1
  
          display.print(F("Range: ")); 
          display.println(range); // line 2
          
          display.print(F("Satellites:"));
          display.println((int)GPS.satellites);  // line 3
  
#if CHECK_MEMORY_USE
          display.print(F("Mem=")); // line 4?
          display.println(freeMemory());
#endif
  
          display.display();
      }
#endif
      
      free(current);

    }
    else 
    {
      if(targetsFound < NUM_TARGETS) 
      {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(F("Take me outside!"));
        
#if CHECK_MEMORY_USE
        display.print(F("Mem=")); // line 2?
        display.println(freeMemory());
#endif
        display.print(F("Satellites:"));
        display.println((int)GPS.satellites);
    
        display.display();
      }
      else 
      {
        // todo: keep playing victory sound?
      }
    }
  }
}

