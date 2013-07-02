/*
 * LatLng.h - Library for gps locations using Decimal Minutes (plays nice with GPS library from Adafruit)
 * Created by Ryan Twilley, June 2013
 */

#ifndef LatLng_h
#define LatLng_h

#include "Arduino.h"

#define DEG_TO_RAD 0.01745329251994
#define EARTH_RADIUS 6371000.0

class LatLng
{
  public:
    LatLng(float lat, char latDir, float lng, char lngDir);
	float getLatitude();
	float getLongitude();
	char getLatDir();
	char getLngDir();
	
	// great circle distance for this point to another point using the Haversine distance
    float gcDistanceFrom(LatLng* other);
	
  private:
	// stored in degree/decimal minutes (always positive)
    float _latitude;
	
	// N or S
	char _latDir;
	
	// stored in degree/decimal minutes (always positive)
	float _longitude;
	
	// E or W
	char _lngDir;
	
	// returns decimal degrees from decimal minutes format
	void getDecimalDegrees(float* lat, float* lon);
};

#endif