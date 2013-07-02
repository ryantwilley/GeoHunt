/*
 * LatLng.cpp - Library for gps locations using Decimal Minutes (plays nice with GPS library from Adafruit)
 * Created by Ryan Twilley, June 2013
 */
 
#include "Arduino.h"
#include "LatLng.h"

LatLng::LatLng(float lat, char latDir, float lng, char lngDir) {
  _latitude = lat;
  _latDir = latDir;
  _longitude = lng;
  _lngDir = lngDir;
}

float LatLng::getLatitude() {
  return _latitude;
}

float LatLng::getLongitude() {
  return _longitude;
}

char LatLng::getLatDir() {
  return _latDir;
}

char LatLng::getLngDir() {
  return _lngDir;
}

// returns decimal degrees from decimal minutes format
void LatLng::getDecimalDegrees(float* lat, float* lon) {
  // divide by 100, throw away decimal part (this is degree piece)
  *lat = round(_latitude / 100);
  // subtract degree * 100, this is decimal minute piece
  *lat += (_latitude - *lat * 100) / 60;
  
  *lon = round(_longitude / 100);
  *lon += (_longitude - *lon * 100) / 60;
}

// great circle distance for this point to another point using the Haversine distance
float LatLng::gcDistanceFrom (LatLng* other) {

  float currentLat, currentLon, targetLat, targetLon;
  getDecimalDegrees(&currentLat, &currentLon);
  other->getDecimalDegrees(&targetLat, &targetLon);
  
  // convert to radians
  float lat1, lat2, lon1, lon2, h, distance;
  lat1 = currentLat * DEG_TO_RAD;
  lon1 = currentLon * DEG_TO_RAD;
  
  lat2 = targetLat * DEG_TO_RAD;
  lon2 = targetLon * DEG_TO_RAD;
  
  // returns the great-circle distance between two points (radians) on a sphere
  h = sq((sin((lat1 - lat2) / 2.0))) + (cos(lat1) * cos(lat2) * sq((sin((lon1 - lon2) / 2.0))));
  distance = 2.0 * EARTH_RADIUS * asin (sqrt(h)); 

  return distance;
}