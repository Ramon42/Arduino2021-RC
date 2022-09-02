#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <AltSoftSerial.h>

//SoftwareSerial serial1(4, 9); // RX, TX
TinyGPSPlus gps;
AltSoftSerial serial1;


float avg_lat, avg_lon;
float dist_waypoint;
float azimuth_waypoint;

double waypoints[][5] = {
  { -23.372064, -49.513398},
  { -23.377777, -49.517777}
};

void setup() {
  serial1.begin(9600);
  Serial.begin(9600);

  Serial.print("Waypoint: ");
  Serial.print(waypoints[0][0], 6);
  Serial.print(", ");
  Serial.println(waypoints[0][1], 6);
}

void loop() {
  while (serial1.available() > 0) {
    if (gps.encode(serial1.read())) {
      if (gps.location.isValid())
      {
        avg_lat = gps.location.lat();
        avg_lon = gps.location.lng();
        dist_waypoint = gps.distanceBetween(
                          avg_lat,
                          avg_lon,
                          waypoints[0][0],
                          waypoints[0][1]
                        );
        azimuth_waypoint = gps.courseTo(
                             avg_lat,
                             avg_lon,
                             waypoints[0][0],
                             waypoints[0][1]
                           );
        const char *cardinal_waypoint = gps.cardinal(azimuth_waypoint);

        Serial.print("Latitude: ");
        Serial.println(avg_lat, 6);
        Serial.print("Longitude: ");
        Serial.println(avg_lon, 6);
        Serial.print("Distancia m: ");
        Serial.println(dist_waypoint);
        Serial.print("Azimuth: ");
        Serial.println(azimuth_waypoint);
        Serial.print("Cardinal: ");
        printStr(gps.location.isValid() ? cardinal_waypoint : "*** ", 6);
        Serial.println(".\r");
      }

    }
  }
}
static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
}
