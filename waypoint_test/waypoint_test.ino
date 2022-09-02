#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
#include <Filter.h>

int x = 0, y = 0, z = 0;
float rad, angulo, declinacao, waypoint_ang;


double waypoints[][5] = {
  {-23.372064, -49.513398},
  {-23.372064, -49.513398}
  };

//FILTRO
// 20 is the weight (20 => 20%)
// 0 is the initial value of the filter
ExponentialFilter<float> FilteredLat(20, 0);
ExponentialFilter<float> FilteredLon(20, 0);

//DECLINACAO
int declination_degs = 20;
float declination_mins = 19;
char declination_dir = 'W';

SoftwareSerial serial1(5, 6); // RX, TX
TinyGPS gps1;
QMC5883LCompass bussola;

float SetDeclination( int declination_degs , int declination_mins, char declination_dir )
{
  float declination_offset_radians;
  // Convert declination to decimal degrees
  switch(declination_dir)
  {
    // North and East are positive
    case 'E':
      declination_offset_radians = ( declination_degs + (1/60 * declination_mins)) * (M_PI / 180);
      return(declination_offset_radians);

    // South and West are negative
    case 'W':
      declination_offset_radians =  0 - (( declination_degs + (1/60 * declination_mins) ) * (M_PI / 180));
      return(declination_offset_radians);
  }
}

float AdjustHeading(float heading, float declination_offset_radians)
{
  heading += declination_offset_radians;
  return heading;
}

void setup() {
   serial1.begin(9600);
   Serial.begin(9600);
   Wire.begin();
   bussola.init();
   bussola.setCalibration(-877, 851, -953, 893, -515, 1008);
   bussola.setSmoothing(4, true);

  declinacao = SetDeclination(declination_degs, declination_mins, declination_dir);
  Serial.print("Waypoint: ");
  Serial.print("latitude: ");
  Serial.println(waypoints[0][0], 6);
  Serial.print("longitude: ");
  Serial.println(waypoints[1][0], 6);

}

void loop() {
  bool recebido = false;
  static unsigned long delayPrint;

  while (serial1.available()) {
     char cIn = serial1.read();
     recebido = (gps1.encode(cIn) || recebido);  //Verifica até receber o primeiro sinal dos satelites
  }
  if (recebido){
    float latitude, longitude;
    unsigned long idadeInfo;
    gps1.f_get_position(&latitude, &longitude, &idadeInfo);
    if (latitude != TinyGPS::GPS_INVALID_F_ANGLE) {
      Serial.print("Latitude: ");
      Serial.println(latitude, 6);  //Mostra a latitude com a precisão de 6 dígitos decimais
      FilteredLat.Filter(latitude);
    }
    
    if (longitude != TinyGPS::GPS_INVALID_F_ANGLE) {
      Serial.print("Longitude: ");
      Serial.println(longitude, 6);  //Mostra a longitude com a precisão de 6 dígitos decimais
      FilteredLon.Filter(longitude);
    }
    float average_lat = FilteredLat.Current();
    float average_lon = FilteredLon.Current();

    Serial.print("LATITUDE MÉDIA: ");
    Serial.println(average_lat, 6);
    Serial.print("LONGITUDE MÉDIA: ");
    Serial.println(average_lon, 6);
  
    float velocidade;
    velocidade = gps1.f_speed_kmph();   //km/h
    Serial.print("Velocidade (km/h): ");
    Serial.println(velocidade, 2);  //Conversão de Nós para Km/h
    
    unsigned long sentido;
    sentido = gps1.course();
    Serial.print("Sentido (grau): ");
    Serial.println(float(sentido) / 100, 2);


    
    //haversine
    float distancia_entre;
    distancia_entre = gps1.distance_between(average_lat, average_lon, waypoints[0][0], waypoints[1][0]);
    Serial.print("       Distancia até Waypoint: ");
    Serial.println(distancia_entre, 2);
    //azimuth
    float curso; //course_to == azimuth 
    curso = gps1.course_to(average_lat, average_lon, waypoints[0][0], waypoints[1][0]);
    Serial.print("         COURSE TO TINYGPS: ");
    Serial.println(curso);

    
     //BUSSOLA
    float rad;
    bussola.read(); //Obter o valor dos eixos X, Y e Z do Sensor
    x = bussola.getX();
    y = bussola.getY(); 
    rad = atan2(x, y);
    Serial.print("Rad: ");
    Serial.println(rad);
    rad = AdjustHeading(rad, declinacao);
    Serial.print("Rad declinação: ");
    Serial.println(rad);
    angulo = rad*180/M_PI;
    Serial.print("Conversão rad para ang: ");
    Serial.println(angulo);
    //Ajuste do angulo entre 0 e 360 graus
    if(angulo < 0) 
      angulo+=360;
    
    angulo = 360-angulo;
    Serial.print("ANGULO AJUSTADO FINAL:");
    Serial.println(angulo);
  
    
    float vetor_nav = curso - angulo;
    Serial.print("         VIRAR: "); //angulo negativo= virar para a direita
    Serial.println(vetor_nav);
  
  }
}
