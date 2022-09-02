#include <Wire.h>
#include <MechaQMC5883.h>

MechaQMC5883 qmc;

int x, y, z;
int azimuth, azXY;
float angulo, declinacao;

//DECLINACAO
int declination_degs = -20;
int declination_mins = 19;
char declination_dir = 'W';

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

  /*
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*M_PI;
    return heading;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*M_PI)
    heading -= 2*M_PI;
    return heading;
  */
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  qmc.init();
  //qmc.setMode(Mode_Continuous,ODR_200Hz,RNG_2G,OSR_256);

  declinacao = SetDeclination(declination_degs, declination_mins, declination_dir);
}

void loop() {
  //float azimuth; //is supporting float too
  qmc.read(&x, &y, &z,&azimuth);
  azXY = qmc.azimuth(&x,&y);//you can get custom azimuth
  angulo = atan2(x, y);
  angulo = AdjustHeading(angulo, declinacao);
  angulo = angulo/0.0174532925;
  if(angulo < 0) 
    angulo+=360;
  
  angulo = 360-angulo;
  
  if (angulo > 338 || angulo < 22)
  {
    Serial.print("Norte, ");
  }
  if (angulo > 22 && angulo < 68)
  {
    Serial.print("Nordeste, ");
  }
  if (angulo > 68 && angulo < 113)
  {
    Serial.print("Leste, ");
  }
  if (angulo > 113 && angulo < 158)
  {
    Serial.print("Suldeste, ");
  }
  if (angulo > 158 && angulo < 203)
  {
    Serial.print("Sul, ");
  }
  if (angulo > 203 && angulo < 248)
  {
    Serial.print("Suldoste, ");
  }
  if (angulo > 248 && angulo < 293)
  {
    Serial.print("Oeste, ");
  }
  if (angulo > 293 && angulo < 338)
  {
    Serial.print("Noroeste, ");
  }
  Serial.print("Heading: ");
  Serial.print(angulo);
  Serial.print(" Azimuth padrão: ");
  Serial.print(azimuth);
    Serial.print(" Azimuth X Y: ");
  Serial.print(azXY);
  Serial.println();
  delay(5000);
}
