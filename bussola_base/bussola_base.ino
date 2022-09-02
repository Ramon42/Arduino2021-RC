#include <QMC5883LCompass.h>

#include <Wire.h> //Biblioteca de Comunicacao I2C 
#include <QMC5883L.h> //Biblioteca do Sensor QMC5883 - Modulo GY-273
 
QMC5883LCompass bussola; //Criacao do objeto para o sensor 


int x = 0, y = 0, z = 0, t;
float angulo, declinacao;


//DECLINACAO
int declination_degs = -20;
float declination_mins = 19;
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
}

void setup()
{
  
  Wire.begin(); //Inicializacao da Comunicacao I2C
  Serial.begin(9600); //Inicializacao da comunicacao Serial
  
  bussola.init(); //Inicializando o Sensor QMC5883
  //bussola.setCalibration(-718, 851, -850, 738, -515, 1008);
  bussola.setCalibration(-877, 851, -953, 893, -515, 1008);
  bussola.setSmoothing(4, true);
  //bussola.setSamplingRate(10);
  
  declinacao = SetDeclination(declination_degs, declination_mins, declination_dir);
  Serial.print("DECLINACAO EM RADIANO: ");
  Serial.println(declinacao);
  delay(50);
}
 
void loop() 
{ 
  float m_rad = 0, rad;
  for (int i = 0; i < 100; i++){
    bussola.read(); //Obter o valor dos eixos X, Y e Z do Sensor
    x = bussola.getX();
    y = bussola.getY();
    m_rad += atan2(x, y);
  }
  rad = m_rad / 100;
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
  Serial.println(angulo); //Imprime o valor do angulo na Serial do Arduino
 
  //Apresentando o sentido com base no angulo encontrado
  if (angulo > 338 || angulo < 22)
  {
    Serial.println("Norte");
  }
  if (angulo > 22 && angulo < 68)
  {
    Serial.println("Nordeste");
  }
  if (angulo > 68 && angulo < 113)
  {
    Serial.println("Leste");
  }
  if (angulo > 113 && angulo < 158)
  {
    Serial.println("Suldeste");
  }
  if (angulo > 158 && angulo < 203)
  {
    Serial.println("Sul");
  }
  if (angulo > 203 && angulo < 248)
  {
    Serial.println("Suldoste");
  }
  if (angulo > 248 && angulo < 293)
  {
    Serial.println("Oeste");
  }
  if (angulo > 293 && angulo < 338)
  {
    Serial.println("Noroeste");
  }
 
  delay(1000); //Delay de 500 ms entre novas leituras
}
