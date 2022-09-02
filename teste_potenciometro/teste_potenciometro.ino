#include <Wire.h> //Biblioteca de Comunicacao I2C 
#include <MechaQMC5883.h> //Biblioteca do Sensor QMC5883 - Modulo GY-273


MechaQMC5883 bussola; //Criacao do objeto para o sensor 

int x = 0, y = 0, z = 0;
int angulo = 0;

void setup()
{
  
  Wire.begin(); //Inicializacao da Comunicacao I2C
  Serial.begin(9600); //Inicializacao da comunicacao Serial
  
  bussola.init(); //Inicializando o Sensor QMC5883
}

void loop() 
{ 
  bussola.read(&x,&y,&z); //Obter o valor dos eixos X, Y e Z do Sensor
  angulo = atan2(x, y)/0.0174532925; //Calculo do angulo usando os eixos X e Y atraves da formula


  //Ajuste do angulo entre 0 e 360 graus
  if(angulo < 0) 
  angulo+=360;
  
  angulo = 360-angulo;
  
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
  

  delay(800); //Delay de 500 ms entre novas leituras
}
