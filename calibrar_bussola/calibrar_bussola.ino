#include <QMC5883LCompass.h>

#include <Wire.h> //Biblioteca de Comunicacao I2C 
 
QMC5883LCompass bussola; //Criacao do objeto para o sensor 

static int minx,maxx,miny,maxy,offx=0,offy=0;
int x = 0, y = 0, z = 0;

void calc_offsets(void)  {
   offx = (maxx+minx)/2;
   offy = (maxy+miny)/2;
}

void setup() {
  Wire.begin(); //Inicializacao da Comunicacao I2C
  Serial.begin(9600); //Inicializacao da comunicacao Serial
  
  bussola.init();
  delay(50);
}

void loop() {
  bussola.read();
  x = bussola.getX();
  Serial.println(x);

  delay(500);
}
