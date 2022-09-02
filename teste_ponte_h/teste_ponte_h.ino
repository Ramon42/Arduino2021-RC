//Programa : Controle 2 motores DC usando Ponte H L298N
//Autor : FILIPEFLOP
 
//Definicoes pinos Arduino ligados a entrada da Ponte H
int IN1 = 5;
int IN2 = 6;
int vel = 7;
int IN3 = 8;
int IN4 = 9;
int vel2 = 10;
  
void setup()
{
  //Define os pinos como saida
 pinMode(IN1, OUTPUT);
 pinMode(IN2, OUTPUT);
 pinMode(vel, OUTPUT);
 pinMode(IN3, OUTPUT);
 pinMode(IN4, OUTPUT);
 pinMode(vel2, OUTPUT);
 analogWrite(vel,255);
 analogWrite(vel2,255);
 //Gira o Motor A no sentido horario
 digitalWrite(IN1, HIGH);
 digitalWrite(IN2, LOW);
 digitalWrite(IN3, HIGH);
 digitalWrite(IN4, LOW);
}
  
void loop()
{
}
