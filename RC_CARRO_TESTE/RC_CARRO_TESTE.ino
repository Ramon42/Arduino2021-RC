
/*************************** Marlon Nardi ******************************
  Projeto: Programação do Controle Handler V1.0
  Vídeo ensinando a montar: https://www.marlonnardi.com/p/construa-seu-proprio-robo-esteira_23.html
  Loja: https://www.lojamarlonnardi.com/
  Site: https://www.marlonnardi.com/
  Youtube: https://www.youtube.com/marlonnardiw
  Facebook: https://www.facebook.com/professormarlonnardi
************************************************************************/

/******************* Programação do Controle Handler V1.0 ***************/

//======================= Incluindo bilbiotecas ==============================//
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <Servo.h>

#define direcao 8
//========================= Criando objetos =================================//
RF24 radio(2, 3); // CE, CSN // Instancia/cria o objeto Radio para que possamos trabalhar com ele. Também temos que informar os pinos do Arduino conectados ao CE e ao CSN do NRF24L01.
Servo servo;

//==================== Nomeando as constantes ==============================//
//MONSTER MOTOR SHIELD
int SPEED = 4;
int IN1 = 5;
int IN2 = 6;


//=================== Criando uma estrutura/pacote de dados para a transmissão e recepção de informação através de dois Pipes =======================//

struct EstruturaDadosTXRX
{
  int max_speed = 125;
  bool confirm = false;
  int srv_ang = 70;
  int analog_x = 0;
};

typedef struct EstruturaDadosTXRX TipoDosDadosTXRX;
TipoDosDadosTXRX DadosTransmitidos;
TipoDosDadosTXRX DadosRecebidos;

//=================== Declaração de variáveis globáis =======================//
int i, leme_ang; //Variável para contagem
//=================== Declaração da variável global somente de leitura para armazenas os endereço de leitura e escrita =======================//
//Endereço: 0         1
const byte Enderecos[][6] = {"00001", "00002"}; //Criamos um vetor constante do tipo Byte que terá duas posições, pois precisamos de um endereço para escrever e outro para ler. Apesar do
//endereço ter somete 5 números, declaramos 6 para que o comppilador adicione o caracter null \0 automaticamente.

void setup() {
  //==================== Declaração de entradas e saídas ========================//
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  //================== Configurações iniciais do NRF24L01 ======================//
  radio.begin();// Inicia o transceptor NRF24L01
  Wire.begin();
  servo.attach(direcao);
  servo.write(90);
  Serial.begin(9600);
}


void loop() {
  //========================= Recebendo os dados ===========================//
  delay(5);// Tempo para não haver perca de dados.
  servo.write(90);
  motorFrente();
  delay(5000);
  servo.write(65);
  motorRe();
  delay(5000);~´
}

//====================================== METODOS MOTOR MONSTER SHIELD ==============================================//
void motorFrente()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}
void motorRe()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}
void motorParar()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}
