
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
#include <QMC5883LCompass.h>
#include <Servo.h>
#include <Filter.h>
#include <AltSoftSerial.h>
#include <TinyGPS++.h>

#define leme_pin 10
//========================= Criando objetos =================================//
RF24 radio(2, 3); // CE, CSN // Instancia/cria o objeto Radio para que possamos trabalhar com ele. Também temos que informar os pinos do Arduino conectados ao CE e ao CSN do NRF24L01.
QMC5883LCompass bussola;
Servo leme;

//==================== Nomeando as constantes ==============================//
//MONSTER MOTOR SHIELD
#define BRAKEVCC 0
#define CW  1
#define CCW 2
#define BRAKEGND 3

#define MOTOR_A 0
#define MOTOR_B 1
const uint8_t PWM_MAX = 255;
const uint8_t PWM_HALF = PWM_MAX / 2;

const int currentSensingThreshhold = 100;

const int inAPin = 7;
const int inBPin = 4; // PRESTA ATENÇÃO AQUI CARALHO
const int pwmPin[2] = {5, 6};
const int csPin[2] = {2, 3};

//=================== Criando uma estrutura/pacote de dados para a transmissão e recepção de informação através de dois Pipes =======================//

struct EstruturaAutonoma
{
  float ang = 0;
  int max_speed = 255;
  bool confirm = false;
  int srv_ang = 70;
};
struct EstruturaControle
{
  int analog_x = 0;
  int analog_y = 0;
};
typedef struct EstruturaAutonoma TipoAutonomoTXRX;
TipoAutonomoTXRX DadosTransmitidosA;
TipoAutonomoTXRX DadosRecebidosA;

typedef struct EstruturaControle TipoControleTXRX;
TipoControleTXRX DadosTransmitidosC;
TipoControleTXRX DadosRecebidosC;

//=================== Declaração de variáveis globáis =======================//
int i, leme_ang; //Variável para contagem
bool autonomo = false;
float graus; //Variável para armazenar o valor aferido
float precisao;
int x = 0, y = 0, z = 0;

//=================== Declaração da variável global somente de leitura para armazenas os endereço de leitura e escrita =======================//
//Endereço: 0         1
const byte Enderecos[][6] = {"00001", "00002"}; //Criamos um vetor constante do tipo Byte que terá duas posições, pois precisamos de um endereço para escrever e outro para ler. Apesar do
//endereço ter somete 5 números, declaramos 6 para que o comppilador adicione o caracter null \0 automaticamente.

void setup() {
  //==================== Declaração de entradas e saídas ========================//

  //================== Configurações iniciais do NRF24L01 ======================//
  radio.begin();// Inicia o transceptor NRF24L01
  Wire.begin();
  leme.attach(leme_pin);
  leme.write(70);
  Serial.begin(9600);
  radio.openWritingPipe(Enderecos[1]); // Informamos para o transceptor qual é o endereço de escrita Enderecos[1] ou 00002 e abrimos o Pipe/Tubo.
  radio.openReadingPipe(1, Enderecos[0]); // Informamos para o transceptor qual é o endereço de leitura Enderecos[0] ou 00001 e abrimos o Pipe/Tubo. Esse primeiro parâmentro, o número 1,
  //define o número do Pipe. Podemos utilizar 6 Pipes, de 0 à 5.
  radio.setPALevel(RF24_PA_MAX);// Muda o PA para potência máxima de transmissão. Para potência minima: RF24_PA_MIN Para potência média: RF24_PA_HIGH Para potência máxima (recomendado para o módulo com etapa amplificadora e antena): RF24_PA_MAX
  radio.setDataRate(RF24_250KBPS); // Mudamos a taxa de trasnferência de dados para 250 Kbps, isso melhora a distância na transmissão.
}


void loop() {
  delay(5);// Tempo para não haver perca de dados
  //========================= Trasmitindo os dados ===========================//
  DadosTransmitidos.ang = graus;
  DadosTransmitidos.confirm = false;
  DadosTransmitidos.srv_ang = leme_ang;

  radio.stopListening(); // Comando para o rádio parar de ouvir, dessa forma ele fala ou transmite.
  radio.write(&DadosTransmitidos, sizeof(TipoDosDadosTXRX));// Transmite/escreve os dados para o outro rádio. DadosTransmitidos = Informação que queremos enviar. TipoDosDadosTXRX: Tamanho dessa variável.


  //========================= Recebendo os dados ===========================//
  delay(5);// Tempo para não haver perca de dados.

  radio.startListening(); // Comando para o rádio começar ouvir, dessa forma ele escuta ou recebe.

  while (!radio.available()); //Fica em looping até receber a informação.
  radio.read(&DadosRecebidos, sizeof(TipoDosDadosTXRX)); // Lê a informação transmitida pelo Robô.

  if (DadosRecebidos.confirm == true) {// Se DadosRecebidos.SensorRadiacaoGama que foi transmitido pelo robô for igual a HIGH.
    Serial.println("OK, dados recebidos");
  }
  else {// Se não
    Serial.println("Falha");
  }
  if (DadosRecebidos.analog_x < 500) { //ré
    Serial.println("Ré");
    Serial.print("velocidade: ");
    Serial.println(DadosRecebidos.max_speed);
    motorGo(MOTOR_A, CCW, DadosRecebidos.max_speed);
    motorGo(MOTOR_B, CCW, DadosRecebidos.max_speed);
    delay(50);
  }
  else if (DadosRecebidos.analog_x > 510) {
    Serial.println("Frente");
    Serial.print("aux velocidade: ");
    Serial.println(DadosRecebidos.max_speed);
    motorGo(MOTOR_A, CW, DadosRecebidos.max_speed);
    motorGo(MOTOR_B, CW, DadosRecebidos.max_speed);
    delay(50);
  }
  else {
    Serial.println("Parado");
    motorOff(MOTOR_A);
    motorOff(MOTOR_B);
    delay(50);
  }

  leme_ang = DadosRecebidos.srv_ang;
  leme.write(leme_ang);
  Serial.print("Leme lido: ");
  Serial.println(leme_ang);
  delay(50);
}

//====================================== METODOS MOTOR MONSTER SHIELD ==============================================//
void motorSetup()
{
  //pinMode(statPin, OUTPUT);
  pinMode(inAPin, OUTPUT);
  pinMode(inBPin, OUTPUT);
  digitalWrite(inAPin, LOW);
  digitalWrite(inBPin, LOW);
  // Initialize digital pins as outputs
  for (int i = 0; i < 2; i++)
  {
    pinMode(pwmPin[i], OUTPUT);
  }
}

void motorOff(uint8_t motor)
{
  digitalWrite(inAPin, LOW);
  digitalWrite(inBPin, LOW);

  analogWrite(pwmPin[motor], 0);
}

void motorGo(uint8_t motor, uint8_t mode, uint8_t speed)
{

  if (motor == MOTOR_A || motor == MOTOR_B)
  {
    switch (mode)
    {
      case BRAKEVCC: // Brake to VCC
        digitalWrite(inAPin, HIGH);
        digitalWrite(inBPin, HIGH);
        break;
      case CW: // Turn Clockwise
        digitalWrite(inAPin, HIGH);
        digitalWrite(inBPin, LOW);
        break;
      case CCW: // Turn Counter-Clockwise
        digitalWrite(inAPin, LOW);
        digitalWrite(inBPin, HIGH);
        break;
      case BRAKEGND: // Brake to GND
        digitalWrite(inAPin, LOW);
        digitalWrite(inBPin, LOW);
        break;

      default:
        // Invalid mode does not change the PWM signal
        return;
    }
    analogWrite(pwmPin[motor], speed);
  }
  return;
}

void send() {
  radio.stopListening();
  bool rslt;
  switch (autonomo) {
    case true:
      rslt = radio.write()
             break;
    case false:
      break;
  }

}
