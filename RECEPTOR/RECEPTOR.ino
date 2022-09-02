/*************************** Marlon Nardi ******************************
  Projeto: Programação do Robô Handler V1.0
  Vídeo ensinando a motar: https://www.marlonnardi.com/p/construa-seu-proprio-robo-esteira_23.html
  Loja: https://www.lojamarlonnardi.com/
  Site: https://www.marlonnardi.com/
  Youtube: https://www.youtube.com/marlonnardiw
  Facebook: https://www.facebook.com/professormarlonnardi
************************************************************************/

/******************* Programação do Robô Handler V1.0 ****************/

//======================= Incluindo bilbiotecas ==============================//
#include <SPI.h>
#include <RF24.h>

//========================= Criando objetos =================================//
RF24 radio(7, 8); // CE, CSN // Instancia/cria o objeto Radio para que possamos trabalhar com ele. Também temos que informar os pinos do Arduino conectados ao CE e ao CSN do NRF24L01.


//==================== Nomeando as constantes ==============================//

#define AnalogY A6
#define AnalogX A7
#define AnalogBt 3
#define switch_ctrl 2
#define red_bt 4
#define led_1 5
#define led_2 9
#define led_3 10
#define l_pot A1
#define r_pot A0

//=================== Criando uma estrutura/pacote de dados para a transmissão e recepção de informação através de dois Pipes =======================//
struct EstruturaAutonoma
{
  float avg_lat;
  float avg_lon;
  float velocidade;
  float azimuth_waypoint;
  float bussola;
  float dist_waypoint;
  int max_speed = 255;
  bool autonomo = false;
  bool confirm = false;
};
struct EstruturaControle
{
  int leme_ang;
  int analog_x = 0;
  int analog_y = 0;
  int max_speed = 255;
  bool autonomo = false;
  bool confirm = false;
};
typedef struct EstruturaAutonoma TipoAutonomoTXRX;
TipoAutonomoTXRX DadosAutonomo;

typedef struct EstruturaControle TipoControleTXRX;
TipoControleTXRX DadosControle;

//=================== Declaração de variáveis globáis =======================//
int max_speed = 255;
bool autonomo = false;

//=================== Declaração da variável global somente de leitura para armazenas os endereço de leitura e escrita =======================//
//Endereço: 0         1
const byte Enderecos[][6] = {"00001", "00002"};//Criamos um vetor constante do tipo Byte que terá duas posições, pois precisamos de um endereço para escrever e outro para ler. Apesar do
//endereço ter somete 5 números, declaramos 6 para que o comppilador adicione o caracter null \0 automaticamente.

void setup() {
  //==================== Declaração de entradas e saídas ========================//
  pinMode(AnalogX, INPUT);
  pinMode(AnalogY, INPUT);
  pinMode(red_bt, INPUT_PULLUP); //BOTÃO DE ESTADO IGUAL SWITCH
  pinMode(switch_ctrl, INPUT_PULLUP); //0= off, 1= on
  pinMode(r_pot, INPUT);
  pinMode(l_pot, INPUT);
  pinMode(led_1, OUTPUT);
  pinMode(led_2, OUTPUT);
  pinMode(led_3, OUTPUT);
  //================== Configurações iniciais do NRF24L01 ======================//
  radio.begin();// Inicia o transceptor NRF24L01
  Serial.begin(9600);
  radio.openWritingPipe(Enderecos[0]); // Informamos para o transceptor qual é o endereço de escrita Enderecos[0] ou 00001 e abrimos o Pipe/Tubo.
  radio.openReadingPipe(1, Enderecos[1]); // Informamos para o transceptor qual é o endereço de leitura Enderecos[1] ou 00002 e abrimos o Pipe/Tubo. Esse primeiro parâmentro, o número 1,
  //define o número do Pipe. Podemos utilizar 6 Pipes, de 0 à 5.
  radio.setPALevel(RF24_PA_MAX); // Muda o PA para potência máxima de transmissão. Para potência minima: RF24_PA_MIN Para potência média: RF24_PA_HIGH Para potência máxima (recomendado para o módulo com etapa amplificadora e antena): RF24_PA_MAX
  radio.setDataRate(RF24_250KBPS); // Mudamos a taxa de trasnferência de dados para 250 Kbps, isso melhora a distância na transmissão.
  radio.setRetries(3, 5); // delay, count
  delay(1000);
}



void loop() {
  delay(5);
  setMaxSpeed();
  switchMode();
  switch (autonomo) {
    case true:
      Serial.println("ENTROU MODO AUTONOMO");
      modo_autonomo();
      break;
    case false:
      modo_controle();
      break;
  }
  getData();
  send();
  Serial.print("MODO: ");
  Serial.println(autonomo);
}

void setMaxSpeed() {
  if (digitalRead(red_bt)) {
    digitalWrite(led_1, HIGH);
    max_speed = map(analogRead(l_pot), 0, 1023, 0, 255);
    DadosControle.max_speed = max_speed;
    DadosAutonomo.max_speed = max_speed;
    Serial.print("max speed: ");
    Serial.println(max_speed);
    delay(500);
    digitalWrite(led_1, LOW);
  }
}

void modo_controle() {
  int val_x = analogRead(AnalogX);
  int val_y = analogRead(AnalogY); //Y controla servo
  Serial.print("EIXO X: ");
  Serial.println(val_x);
  Serial.print("EIXO Y: ");
  Serial.println(val_y);
  DadosControle.analog_x = val_x;
  DadosControle.leme_ang = map(val_y, 0, 1023, 0, 180);
  Serial.print("LEME CONVERTIDO: ");
  Serial.println(DadosControle.leme_ang);
}

void modo_autonomo() {
  Serial.print("Latitude: ");
  Serial.println(DadosAutonomo.avg_lat);
  Serial.print("Longitude: ");
  Serial.println(DadosAutonomo.avg_lon);
  Serial.print("Velocidade: ");
  Serial.println(DadosAutonomo.velocidade);
  Serial.print("Bussola: ");
  Serial.println(DadosAutonomo.bussola);
  Serial.print("Distancia para Waypoint: ");
  Serial.println(DadosAutonomo.dist_waypoint);
}

void send() {
  radio.stopListening();
  switch (autonomo) {
    case true:
      DadosAutonomo.confirm = true;
      DadosAutonomo.autonomo = autonomo;
      radio.write(&DadosAutonomo, sizeof(TipoAutonomoTXRX));
      break;
    case false:
      DadosControle.confirm = true;
      DadosControle.autonomo = autonomo;
      radio.write(&DadosControle, sizeof(TipoControleTXRX));
      break;
  }
  radio.flush_tx();
  radio.startListening();
  radio.flush_tx();
}

void getData() {
  if (radio.available()) {
    switch (autonomo) {
      case true:
        radio.read(&DadosAutonomo, sizeof(TipoAutonomoTXRX));
        break;
      case false:
        radio.read(&DadosControle, sizeof(TipoControleTXRX));
        break;
    }
  }
}

void switchMode() {
  autonomo = digitalRead(switch_ctrl);
}
