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
#include <nRF24L01.h>

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
struct EstruturaDadosTXRX
{
  float ang = 0;
  float avg_lat = 0;
  float avg_lon = 0;
  float velocidade = 0;
  float sentido = 0;
};
typedef struct EstruturaDadosTXRX TipoDosDadosTXRX;
TipoDosDadosTXRX DadosTransmitidos;
TipoDosDadosTXRX DadosRecebidos;

//=================== Declaração de variáveis globáis =======================//
int max_speed = 255;

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
  radio.setPALevel(RF24_PA_MAX); // Muda o PA para potência máxima de transmissão. Para potência minima: RF24_PA_MIN Para potência média: RF24_PA_HIGH Para potência máxima (recomendado para o módulo com etapa amplificadora e antena): RF24_PA_MAX
  radio.setDataRate(RF24_250KBPS); // Mudamos a taxa de trasnferência de dados para 250 Kbps, isso melhora a distância na transmissão.
  radio.enableAckPayload();
  radio.setRetries(5,5);
  //radio.enableDynamicPayloads();
  //radio.enableAckPayload();

  radio.startListening();
  delay(1000);
}



void loop() {
  //========================= Recebendo os dados ===========================//
  delay(5); // Tempo para não haver perca de dados.

  radio.flush_tx();
  //radio.startListening(); // Comando para o rádio começar ouvir, dessa forma ele escuta ou recebe.

  if ( radio.available() > 0) {// Se o NRF24L01 receber dados.
    radio.read(&DadosTransmitidos, sizeof(TipoDosDadosTXRX)); // Lê a informação transmitida
    Serial.print("Latitude média: ");
    Serial.println(DadosTransmitidos.avg_lat);
    Serial.print("Longitude média: ");
    Serial.println(DadosTransmitidos.avg_lon);
    Serial.print("Velocidade: ");
    Serial.println(DadosTransmitidos.velocidade);
    Serial.print("Sentido: ");
    Serial.println(DadosTransmitidos.sentido);
    Serial.print("Angulo: ");
    Serial.println(DadosTransmitidos.ang);
    delay(5);
  }


  //========================= Trasmitindo os dados ===========================//
  delay(5); // Tempo para não haver perca de dados.

  //radio.stopListening(); // Comando para o rádio parar de ouvir, dessa forma ele fala ou transmite.



  //radio.write(&DadosRecebidos, sizeof(TipoDosDadosTXRX));// Transmite/escreve os dados para o outro rádio. DadosTransmitidos = Informação que queremos enviar. TipoDosDadosTXRX: Tamanho dessa variável.
}
