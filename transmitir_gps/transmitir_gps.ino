
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
#include <nRF24L01.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
#include <Filter.h>
#include <TinyGPS++.h>
#include <AltSoftSerial.h>

//========================= Criando objetos =================================//
RF24 Radio(2, 3); // CE, CSN // Instancia/cria o objeto Radio para que possamos trabalhar com ele. Também temos que informar os pinos do Arduino conectados ao CE e ao CSN do NRF24L01.
TinyGPSPlus gps1;
AltSoftSerial serial1;
QMC5883LCompass bussola;

//==================== Nomeando as constantes ==============================//
#define SIZE 32 //tamanho para buffer transmissor
char buffer[SIZE + 1];

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
float ang, declinacao;
float m_rad = 0, rad;
int x = 0, y = 0;

//DECLINACAO
int declination_degs = -20;
float declination_mins = 19;
char declination_dir = 'W';

//GPS
float latitude, longitude;
float avg_lat;
float avg_lon;
float velocidade;
unsigned long sentido;

//FILTRO
// 20 is the weight (20 => 20%)
// 0 is the initial value of the filter
ExponentialFilter<float> FilteredLat(20, 0);
ExponentialFilter<float> FilteredLon(20, 0);

//=================== Declaração da variável global somente de leitura para armazenas os endereço de leitura e escrita =======================//
//Endereço: 0         1
const byte Enderecos[][6] = {"00001", "00002"}; //Criamos um vetor constante do tipo Byte que terá duas posições, pois precisamos de um endereço para escrever e outro para ler. Apesar do
//endereço ter somete 5 números, declaramos 6 para que o comppilador adicione o caracter null \0 automaticamente.

void setup() {
  //==================== Declaração de entradas e saídas ========================//

  //================== Configurações iniciais do NRF24L01 ======================//
  bussola.init(); //Inicializando o Sensor QMC5883
  bussola.setCalibration(-877, 851, -953, 893, -515, 1008);
  //bussola.setSmoothing(4, true);

  declinacao = SetDeclination(declination_degs, declination_mins, declination_dir);

  Radio.begin();// Inicia o transceptor NRF24L01
  Wire.begin();
  Serial.begin(9600);
  serial1.begin(9600);
  Radio.enableAckPayload();
  Radio.setRetries(5, 5);
  Radio.openWritingPipe(Enderecos[1]); // Informamos para o transceptor qual é o endereço de escrita Enderecos[1] ou 00002 e abrimos o Pipe/Tubo.
  Radio.openReadingPipe(1, Enderecos[0]); // Informamos para o transceptor qual é o endereço de leitura Enderecos[0] ou 00001 e abrimos o Pipe/Tubo. Esse primeiro parâmentro, o número 1,
  //define o número do Pipe. Podemos utilizar 6 Pipes, de 0 à 5.
  Radio.setPALevel(RF24_PA_MAX);// Muda o PA para potência máxima de transmissão. Para potência minima: RF24_PA_MIN Para potência média: RF24_PA_HIGH Para potência máxima (recomendado para o módulo com etapa amplificadora e antena): RF24_PA_MAX
  Radio.setDataRate(RF24_250KBPS); // Mudamos a taxa de trasnferência de dados para 250 Kbps, isso melhora a distância na transmissão.

  //Radio.enableDynamicPayloads();
  //Radio.enableAckPayload();

}


void loop() {
  delay(5);// Tempo para não haver perca de dados
  //========================= Trasmitindo os dados ===========================//

  while (serial1.available() > 0) {
    if (gps1.encode(serial1.read())) {
      Serial.print(F("Location: "));
      if (gps1.location.isValid())
      {
        latitude = gps1.location.lat();
        longitude = gps1.location.lng();
        Serial.print(latitude, 6);
        Serial.print(F(","));
        Serial.println(longitude, 6);
        FilteredLat.Filter(latitude);
        FilteredLon.Filter(longitude);
        DadosTransmitidos.avg_lat = FilteredLat.Current();
        DadosTransmitidos.avg_lon = FilteredLon.Current();
        DadosTransmitidos.velocidade = gps1.speed.mph();
        DadosTransmitidos.sentido = gps1.course.deg();
        Serial.print("Latitude média: ");
        Serial.println(DadosTransmitidos.avg_lat);
        Serial.print("Longitude média: ");
        Serial.println(DadosTransmitidos.avg_lon);
        Serial.print("Velocidade: ");
        Serial.println(DadosTransmitidos.velocidade);
        Serial.print("Sentido: ");
        Serial.println(DadosTransmitidos.sentido);
        dist_waypoint = gps.distanceBetween(
                          avg_lat,
                          avg_lon,
                          waypoints[0][0],
                          waypoints[1][0]
                        ) / 1000.0;
        azimuth_waypoint = gps.courseTo(
                             avg_lat,
                             avg_lon,
                             waypoints[0][0],
                             waypoints[1][0]
                           );
      }
    }
  }

  Radio.stopListening(); // Comando para o rádio parar de ouvir, dessa forma ele fala ou transmite.
  Radio.write(&DadosTransmitidos, sizeof(TipoDosDadosTXRX));// Transmite/escreve os dados para o outro rádio. DadosTransmitidos = Informação que queremos enviar. TipoDosDadosTXRX: Tamanho dessa variável.


  //========================= Recebendo os dados ===========================//
  delay(5);// Tempo para não haver perca de dados.

  Radio.flush_tx();
  Radio.startListening(); // Comando para o rádio começar ouvir, dessa forma ele escuta ou recebe.
  Radio.flush_tx();


  //PROBLEMA CASO NÃO SE CONECTE =====================================================//
  //while (!Radio.available()); //Fica em looping até receber a informação.
  //Radio.read(&DadosRecebidos, sizeof(TipoDosDadosTXRX)); // Lê a informação transmitida pelo Robô.

  //tentando corrigir esta caralha=========================================
  if (Radio.available()) {
    Radio.read(&DadosRecebidos, sizeof(TipoDosDadosTXRX));
  }

  delay(5);
}

float SetDeclination( int declination_degs , int declination_mins, char declination_dir )
{
  float declination_offset_radians;
  // Convert declination to decimal degrees
  switch (declination_dir)
  {
    // North and East are positive
    case 'E':
      declination_offset_radians = ( declination_degs + (1 / 60 * declination_mins)) * (M_PI / 180);
      return (declination_offset_radians);

    // South and West are negative
    case 'W':
      declination_offset_radians =  0 - (( declination_degs + (1 / 60 * declination_mins) ) * (M_PI / 180));
      return (declination_offset_radians);
  }
}

float AdjustHeading(float heading, float declination_offset_radians)
{
  heading += declination_offset_radians;
  return heading;
}
