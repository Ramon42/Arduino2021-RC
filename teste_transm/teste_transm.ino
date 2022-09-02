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

//========================= Criando objetos =================================//
RF24 Radio(7, 8); // CE, CSN // Instancia/cria o objeto Radio para que possamos trabalhar com ele. Também temos que informar os pinos do Arduino conectados ao CE e ao CSN do NRF24L01.


//==================== Nomeando as constantes ==============================//
#define LEDIndicadorRadicaoGama 22

#define AnalogicoEsquerdoY A6// 0 à 950
#define AnalogicoEsquerdoX A7// 80 à 1023 Meio 511

#define AnalogicoDireitoX A1
#define BotaoL1 A2 // 0 ou 1023
#define AnalogicoL2 A3// 0 à 1023
#define AnalogicoR2 A5 // 13 à 1023


//=================== Criando uma estrutura/pacote de dados para a transmissão e recepção de informação através de dois Pipes =======================//
struct EstruturaDadosTXRX
{

  boolean SensorRadiacaoGama = false;
  int ValorConvertidoAnalogicoR2 = 0;
  int DirecaoRobo = 0;
};

typedef struct EstruturaDadosTXRX TipoDosDadosTXRX;
TipoDosDadosTXRX DadosTransmitidos;
TipoDosDadosTXRX DadosRecebidos;

//=================== Declaração de variáveis globáis =======================//
int ValorAnalogicoR2;
int ValorAnalogicoEsquerdoY;
int ValorAnalogicoEsquerdoX;
boolean EstadoBotao = 0;


//=================== Declaração da variável global somente de leitura para armazenas os endereço de leitura e escrita =======================//
                    //Endereço: 0         1
const byte Enderecos[][6] = {"00001", "00002"}; //Criamos um vetor constante do tipo Byte que terá duas posições, pois precisamos de um endereço para escrever e outro para ler. Apesar do
//endereço ter somete 5 números, declaramos 6 para que o comppilador adicione o caracter null \0 automaticamente.

void setup() {
  //==================== Declaração de entradas e saídas ========================//
  pinMode(LEDIndicadorRadicaoGama, OUTPUT);// Declara o pino do LEDIndicadorRadicaoGama como saída.
  Serial.begin(9600);
  //================== Configurações iniciais do NRF24L01 ======================//
  Radio.begin();// Inicia o transceptor NRF24L01
  Radio.openWritingPipe(Enderecos[1]); // Informamos para o transceptor qual é o endereço de escrita Enderecos[1] ou 00002 e abrimos o Pipe/Tubo.
  Radio.openReadingPipe(1, Enderecos[0]); // Informamos para o transceptor qual é o endereço de leitura Enderecos[0] ou 00001 e abrimos o Pipe/Tubo. Esse primeiro parâmentro, o número 1,
  //define o número do Pipe. Podemos utilizar 6 Pipes, de 0 à 5.
  Radio.setPALevel(RF24_PA_MAX);// Muda o PA para potência máxima de transmissão. Para potência minima: RF24_PA_MIN Para potência média: RF24_PA_HIGH Para potência máxima (recomendado para o módulo com etapa amplificadora e antena): RF24_PA_MAX
  Radio.setDataRate(RF24_250KBPS); // Mudamos a taxa de trasnferência de dados para 250 Kbps, isso melhora a distância na transmissão.
}


void loop() {
  delay(5);// Tempo para não haver perca de dados
  //========================= Trasmitindo os dados ===========================//

  ValorAnalogicoEsquerdoX = analogRead(AnalogicoEsquerdoX);// Lê o valor analógico do AnalogicoEsquerdoX e manda para a variável ValorAnalogicoEsquerdoX.
  ValorAnalogicoEsquerdoY = analogRead(AnalogicoEsquerdoY);// Lê o valor analógico do AnalogicoEsquerdoY e manda para a variável ValorAnalogicoEsquerdoY.
  Serial.print("X: ");
  Serial.println(ValorAnalogicoEsquerdoX);
  Serial.print("y: ");
  Serial.println(ValorAnalogicoEsquerdoY);
  //Robô para Trás
  if (ValorAnalogicoEsquerdoX < 471) {
    DadosTransmitidos.DirecaoRobo = 2;
    Serial.println("RÉ");
  }
  //Robô para Frente
  if (ValorAnalogicoEsquerdoX >= 520) {
    DadosTransmitidos.DirecaoRobo = 3;
    Serial.println("FRENTE");
  }

  //Vira o robô para a direita
  if (ValorAnalogicoEsquerdoY <= 482) {
    DadosTransmitidos.DirecaoRobo = 1;
    Serial.println("ESQUERDA");
  }
  //Vira o robô para a esquerda
  if (ValorAnalogicoEsquerdoY >= 562) {
    DadosTransmitidos.DirecaoRobo = 0;
    Serial.println("DIREITA");
  }

  ValorAnalogicoR2 = analogRead(AnalogicoR2);
  DadosTransmitidos.ValorConvertidoAnalogicoR2 = map(ValorAnalogicoR2, 13, 1023, 0, 255);
  Serial.println("PASSOU TRANSMISSÃO");
  Radio.stopListening(); // Comando para o rádio parar de ouvir, dessa forma ele fala ou transmite.
  Radio.write(&DadosTransmitidos, sizeof(TipoDosDadosTXRX));// Transmite/escreve os dados para o outro rádio. DadosTransmitidos = Informação que queremos enviar. TipoDosDadosTXRX: Tamanho dessa variável.


  //========================= Recebendo os dados ===========================//
  delay(5);// Tempo para não haver perca de dados.

  Radio.startListening(); // Comando para o rádio começar ouvir, dessa forma ele escuta ou recebe.
  Serial.println("START LISTENING");
    
  while (!Radio.available()); //Fica em looping até receber a informação. 
    Radio.read(&DadosRecebidos, sizeof(TipoDosDadosTXRX)); // Lê a informação transmitida pelo Robô.
  Serial.println("SAIU DO LOOP");
  if (DadosRecebidos.SensorRadiacaoGama == HIGH) {// Se DadosRecebidos.SensorRadiacaoGama que foi transmitido pelo robô for igual a HIGH.
    digitalWrite(LEDIndicadorRadicaoGama, HIGH);// Liga o LEDIndicadorRadicaoGama
  }
  else {// Se não
    digitalWrite(LEDIndicadorRadicaoGama, LOW);// Desliga o LEDIndicadorRadicaoGama
    Serial.println("RECEBIDO");
  }
}
