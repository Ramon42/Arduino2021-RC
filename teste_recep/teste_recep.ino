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
#include <Servo.h>

//========================= Criando objetos =================================//
RF24 radio(7, 8); // CE, CSN // Instancia/cria o objeto Radio para que possamos trabalhar com ele. Também temos que informar os pinos do Arduino conectados ao CE e ao CSN do NRF24L01.
Servo leme;

//==================== Nomeando as constantes ==============================//
#define PinoSensorRadiacaoGama 22
#define VEL 2
#define IN1 3
#define IN2 4
#define srv 5

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
boolean buttonState = 0;

//=================== Declaração da variável global somente de leitura para armazenas os endereço de leitura e escrita =======================//
                    //Endereço: 0         1
const byte Enderecos[][6] = {"00001", "00002"};//Criamos um vetor constante do tipo Byte que terá duas posições, pois precisamos de um endereço para escrever e outro para ler. Apesar do
//endereço ter somete 5 números, declaramos 6 para que o comppilador adicione o caracter null \0 automaticamente.

void setup() {
//==================== Declaração de entradas e saídas ========================//
  pinMode(PinoSensorRadiacaoGama, INPUT_PULLUP);// Declara o pino do PinoSensorRadiacaoGama como entrada PULLUP.

// Pinos para o controle dos  motores através da Monster Shield
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  analogWrite(VEL, 250); // Escreve o valor PMW de 0 à 255 responsável pela velocidade do robô no pino 5.
  leme.attach(srv);

//================== Configurações iniciais do NRF24L01 ======================//
  Serial.begin(9600);
  radio.begin();// Inicia o transceptor NRF24L01
  radio.openWritingPipe(Enderecos[0]); // Informamos para o transceptor qual é o endereço de escrita Enderecos[0] ou 00001 e abrimos o Pipe/Tubo.
  radio.openReadingPipe(1, Enderecos[1]); // Informamos para o transceptor qual é o endereço de leitura Enderecos[1] ou 00002 e abrimos o Pipe/Tubo. Esse primeiro parâmentro, o número 1,
  //define o número do Pipe. Podemos utilizar 6 Pipes, de 0 à 5.
  radio.setPALevel(RF24_PA_MAX); // Muda o PA para potência máxima de transmissão. Para potência minima: RF24_PA_MIN Para potência média: RF24_PA_HIGH Para potência máxima (recomendado para o módulo com etapa amplificadora e antena): RF24_PA_MAX
  radio.setDataRate(RF24_250KBPS); // Mudamos a taxa de trasnferência de dados para 250 Kbps, isso melhora a distância na transmissão.

}



void loop() {
   //========================= Recebendo os dados ===========================//
  delay(5); // Tempo para não haver perca de dados.

  radio.startListening(); // Comando para o rádio começar ouvir, dessa forma ele escuta ou recebe.

  if ( radio.available()) {// Se o NRF24L01 receber dados.
    while (radio.available()) {// Enquanto o NRF24L01 receber dados.
      radio.read(&DadosTransmitidos, sizeof(TipoDosDadosTXRX)); // Lê a informação transmitida pelo Robô.
     }


 //========================= Controle dos Motores ===========================//
     //Controle da velocidade dos motores
      

    leme.write(70);
    Serial.println(DadosTransmitidos.DirecaoRobo);
    if (DadosTransmitidos.DirecaoRobo == 3) {
      //---------------- Robô para frente -----------//
      //Motor lado esquerdo olhando de trás do robô
      digitalWrite(3, LOW);
      digitalWrite(4, HIGH);
      Serial.println("LEU DIRECAO 3");
    }

    if (DadosTransmitidos.DirecaoRobo == 2) {
      //---------------- Robô para trás -----------//
      //Motor lado esquerdo olhando de trás do robô
      digitalWrite(3, HIGH);
      digitalWrite(4, LOW);
      Serial.println("LEU DIRECAO 2");
    }

    if (DadosTransmitidos.DirecaoRobo == 0) {
      //---------------- Robô para o lado esquerdo -----------//
      //Motor lado esquerdo olhando de trás do robô
      digitalWrite(3, LOW);
      digitalWrite(4, HIGH);
      leme.write(0);
      Serial.println("LEU DIRECAO 0");
    }

    if (DadosTransmitidos.DirecaoRobo == 1) {
      //---------------- Robô para o lado direito -----------//
      //Motor lado esquerdo olhando de trás do robô
      digitalWrite(3, LOW);
      digitalWrite(4, HIGH);
      leme.write(170);
      Serial.println("LEU DIRECAO 1");
    }


  }
  
//========================= Trasmitindo os dados ===========================//
  delay(5); // Tempo para não haver perca de dados.
  
  radio.stopListening(); // Comando para o rádio parar de ouvir, dessa forma ele fala ou transmite.
  Serial.println("STOP LISTENING");
  /*
  DadosRecebidos.SensorRadiacaoGama = digitalRead(PinoSensorRadiacaoGama);// Faz a leitura digital do PinoSensorRadiacaoGama e armazena na variável DadosRecebidos.SensorRadiacaoGama.
  radio.write(&DadosRecebidos, sizeof(TipoDosDadosTXRX));// Transmite/escreve os dados para o outro rádio. DadosTransmitidos = Informação que queremos enviar. TipoDosDadosTXRX: Tamanho dessa variável.
  */
}
