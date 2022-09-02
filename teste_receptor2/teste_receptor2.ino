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
#define MODE 2

//=================== Criando uma estrutura/pacote de dados para a transmissão e recepção de informação através de dois Pipes =======================//
struct EstruturaDadosTXRX
{
  float ang_bussola = 0.0;
  float ang_gps = 0; //sentido / 100
  float avg_lat = 0;
  float avg_lon = 0;
  float dist = 0; //distancia_entre
  float azimuth = 0; //curso
  float velocidade = 0;
  float vetor_nav = 0;
  char confirm = 'O';
  int srv_ang = 70;
  int analog_x = 0;
  int analog_y = 0;
  bool control_mode = false; //se control_mode = false == funcionando em modo autonomo
  int nivel_agua = 0;
};
typedef struct EstruturaDadosTXRX TipoDosDadosTXRX;
TipoDosDadosTXRX DadosTransmitidos;
TipoDosDadosTXRX DadosRecebidos;

//=================== Declaração de variáveis globáis =======================//


//=================== Declaração da variável global somente de leitura para armazenas os endereço de leitura e escrita =======================//
                    //Endereço: 0         1
const byte Enderecos[][6] = {"00001", "00002"};//Criamos um vetor constante do tipo Byte que terá duas posições, pois precisamos de um endereço para escrever e outro para ler. Apesar do
//endereço ter somete 5 números, declaramos 6 para que o comppilador adicione o caracter null \0 automaticamente.

String leStringSerial(){
  String conteudo = "";
  char caractere;
  
  // Enquanto receber algo pela serial
  while(Serial.available() > 0) {
    // Lê byte da serial
    caractere = Serial.read();
    // Ignora caractere de quebra de linha
    if (caractere != '\n'){
      // Concatena valores
      conteudo.concat(caractere);
    }
    // Aguarda buffer serial ler próximo caractere
    delay(10);
  }
    
  Serial.print("Recebi: ");
  Serial.println(conteudo);
    
  return conteudo;
}

void setup() {
//==================== Declaração de entradas e saídas ========================//
  pinMode(AnalogX, INPUT);
  pinMode(AnalogY, INPUT);
  pinMode(MODE, INPUT_PULLUP); 
//================== Configurações iniciais do NRF24L01 ======================//
  radio.begin();// Inicia o transceptor NRF24L01
  Serial.begin(9600);
  radio.openWritingPipe(Enderecos[0]); // Informamos para o transceptor qual é o endereço de escrita Enderecos[0] ou 00001 e abrimos o Pipe/Tubo.
  radio.openReadingPipe(1, Enderecos[1]); // Informamos para o transceptor qual é o endereço de leitura Enderecos[1] ou 00002 e abrimos o Pipe/Tubo. Esse primeiro parâmentro, o número 1,
  //define o número do Pipe. Podemos utilizar 6 Pipes, de 0 à 5.
  radio.setPALevel(RF24_PA_MIN); // Muda o PA para potência máxima de transmissão. Para potência minima: RF24_PA_MIN Para potência média: RF24_PA_HIGH Para potência máxima (recomendado para o módulo com etapa amplificadora e antena): RF24_PA_MAX
  radio.setDataRate(RF24_250KBPS); // Mudamos a taxa de transferência de dados para 250 Kbps, isso melhora a distância na transmissão.
  delay(1000);
}



void loop() {
/*
   //========================= Recebendo os dados ===========================//
  delay(5);

  radio.startListening();

  if ( radio.available()) {
    while (radio.available()) {
      radio.read(&DadosTransmitidos, sizeof(TipoDosDadosTXRX)); 
     }
     Serial.print("Angulo servo lido: ");
     Serial.println(DadosTransmitidos.srv_ang);
     delay(5);
  }
  
//========================= Trasmitindo os dados ===========================//
  delay(5); 
  
  radio.stopListening(); 
  int val_x = analogRead(AnalogX);
  int val_y = analogRead(AnalogY); 
  Serial.print("EIXO X: ");
  Serial.println(val_x);
  Serial.print("EIXO Y: ");
  Serial.println(val_y);
  DadosRecebidos.analog_x = val_x;
  DadosRecebidos.analog_y = map(val_y, 0, 1023, 0, 180);
  DadosRecebidos.confirm = 'k';
  radio.write(&DadosRecebidos, sizeof(TipoDosDadosTXRX));
*/

  //PARTE FINAL
  Serial.print("MODO LIDO: ");
  Serial.println(digitalRead(MODE));
  if (digitalRead(MODE)){ //caso esteja em modo de controle remoto
    Serial.println("ENTROU");
    radio.stopListening(); //Para de escutar os dados transmitidos do barco
    int val_x = analogRead(AnalogX);
    int val_y = analogRead(AnalogY); 
    DadosRecebidos.analog_x = val_x;
    DadosRecebidos.analog_y = map(val_y, 0, 1023, 0, 180);
    DadosRecebidos.control_mode = true;
    DadosRecebidos.confirm = 'k';
    radio.write(&DadosRecebidos, sizeof(TipoDosDadosTXRX));
    delay(5);
  }
  else{ //caso esteja no modo autonomo, foca em receber dados, mas deve enviar atualizações de modo/waypoints
    //========================= Recebendo os dados ===========================//
    delay(5);
    radio.startListening();
    if ( radio.available()) {
      while (radio.available()) {
        radio.read(&DadosTransmitidos, sizeof(TipoDosDadosTXRX)); 
      }
    }
    Serial.print("Angulo Bussola: ");
    Serial.println(DadosTransmitidos.srv_ang);
    Serial.print("Nivel interior da agua: ");
    Serial.println(DadosTransmitidos.nivel_agua);
    delay(5);

    //========================= Trasmitindo os dados ===========================//
    delay(5); 
    radio.stopListening(); 
    DadosRecebidos.control_mode = digitalRead(MODE);
    DadosRecebidos.confirm = 'k';
    radio.write(&DadosRecebidos, sizeof(TipoDosDadosTXRX));
  }
}
