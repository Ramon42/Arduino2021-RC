#include <SPI.h> //INCLUSÃO DE BIBLIOTECA
#include <nRF24L01.h> //INCLUSÃO DE BIBLIOTECA
#include <RF24.h> //INCLUSÃO DE BIBLIOTECA
#include <Servo.h>

RF24 radio(7, 8); //CRIA UMA INSTÂNCIA UTILIZANDO OS PINOS (CE, CSN)
Servo servo;
const byte endereco[6] = "00001"; //CRIA UM ENDEREÇO PARA ENVIO DOS
//DADOS (O TRANSMISSOR E O RECEPTOR DEVEM SER CONFIGURADOS COM O MESMO ENDEREÇO)

int estadoBotao = 1; //VARIÁVEL PARA ARMAZENAR O ESTADO DO BOTÃO

void setup() {
  servo.attach(3);
  radio.begin(); //INICIALIZA A COMUNICAÇÃO SEM FIO
  radio.openReadingPipe(0, endereco);//DEFINE O ENDEREÇO PARA RECEBIMENTO DE DADOS VINDOS DO TRANSMISSOR
  radio.setPALevel(RF24_PA_HIGH); //DEFINE O NÍVEL DO AMPLIFICADOR DE POTÊNCIA
  radio.startListening(); //DEFINE O MÓDULO COMO RECEPTOR (NÃO ENVIA DADOS)
}

void loop(){
  if (radio.available()){ //SE A COMUNICAÇÃO ESTIVER HABILITADA, FAZ
    char text[32] = ""; //VARIÁVEL RESPONSÁVEL POR ARMAZENAR OS DADOS RECEBIDOS
    radio.read(&estadoBotao, sizeof(estadoBotao)); // LÊ OS DADOS RECEBIDOS
    
  if(estadoBotao == 0){ //SE O PARÂMETRO RECEBIDO (ESTADO ATUAL DO BOTÃO) FOR IGUAL A 0 (PRESSIONADO), FAZ
    servo.write(180);
  }else{
        if(estadoBotao == 1){ //SE O PARÂMETRO RECEBIDO (ESTADO ATUAL DO BOTÃO) FOR IGUAL A 1 (NÃO PRESSIONADO), FAZ
        servo.write(0);
        }
    }
  }
  delay(5); //INTERVALO DE 5 MILISSEGUNDOS
}
