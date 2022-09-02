#include <SPI.h> //INCLUSÃO DE BIBLIOTECA
#include <nRF24L01.h> //INCLUSÃO DE BIBLIOTECA
#include <RF24.h> //INCLUSÃO DE BIBLIOTECA

RF24 radio(7, 8); //CRIA UMA INSTÂNCIA UTILIZANDO OS PINOS (CE, CSN)     
const byte endereco[6] = "00001"; //CRIA UM ENDEREÇO PARA ENVIO DOS
//DADOS (O TRANSMISSOR E O RECEPTOR DEVEM SER CONFIGURADOS COM O MESMO ENDEREÇO)

int pinoBotao = 2; //PINO DIGITAL UTILIZADO PELO BOTÃO
int estadoBotao = 0; //VARIÁVEL PARA ARMAZENAR O ESTADO DO BOTÃO

void setup() {
  pinMode(pinoBotao, INPUT_PULLUP); //DEFINE O PINO COMO ENTRADA / "_PULLUP" É PARA ATIVAR O RESISTOR INTERNO
    //DO ARDUINO PARA GARANTIR QUE NÃO EXISTA FLUTUAÇÃO ENTRE 0 (LOW) E 1 (HIGH)
  radio.begin(); //INICIALIZA A COMUNICAÇÃO SEM FIO
  radio.openWritingPipe(endereco); //DEFINE O ENDEREÇO PARA ENVIO DE DADOS AO RECEPTOR
  radio.setPALevel(RF24_PA_HIGH);  //DEFINE O NÍVEL DO AMPLIFICADOR DE POTÊNCIA
  radio.stopListening(); //DEFINE O MÓDULO COMO TRANSMISSOR (NÃO RECEBE DADOS)
  Serial.begin(9600);
}

void loop(){
  Serial.println(digitalRead(pinoBotao));
  if(digitalRead(pinoBotao) == HIGH){ //SE A LEITURA DO PINO FOR IGUAL A HIGH, FAZ
    estadoBotao = 1; //VARIÁVEL RECEBE VALOR 1
    radio.write(&estadoBotao, sizeof(estadoBotao)); //ENVIA AO RECEPTOR OS DADOS
  }else{ //SENÃO, FAZ
        if(digitalRead(pinoBotao) == LOW){ //SE A LEITURA DO PINO FOR IGUAL A LOW, FAZ
        estadoBotao = 0; //VARIÁVEL RECEBE VALOR 0
        radio.write(&estadoBotao, sizeof(estadoBotao)); //ENVIA AO RECEPTOR OS DADOS
        }
  }
}
