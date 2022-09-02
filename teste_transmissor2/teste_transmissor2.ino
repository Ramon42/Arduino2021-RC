//======================= Incluindo bilbiotecas ==============================//
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
#include <Filter.h>
#include <Servo.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>


//========================= Criando objetos =================================//
RF24 Radio(2, 3); // CE, CSN // Instancia/cria o objeto Radio para que possamos trabalhar com ele. Também temos que informar os pinos do Arduino conectados ao CE e ao CSN do NRF24L01.
QMC5883LCompass bussola;
Servo leme;
//SoftwareSerial serial1(5, 6); // RX, TX
TinyGPS gps1;

//==================== Nomeando as constantes ==============================//
#define NV_AGUA A1
#define leme_pin 10

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
const int inAPin[2] = {7, 4};
const int inBPin[2] = {8, 9};
const int pwmPin[2] = {5, 6};
const int enPin[2] = {0, 1};
const int csPin[2] = {2, 3};
const int statPin = 13;

//=================== Criando uma estrutura/pacote de dados para a transmissão e recepção de informação através de dois Pipes =======================//
struct EstruturaDadosTXRX //criar estruturas diferentes para cada modo de operação?
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
int i, leme_ang; //Variável para contagem
int x = 0, y = 0;
float rad, angulo, declinacao, waypoint_ang;

int tol_ang = 5; //tolerancia para manter o curso (até 5 graus ele continua reto)
int tol_dist = 2; //distancia em metros de tolerancia do waypoint
int centro_leme = 70;


//FILTRO
// 20 is the weight (20 => 20%)
// 0 is the initial value of the filter
//ExponentialFilter<float> FilteredLat(20, 0);
//ExponentialFilter<float> FilteredLon(20, 0);
//DECLINACAO
int declination_degs = 20;
float declination_mins = 19;
char declination_dir = 'W';
unsigned long millisLerGPS = millis();
unsigned long millisTx = millis();

//GPS
float latitude, longitude;
float average_lat;
float average_lon;
float velocidade;
unsigned long sentido;
float distancia_entre;
float curso; //course_to == azimuth 
float vetor_nav;

int wp_number = 2; //numero total de waypoints
int wp_atual = 0; //numero do waypoint atual
double waypoints[2][2] = {
  {-23.372064, -49.513398},
  {-23.372064, -49.513398}
};

//=================== Declaração da variável global somente de leitura para armazenas os endereço de leitura e escrita =======================//
                    //Endereço: 0         1
const byte Enderecos[][6] = {"00001", "00002"}; //Criamos um vetor constante do tipo Byte que terá duas posições, pois precisamos de um endereço para escrever e outro para ler. Apesar do
//endereço ter somete 5 números, declaramos 6 para que o comppilador adicione o caracter null \0 automaticamente.

//====================== Funções =========================//
float SetDeclination( int declination_degs , int declination_mins, char declination_dir )
{
  float declination_offset_radians;
  // Convert declination to decimal degrees
  switch(declination_dir)
  {
    // North and East are positive
    case 'E':
      declination_offset_radians = ( declination_degs + (1/60 * declination_mins)) * (M_PI / 180);
      return(declination_offset_radians);

    // South and West are negative
    case 'W':
      declination_offset_radians =  0 - (( declination_degs + (1/60 * declination_mins) ) * (M_PI / 180));
      return(declination_offset_radians);
  }
}

float AdjustHeading(float heading, float declination_offset_radians)
{
  heading += declination_offset_radians;
  return heading;
}


//====================== Modo controle remoto ======================//
void Remoto(){
    delay(5);
    if (DadosRecebidos.confirm == 'k') {
      Serial.println("OK, dados recebidos");
    }
    else {
      Serial.println("Falha");
    }
    //apenas caso control_mode == true
    if (DadosRecebidos.analog_x < 500){ //ré
      //velocidade
      Serial.println("Ré");
      int aux_vel = map(DadosRecebidos.analog_x, 500, 0, 0, 255);
      motorGo(MOTOR_A, CCW, aux_vel);
      motorGo(MOTOR_B, CCW, aux_vel);
      delay(50);
    }
    else if (DadosRecebidos.analog_x > 510){ //FRENTE
      Serial.println("Frente");
      int aux_vel = map(DadosRecebidos.analog_x, 0, 500, 0, 255);
      motorGo(MOTOR_A, CW, aux_vel);
      motorGo(MOTOR_B, CW, aux_vel);
      delay(50);
    }
    else{
      Serial.println("Parado");
      motorOff(MOTOR_A);
      motorOff(MOTOR_B);
      delay(50);
    }
    //checa corrente
    if ((analogRead(csPin[0]) < currentSensingThreshhold) && (analogRead(csPin[1]) < currentSensingThreshhold))
    {
      Serial.println("Dentro do threshold de corrente");

    }
    else
    {
      Serial.println("Cuidado ai porra");
      //colocar state pra diminuir velocidade
    }
    leme.write(DadosRecebidos.analog_y);
    Serial.print("Angulo leme: ");
    Serial.println(DadosRecebidos.analog_y);
    delay(10);
}

//====================== Modo autonomo ======================//
/*
void Autonomo(){
    delay(5);
    Radio.stopListening();
    bool recebido = false;
    if ((millis() - millisLerGPS) < 2500){ //leitura do gps a cada 2,5 segundos
      millisLerGPS = millis();
      while (serial1.available()) {
        char cIn = serial1.read();
        recebido = (gps1.encode(cIn) || recebido);  //Verifica até receber o primeiro sinal dos satelites
      }
      if (recebido){
        
        unsigned long idadeInfo;
        gps1.f_get_position(&latitude, &longitude, &idadeInfo);
        if (latitude != TinyGPS::GPS_INVALID_F_ANGLE) {
          FilteredLat.Filter(latitude);
        }
        if (longitude != TinyGPS::GPS_INVALID_F_ANGLE) {
          FilteredLon.Filter(longitude);
        }
        average_lat = FilteredLat.Current();
        average_lon = FilteredLon.Current();
        
        velocidade = gps1.f_speed_kmph();   //km/h
        sentido = gps1.course();

        //haversine
        distancia_entre = gps1.distance_between(average_lat, average_lon, waypoints[wp_atual][0], waypoints[wp_atual][1]);

        //azimuth
        curso = gps1.course_to(average_lat, average_lon, waypoints[wp_atual][0], waypoints[wp_atual][1]);

      
        //BUSSOLA
        bussola.read(); //Obter o valor dos eixos X, Y e Z do Sensor
        x = bussola.getX();
        y = bussola.getY(); 
        rad = atan2(x, y);
        rad = AdjustHeading(rad, declinacao);
        angulo = rad*180/M_PI;
        //Ajuste do angulo entre 0 e 360 graus
        if(angulo < 0) 
          angulo+=360;
          
        angulo = 360-angulo;
        
        vetor_nav = curso - angulo;
        
        if (vetor_nav < -(tol_ang)){ //angulo negativo= virar para a direita
          leme.write(170);
        }
        else if (vetor_nav > tol_ang){
          leme.write(0);
        }
        else{
          leme.write(centro_leme);
        }

        //checar se a distancia do waypoint é suficiente para ir para o próximo
        if(distancia_entre < tol_dist){ //passa para o próximo waypoint
          wp_atual++;
        }
        if (wp_atual > wp_number){//caso tenha terminado todos os waypoints
            
        }
      }
    }
    if ((millis() - millisTx) < 5000){ //delay para transmitir os dados para o controle
      millisTx = millis();
      DadosRecebidos.ang_bussola = angulo;
      DadosRecebidos.ang_gps = (sentido / 100);
      DadosRecebidos.avg_lat = average_lat;
      DadosRecebidos.avg_lon = average_lon;
      DadosRecebidos.dist = distancia_entre;
      DadosRecebidos.azimuth = curso;
      DadosRecebidos.velocidade = velocidade;
      DadosRecebidos.vetor_nav = vetor_nav;
      DadosRecebidos.confirm = 'k';
      DadosRecebidos.nivel_agua = analogRead(NV_AGUA);
      Radio.write(&DadosRecebidos, sizeof(TipoDosDadosTXRX));
      Serial.print("Latitude: ");
      Serial.println(latitude, 6);  //Mostra a latitude com a precisão de 6 dígitos decimais
      Serial.print("Longitude: ");
      Serial.println(longitude, 6);  //Mostra a longitude com a precisão de 6 dígitos decimais
      
      Serial.print("LATITUDE MÉDIA: ");
      Serial.println(average_lat, 6);
      Serial.print("LONGITUDE MÉDIA: ");
      Serial.println(average_lon, 6);
      
      Serial.print("Velocidade (km/h): ");
      Serial.println(velocidade, 2);
      
      Serial.print("Sentido (grau): ");
      Serial.println(float(sentido) / 100, 2);
      
      Serial.print("Distancia até Waypoint: ");
      Serial.println(distancia_entre, 2);
      Serial.print("Azimuth: ");
      Serial.println(curso);
      
      Serial.print("ANGULO AJUSTADO FINAL:");
      Serial.println(angulo);
      Serial.print("VIRAR: "); //angulo negativo= virar para a direita
      Serial.println(vetor_nav);
    }
}
*/

//====================================== METODOS MOTOR MONSTER SHIELD ==============================================//
void motorSetup()
{
  //pinMode(statPin, OUTPUT);

  // Initialize digital pins as outputs
  for (int i = 0; i < 2; i++)
  {
    pinMode(inAPin[i], OUTPUT);
    pinMode(inBPin[i], OUTPUT);
    pinMode(pwmPin[i], OUTPUT);
  }
  // Initialize with brake applied
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(inAPin[i], LOW);
    digitalWrite(inBPin[i], LOW);
  }
}

void motorOff(uint8_t motor)
{
  // Initialize brake to Vcc
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(inAPin[i], LOW);
    digitalWrite(inBPin[i], LOW);
  }
  analogWrite(pwmPin[motor], 0);
}
void motorGo(uint8_t motor, uint8_t mode, uint8_t speed)
{

  if (motor == MOTOR_A || motor == MOTOR_B)
  {
    switch (mode)
    {
    case BRAKEVCC: // Brake to VCC
      digitalWrite(inAPin[motor], HIGH);
      digitalWrite(inBPin[motor], HIGH);
      break;
    case CW: // Turn Clockwise
      digitalWrite(inAPin[motor], HIGH);
      digitalWrite(inBPin[motor], LOW);
      break;
    case CCW: // Turn Counter-Clockwise
      digitalWrite(inAPin[motor], LOW);
      digitalWrite(inBPin[motor], HIGH);
      break;
    case BRAKEGND: // Brake to GND
      digitalWrite(inAPin[motor], LOW);
      digitalWrite(inBPin[motor], LOW);
      break;

    default:
      // Invalid mode does not change the PWM signal
      return;
    }
    analogWrite(pwmPin[motor], speed);
  }
  return;
}


void setup() {
  motorSetup();
  //==================== Declaração de entradas e saídas ========================//
  pinMode(NV_AGUA, INPUT);
  
  //================== Configurações iniciais do NRF24L01 ======================//
  Radio.begin();// Inicia o transceptor NRF24L01
  Wire.begin();
  
  leme.attach(leme_pin);
  leme.write(70);

  bussola.init();
  bussola.setCalibration(-877, 851, -953, 893, -515, 1008);
  bussola.setSmoothing(4, true);
  declinacao = SetDeclination(declination_degs, declination_mins, declination_dir);

  Serial.begin(9600);
  //serial1.begin(9600);
  
  Radio.openWritingPipe(Enderecos[1]); // Informamos para o transceptor qual é o endereço de escrita Enderecos[1] ou 00002 e abrimos o Pipe/Tubo.
  Radio.openReadingPipe(1, Enderecos[0]); // Informamos para o transceptor qual é o endereço de leitura Enderecos[0] ou 00001 e abrimos o Pipe/Tubo. Esse primeiro parâmentro, o número 1,
  //define o número do Pipe. Podemos utilizar 6 Pipes, de 0 à 5.
  Radio.setPALevel(RF24_PA_MIN);// Muda o PA para potência máxima de transmissão. Para potência minima: RF24_PA_MIN Para potência média: RF24_PA_HIGH Para potência máxima (recomendado para o módulo com etapa amplificadora e antena): RF24_PA_MAX
  Radio.setDataRate(RF24_250KBPS); // Mudamos a taxa de trasnferência de dados para 250 Kbps, isso melhora a distância na transmissão.
  delay(1000);
  Serial.print("Waypoint: ");
  Serial.print("latitude: ");
  Serial.println(waypoints[0][0], 6);
  Serial.print("longitude: ");
  Serial.println(waypoints[1][0], 6);
}


void loop() {
  delay(5);
  Serial.println("Entrou loop");
  Radio.startListening(); //escuta as informações do radio (necessário para pegar o modo de operação)
  while (!Radio.available()); //Fica em looping até receber a informação. 
  Radio.read(&DadosRecebidos, sizeof(TipoDosDadosTXRX));
  
  Serial.println(DadosRecebidos.control_mode);
  
  switch (DadosRecebidos.control_mode){
    case false:
      Remoto();
      Serial.println("Modo Controle Remoto");
      break;
    case true:
      //Autonomo();
      break;
  }
}
