
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
#include <Servo.h>
#include <Filter.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define leme_pin 10
//========================= Criando objetos =================================//
RF24 radio(2, 3); // CE, CSN // Instancia/cria o objeto Radio para que possamos trabalhar com ele. Também temos que informar os pinos do Arduino conectados ao CE e ao CSN do NRF24L01.
QMC5883LCompass bussola;
Servo leme;
TinyGPSPlus gps;
SoftwareSerial serialGPS (8, 9);

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
// ======== WAYPOINTS ======== //
double waypoints[][5] = {
  { -23.372064, -49.513398},
  { -23.372064, -49.513398}
};

int leme_ang;
bool autonomo = false;
float vetor_nav;

//VARIAVEIS PARA GPS
float avg_lat, avg_lon;
float dist_waypoint; //haversine
float azimuth_waypoint; //course_to

//VARIAVEIS PARA BUSSOLA
int x = 0, y = 0;
float ang, rad, rad_m, declinacao;
//DECLINACAO
int declination_degs = 20;
float declination_mins = 19;
char declination_dir = 'W';

//TIMMINGS
unsigned long currentMillis;
unsigned long prevMillis;
unsigned long txIntervalMillis = 1000;

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
  radio.begin();// Inicia o transceptor NRF24L01
  radio.openWritingPipe(Enderecos[1]);
  radio.openReadingPipe(1, Enderecos[0]);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  Wire.begin();
  radio.setRetries(3, 5); // delay, count
  send(); // to get things started
  prevMillis = millis(); // set clock


  //LEME
  leme.attach(leme_pin);
  leme.write(70);

  //SERIAL
  Serial.begin(9600);
  serialGPS.begin(9600);

  //BUSSOLA
  bussola.init();
  bussola.setCalibration(-877, 851, -953, 893, -515, 1008);
  bussola.setSmoothing(4, true);
  declinacao = SetDeclination(declination_degs, declination_mins, declination_dir);
}


void loop() {
  delay(5);// Tempo para não haver perca de dados
  switch (autonomo) {
    case true:
      modo_autonomo();
      send();
      getData();
      break;
    case false:
      getData();
      modo_controle();
      send();
      break;
  }
  currentMillis = millis();
  if (currentMillis - prevMillis >= txIntervalMillis) {
    Serial.print("LEME: ");
    Serial.println(DadosControle.leme_ang);
    Serial.print("EIXO X: ");
    Serial.println(DadosControle.analog_x);
    Serial.print("EIXO Y: ");
    Serial.println(DadosControle.analog_y);
    prevMillis = millis();
  }
  getData();
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

void getGPS() {
  if (serialGPS.available() > 0) {
    if (gps.encode(serialGPS.read())) {
      if (gps.location.isValid()) {
        FilteredLat.Filter(gps.location.lat());
        FilteredLon.Filter(gps.location.lng());
        avg_lat = FilteredLat.Current();
        avg_lon = FilteredLon.Current();
        dist_waypoint = gps.distanceBetween(
                          avg_lat,
                          avg_lon,
                          waypoints[0][0],
                          waypoints[0][1]
                        );
        azimuth_waypoint = gps.courseTo(
                             avg_lat,
                             avg_lon,
                             waypoints[0][0],
                             waypoints[0][1]
                           );
        DadosAutonomo.avg_lat = avg_lat;
        DadosAutonomo.avg_lon = avg_lon;
        DadosAutonomo.dist_waypoint = dist_waypoint;
        DadosAutonomo.azimuth_waypoint = azimuth_waypoint;
      }
    }
  }
}

void getCompass() {
  for (int i = 0; i < 50; i++) {
    bussola.read();
    x = bussola.getX();
    y = bussola.getY();
    rad_m += atan2(x, y);
  }
  rad = rad_m / 50;
  rad = AdjustHeading(rad, declinacao);
  ang = rad * 180 / M_PI;
  if (ang < 0) ang += 360;
  ang = 360 - ang;
  DadosAutonomo.bussola = ang;
}

float getVetor() {
  return (DadosAutonomo.azimuth_waypoint - DadosAutonomo.bussola);
}

void send() {
  radio.stopListening();
  switch (autonomo) {
    case true:
      radio.write(&DadosAutonomo, sizeof(TipoAutonomoTXRX));
      break;
    case false:
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
        autonomo = DadosAutonomo.autonomo;
        break;
      case false:
        radio.read(&DadosControle, sizeof(TipoControleTXRX));
        autonomo = DadosControle.autonomo;
        break;
    }
  }
}

void modo_controle() {
  if (DadosControle.analog_x < 500) { //ré
    motorGo(MOTOR_A, CCW, DadosControle.max_speed); //motor direita
    motorGo(MOTOR_B, CCW, DadosControle.max_speed); //motor esquerda
    delay(5);
  }
  else if (DadosControle.analog_x > 510) {
    motorGo(MOTOR_A, CW, DadosControle.max_speed);
    motorGo(MOTOR_B, CW, DadosControle.max_speed);
    delay(5);
  }
  else {
    motorOff(MOTOR_A);
    motorOff(MOTOR_B);
    delay(5);
  }
  leme.write(DadosControle.leme_ang);
  delay(5);

}

void modo_autonomo() {
  getGPS();
  getCompass();
  vetor_nav = getVetor();
}
