
#define joystickX A7
#define joystickY A6
#define joystick_bt 3
#define red_bt 4
#define switch_ctrl 2
#define r_pot A1
#define l_pot A0


bool ld = false;

void setup() {

  pinMode(joystickX, INPUT); //X CONTROLA VELOCIDADE
  pinMode(joystickY, INPUT); //Y CONTROLA DIREÇÃO
  pinMode(joystick_bt, INPUT);
  pinMode(red_bt, INPUT_PULLUP); //BOTÃO DE ESTADO IGUAL SWITCH
  pinMode(switch_ctrl, INPUT_PULLUP); //0= off, 1= on
  pinMode(r_pot, INPUT);
  pinMode(l_pot, INPUT);
  pinMode(5, OUTPUT);

  Serial.begin(9600);

}

void loop() {
  int val_x = analogRead(joystickX);
  int val_y = analogRead(joystickY);
  int r_pot_val = analogRead(r_pot);
  int l_pot_val = analogRead(l_pot);
  int switch_val = digitalRead(switch_ctrl);
  int red_val = digitalRead(red_bt);

  Serial.print("VALOR X: ");
  Serial.println(val_x);
  Serial.print("VALOR Y: ");
  Serial.println(val_y);
  
  Serial.print("POTENCIOMETRO DIREITA: ");
  Serial.println(r_pot_val);
  Serial.print("POTENCIOMETRO ESQUERDA: ");
  Serial.println(l_pot_val);

  Serial.print("SWITCH: ");
  Serial.println(switch_val);
  Serial.print("BOTÃO VERMELHO: ");
  Serial.println(red_val);

  digitalWrite(5, red_val);
  digitalWrite(9, red_val);
  digitalWrite(10, red_val);
  delay(1000);
}
