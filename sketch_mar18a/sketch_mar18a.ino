#include <Servo.h>

Servo esc_signal; 
int velocity = 0;

void setup() { 
  esc_signal.attach(9); 
  esc_signal.write(velocity); 
  delay(2000); 
}


void loop() { 
  for (velocity = 0; velocity <= 10; velocity += 1) { 
    esc_signal.write(velocity); 
    delay(100); 
  } 
  for (velocity = 10; velocity > 0; velocity -= 1) { 
    esc_signal.write(velocity); 
    delay(100); 
  } 
  esc_signal.write(0); 
  delay(5000); 
} 
