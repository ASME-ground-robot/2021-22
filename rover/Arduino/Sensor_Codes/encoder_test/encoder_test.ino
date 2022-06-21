#include <CustomDualG2HighPowerMotorShield.h>
DualG2HighPowerMotorShield18v22 md;

#define ENCA A15

int tick = 0;
int currentstate;
int laststate;

void setup() {
  Serial.begin(115200);
  pinMode(ENCA, INPUT);
  laststate = analogRead(ENCA);
  
  md.init();
  // Uncomment to flip a motor's direction:
  md.flipM3(true);
  md.flipM4(true);

  delay(20000);
}

void loop() {
  currentstate = analogRead(ENCA);
  if (currentstate < laststate + 2) {
    tick++;
  }
  if (currentstate > laststate - 2) {
    tick --;
  }
  Serial.print("Ticks: ");
  Serial.println(tick);
  
  float ticks_per_revo = 125;
  float ratio = tick/ticks_per_revo;
  float wheel_radius = 0.1397; //m
  float encoder_dist = 2*PI*wheel_radius*ratio;
  Serial.print("Distance: ");
  Serial.println(encoder_dist);
  
  if (encoder_dist < 0.87) {
    md.enableDrivers();
    delay(1);
    md.setM4Speed(140); //BR
  }
  else {
    md.setM4Speed(0); //BR
  }
  laststate = currentstate;
}
