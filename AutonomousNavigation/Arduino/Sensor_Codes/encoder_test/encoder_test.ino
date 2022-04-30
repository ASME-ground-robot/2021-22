#define ENCA A15

int tick = 0;
int currentstate;
int laststate;

void setup() {
  Serial.begin(115200);
  pinMode(ENCA, INPUT);
  laststate = analogRead(ENCA); 
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
  
  float ticks_per_revo = 25;
  float ratio = tick/ticks_per_revo;
  float wheel_radius = 0.127; //m
  float encoder_dist = 2*3.14*wheel_radius*ratio;
  Serial.print("Distance: ");
  Serial.println(encoder_dist);
  laststate = currentstate;
}
