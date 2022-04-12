#define Encoder_A 1
#define Encoder_B 2

boolean Direction = true;

const int enc_max = 32768;
const int enc_min = -32768;

volatile int tick_count = 0;

int interval = 1000;
long previousmillis = 0;
long currentmillis = 0;

void setup() {
  Serial.begin(115200);
  
  pinMode(Encoder_A, INPUT_PULLUP);
  pinMode(Encoder_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(Encoder_A), tick, RISING);
}

void loop() {
  currentmillis = millis();

  if (currentmillis - previousmillis > interval) {
    previousmillis = currentmillis;
    
    Serial.println("Number of Ticks: ");
    Serial.println(tick_count);
    Serial.println();
  }
}


void tick() {
  int val = digitalRead(Encoder_B);
  if (val == LOW) {
    Direction = false; //Reverse
  }
  else {
    Direction = true;
  }

  if (Direction) {
    if (tick_count == enc_max) {
      tick_count = enc_min;
    }
    else {
      tick_count++;
    }
  }

  else {
    if (tick_count == enc_min) {
      tick_count = enc_max;
    }
    else {
      tick_count--;
    }
  }
}
