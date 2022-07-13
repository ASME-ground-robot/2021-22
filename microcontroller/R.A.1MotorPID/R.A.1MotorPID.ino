// This alternate version of the code does not require
// atomic.h. Instead, interrupts() and noInterrupts() 
// are used. Please use this code if your 
// platform does not support ATOMIC_BLOCK.

#define ENCA 21 // YELLOW
#define ENCB 27// WHITE
#define PWM 13
#define IN1 39
#define IN2 40

/*
const int enca[] = {2,3,18,19,20};
const int encb[] = {22,23,24,25,26};
const int pwm[] = {8,9,10,11,12};
const int in1[] = {49,47,46,43,42};
const int in2[] = {50,48,45,44,41};
*/
/*
 *motor 4 is dummy due to mechanical issues
const int enca[] = {2,3,18,19,20,21};
const int encb[] = {22,23,24,25,26,27};
const int pwm[] = {8,9,10,11,12,13};
const int in1[] = {49,48,45,44,41,40};
const int in2[] = {50,47,46,43,42,39};
*/

volatile int posi = 0; //
long prevT = 0;
float eprev = 0;
float eintegral = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  
  Serial.println("target pos");
}

void loop() {

  // set target position
  int target =  3000;
  //int target[1] = -500;
  //int target = -750*sin(prevT/1e6);



  // PID constants
  float kp = 1;
  float kd = 0.009; //decreases overshoot
  float ki = 0.0001;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position
  int pos = 0; 
  noInterrupts(); // disable interrupts temporarily while reading
  pos = posi;
  interrupts(); // turn interrupts back on
  
  // error
  int e = target - pos;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power in pwm
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);


  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}
