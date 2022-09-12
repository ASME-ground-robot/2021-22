#include "CustomDualG2HighPowerMotorShield.h"

boolean DualG2HighPowerMotorShield::_flipM1 = false;
boolean DualG2HighPowerMotorShield::_flipM2 = false;
boolean DualG2HighPowerMotorShield::_flipM3 = false;
boolean DualG2HighPowerMotorShield::_flipM4 = false;

// Constructors ////////////////////////////////////////////////////////////////

DualG2HighPowerMotorShield::DualG2HighPowerMotorShield()
{
// Green: Top MD
  _M1nSLEEP = 25;
  _M1nFAULT = 23;
  _M1DIR = 27;
  _M1PWM = 9;
  _M1CS = A0;

//Yellow: Top MD
  _M2nSLEEP = 31;
  _M2nFAULT = 29;
  _M2DIR = 33;
  _M2PWM = 10;
  _M2CS = A1;
  
//Blue: Bottom MD
  _M3nSLEEP = 24;
  _M3nFAULT = 22;
  _M3DIR = 26;
  _M3PWM = 45;
  _M3CS = A2;
  
//Orange: Bottom MD
  _M4nSLEEP = 30;
  _M4nFAULT = 28;
  _M4DIR = 32;
  _M4PWM = 46;
  _M4CS = A3;
}

/* DualG2HighPowerMotorShield::DualG2HighPowerMotorShield(unsigned char M1nSLEEP,
                                                       unsigned char M1DIR,
                                                       unsigned char M1PWM,
                                                       unsigned char M1nFAULT,
                                                       unsigned char M1CS,
                                                       unsigned char M2nSLEEP,
                                                       unsigned char M2DIR,
                                                       unsigned char M2PWM,
                                                       unsigned char M2nFAULT,
                                                       unsigned char M2CS
													   unsigned char M3nSLEEP,
                                                       unsigned char M3DIR,
                                                       unsigned char M3PWM,
                                                       unsigned char M3nFAULT,
                                                       unsigned char M3CS,
                                                       unsigned char M4nSLEEP,
                                                       unsigned char M4DIR,
                                                       unsigned char M4PWM,
                                                       unsigned char M4nFAULT,
                                                       unsigned char M4CS)
{
  _M1nSLEEP = M1nSLEEP;
  _M1nFAULT = M1nFAULT;
  _M1DIR = M1DIR;
  _M1PWM = M1PWM;
  _M1CS = M1CS;

  _M2nSLEEP = M2nSLEEP;
  _M2nFAULT = M2nFAULT;
  _M2DIR = M2DIR;
  _M2PWM = M2PWM;
  _M2CS = M2CS;
  
  _M3nSLEEP = M3nSLEEP;
  _M3nFAULT = M3nFAULT;
  _M3DIR = M3DIR;
  _M3PWM = M3PWM;
  _M3CS = M3CS;
  
  _M4nSLEEP = M4nSLEEP;
  _M4nFAULT = M4nFAULT;
  _M4DIR = M4DIR;
  _M4PWM = M4PWM;
  _M4CS = M4CS;
} */

// Public Methods //////////////////////////////////////////////////////////////
void DualG2HighPowerMotorShield::init()
{
  pinMode(_M1nSLEEP, OUTPUT);
  pinMode(_M2nSLEEP, OUTPUT);
  pinMode(_M1PWM, OUTPUT);
  pinMode(_M1nFAULT, INPUT_PULLUP);
  pinMode(_M1CS, INPUT);
  pinMode(_M1DIR, OUTPUT);
  pinMode(_M2DIR, OUTPUT);
  pinMode(_M2PWM, OUTPUT);
  pinMode(_M2nFAULT, INPUT_PULLUP);
  pinMode(_M2CS, INPUT);

#ifdef DUALG2HIGHPOWERMOTORSHIELD_TIMER1_AVAILABLE
  if (_M1PWM == _M1PWM_TIMER1_PIN && _M2PWM == _M2PWM_TIMER1_PIN)
  {
    // Timer 1 configuration
    // prescaler: clockI/O / 1
    // outputs enabled
    // phase-correct PWM
    // top of 400
    //
    // PWM frequency calculation
    // 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
    TCCR1A = 0b10100000;
    TCCR1B = 0b00010001;
    ICR1 = 400;
  }
#endif

#ifdef DUALG2HIGHPOWERMOTORSHIELD_TIMER2_AVAILABLE
	if (_M1PWM == _M1PWM_TIMER2_PIN && _M2PWM == _M2PWM_TIMER2_PIN)
	{
		// Timer 2 configuration
		// prescaler: clockI/O / 1
		// outputs enabled
		// fast PWM
		// top of 255
		//
		// PWM frequency calculation
		// 16MHz / 8 (prescaler) / 1 (fast PWM) / 255 (top) = 7.8kHz
		TCCR2A = 0b10100011; //11 in first two bits = fast PWM (could switch to phase correct PWM by making first two bits = 01, PWM frequency would be 3.9kHz)
		TCCR2B = 0b00000010; //first 3 bits are prescaler
	}
  #endif

  pinMode(_M3nSLEEP, OUTPUT);
  pinMode(_M4nSLEEP, OUTPUT);
  pinMode(_M3PWM, OUTPUT);
  pinMode(_M3nFAULT, INPUT_PULLUP);
  pinMode(_M3CS, INPUT);
  pinMode(_M3DIR, OUTPUT);
  pinMode(_M4DIR, OUTPUT);
  pinMode(_M4PWM, OUTPUT);
  pinMode(_M4nFAULT, INPUT_PULLUP);
  pinMode(_M4CS, INPUT);

#ifdef DUALG2HIGHPOWERMOTORSHIELD_TIMER5_AVAILABLE
    if (_M3PWM == _M3PWM_TIMER5_PIN && _M4PWM == _M4PWM_TIMER5_PIN)
    {
      // Timer 5 configuration
      // prescaler: clockI/O / 1
      // outputs enabled
      // phase-correct PWM
      // top of 400
      //
      // PWM frequency calculation
      // 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
      TCCR5A = 0b10100000;
      TCCR5B = 0b00010001;
      ICR5 = 400;
    }
  #endif

}
// Set speed for motor 1, speed is a number betwenn -400 and 400
void DualG2HighPowerMotorShield::setM1Speed(int speed)
{
  unsigned char reverse = 0;

  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 400)  // Max PWM dutycycle
    speed = 400;
	

#ifdef DUALG2HIGHPOWERMOTORSHIELD_TIMER1_AVAILABLE
  if (_M1PWM == _M1PWM_TIMER1_PIN && _M2PWM == _M2PWM_TIMER1_PIN)
  {
    OCR1A = speed;
  }
  else
  {
    analogWrite(_M1PWM, speed * 51 / 80); // map 400 to 255
  }
#else
  analogWrite(_M1PWM, speed * 51 / 80); // map 400 to 255
#endif

#ifdef DUALG2HIGHPOWERMOTORSHIELD_TIMER2_AVAILABLE
    if (_M1PWM == _M1PWM_TIMER2_PIN && _M2PWM == _M2PWM_TIMER2_PIN)
    {
      OCR2B = speed*51/80;
    }
    else
    {
      analogWrite(_M1PWM,speed * 51 / 80); // map 400 to 255
    }
  #else
    analogWrite(_M1PWM, speed * 51 / 80); // map 400 to 255
  #endif

  if (reverse ^ _flipM1) // flip if speed was negative or _flipM1 setting is active, but not both
  {
    digitalWrite(_M1DIR, HIGH);
  }
  else
  {
    digitalWrite(_M1DIR, LOW);
  }
}

// Set speed for motor 2, speed is a number betwenn -400 and 400
void DualG2HighPowerMotorShield::setM2Speed(int speed)
{
  unsigned char reverse = 0;

  if (speed < 0)
  {
    speed = -speed;  // make speed a positive quantity
    reverse = 1;  // preserve the direction
  }
  if (speed > 400)  // Max
    speed = 400;

#ifdef DUALG2HIGHPOWERMOTORSHIELD_TIMER1_AVAILABLE
  if (_M1PWM == _M1PWM_TIMER1_PIN && _M2PWM == _M2PWM_TIMER1_PIN)
  {
    OCR1B = speed;
  }
  else
  {
    analogWrite(_M2PWM, speed * 51 / 80); // map 400 to 255
  }
#else
  analogWrite(_M2PWM, speed * 51 / 80); // map 400 to 255
#endif

#ifdef DUALG2HIGHPOWERMOTORSHIELD_TIMER2_AVAILABLE
    if (_M1PWM == _M1PWM_TIMER2_PIN && _M2PWM == _M2PWM_TIMER2_PIN)
    {
      OCR2A = speed*51/80;
    }
    else
    {
      analogWrite(_M2PWM,speed * 51 / 80); // map 400 to 255
    }
  #else
    analogWrite(_M2PWM, speed * 51 / 80); // map 400 to 255
  #endif

  if (reverse ^ _flipM2) // flip if speed was negative or _flipM1 setting is active, but not both
  {
    digitalWrite(_M2DIR, HIGH);
  }
  else
  {
    digitalWrite(_M2DIR, LOW);
  }
}

// Set speed for motor 3, speed is a number betwenn -400 and 400
void DualG2HighPowerMotorShield::setM3Speed(int speed)
{
  unsigned char reverse = 0;

  if (speed < 0)
  {
    speed = -speed;  // make speed a positive quantity
    reverse = 1;  // preserve the direction
  }
  if (speed > 400)  // Max
    speed = 400;

#ifdef DUALG2HIGHPOWERMOTORSHIELD_TIMER5_AVAILABLE
    if (_M3PWM == _M3PWM_TIMER5_PIN && _M4PWM == _M4PWM_TIMER5_PIN)
    {
      OCR5B = speed;
    }
    else
    {
      analogWrite(_M3PWM,speed * 51 / 80); // map 400 to 255
    }
  #else
    analogWrite(_M3PWM,speed * 51 / 80); // map 400 to 255
  #endif


  if (reverse ^ _flipM3) // flip if speed was negative or _flipM1 setting is active, but not both
  {
    digitalWrite(_M3DIR, HIGH);
  }
  else
  {
    digitalWrite(_M3DIR, LOW);
  }
}



// Set speed for motor 4, speed is a number betwenn -400 and 400
void DualG2HighPowerMotorShield::setM4Speed(int speed)
{
  unsigned char reverse = 0;

  if (speed < 0)
  {
    speed = -speed;  // make speed a positive quantity
    reverse = 1;  // preserve the direction
  }
  if (speed > 400)  // Max
    speed = 400;

#ifdef DUALG2HIGHPOWERMOTORSHIELD_TIMER5_AVAILABLE
    if (_M3PWM == _M3PWM_TIMER5_PIN && _M4PWM == _M4PWM_TIMER5_PIN)
    {
      OCR5A = speed;
    }
    else
    {
      analogWrite(_M4PWM,speed * 51 / 80); // map 400 to 255
    }
  #else
    analogWrite(_M4PWM,speed * 51 / 80); // map 400 to 255
  #endif


  if (reverse ^ _flipM4) // flip if speed was negative or _flipM1 setting is active, but not both
  {
    digitalWrite(_M4DIR, HIGH);
  }
  else
  {
    digitalWrite(_M4DIR, LOW);
  }
}




// Set speed for motor 1 and 2
void DualG2HighPowerMotorShield::setSpeeds(int m1Speed, int m2Speed,int m3Speed,int m4Speed)
{
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
  setM3Speed(m3Speed);
  setM4Speed(m4Speed);
}

// Return error status for motor 1
unsigned char DualG2HighPowerMotorShield::getM1Fault()
{
  return !digitalRead(_M1nFAULT);
}

// Return error status for motor 2
unsigned char DualG2HighPowerMotorShield::getM2Fault()
{
  return !digitalRead(_M2nFAULT);
}

// Return error status for motor 3
unsigned char DualG2HighPowerMotorShield::getM3Fault()
{
  return !digitalRead(_M3nFAULT);
}

// Return error status for motor 4
unsigned char DualG2HighPowerMotorShield::getM4Fault()
{
  return !digitalRead(_M4nFAULT);
}

void DualG2HighPowerMotorShield::flipM1(boolean flip)
{
  DualG2HighPowerMotorShield::_flipM1 = flip;
}

void DualG2HighPowerMotorShield::flipM2(boolean flip)
{
  DualG2HighPowerMotorShield::_flipM2 = flip;
}

void DualG2HighPowerMotorShield::flipM3(boolean flip)
{
  DualG2HighPowerMotorShield::_flipM3 = flip;
}

void DualG2HighPowerMotorShield::flipM4(boolean flip)
{
  DualG2HighPowerMotorShield::_flipM4 = flip;
}

// Enables the MOSFET driver for M1.
void DualG2HighPowerMotorShield::enableM1Driver()
{
  digitalWrite(_M1nSLEEP, HIGH);
}

// Enables the MOSFET driver for M2.
void DualG2HighPowerMotorShield::enableM2Driver()
{
  digitalWrite(_M2nSLEEP, HIGH);
}

// Enables the MOSFET driver for M3.
void DualG2HighPowerMotorShield::enableM3Driver()
{
  digitalWrite(_M3nSLEEP, HIGH);
}

// Enables the MOSFET driver for M4.
void DualG2HighPowerMotorShield::enableM4Driver()
{
  digitalWrite(_M4nSLEEP, HIGH);
}

// Enables the MOSFET drivers for both M1 and M2.
void DualG2HighPowerMotorShield::enableDrivers()
{
  enableM1Driver();
  enableM2Driver();
  enableM3Driver();
  enableM4Driver();
}

// Puts the MOSFET driver for M1 into sleep mode.
void DualG2HighPowerMotorShield::disableM1Driver()
{
  digitalWrite(_M1nSLEEP, LOW);
}

// Puts the MOSFET driver for M2 into sleep mode.
void DualG2HighPowerMotorShield::disableM2Driver()
{
  digitalWrite(_M2nSLEEP, LOW);
}

// Puts the MOSFET driver for M3 into sleep mode.
void DualG2HighPowerMotorShield::disableM3Driver()
{
  digitalWrite(_M3nSLEEP, LOW);
}

// Puts the MOSFET driver for M2 into sleep mode.
void DualG2HighPowerMotorShield::disableM4Driver()
{
  digitalWrite(_M4nSLEEP, LOW);
}

// Puts the MOSFET drivers for both M1 and M2 into sleep mode.
void DualG2HighPowerMotorShield::disableDrivers()
{
  disableM1Driver();
  disableM2Driver();
  disableM3Driver();
  disableM4Driver();
}

unsigned int DualG2HighPowerMotorShield::getM1CurrentReading()
{
  return analogRead(_M1CS);
}

unsigned int DualG2HighPowerMotorShield::getM2CurrentReading()
{
  return analogRead(_M2CS);
}

unsigned int DualG2HighPowerMotorShield::getM3CurrentReading()
{
  return analogRead(_M3CS);
}

unsigned int DualG2HighPowerMotorShield::getM4CurrentReading()
{
  return analogRead(_M4CS);
}

// Set voltage offset of M1 current reading at 0 speed.
void DualG2HighPowerMotorShield::calibrateM1CurrentOffset()
{
  setM1Speed(0);
  enableM1Driver();
  delay(1);
  DualG2HighPowerMotorShield::_offsetM1 = getM1CurrentReading();
}

// Set voltage offset of M2 current reading at 0 speed.
void DualG2HighPowerMotorShield::calibrateM2CurrentOffset()
{
  setM2Speed(0);
  enableM2Driver();
  delay(1);
  DualG2HighPowerMotorShield::_offsetM2 = getM2CurrentReading();
}

// Set voltage offset of M3 current reading at 0 speed.
void DualG2HighPowerMotorShield::calibrateM3CurrentOffset()
{
  setM3Speed(0);
  enableM3Driver();
  delay(1);
  DualG2HighPowerMotorShield::_offsetM3 = getM3CurrentReading();
}

// Set voltage offset of M4 current reading at 0 speed.
void DualG2HighPowerMotorShield::calibrateM4CurrentOffset()
{
  setM4Speed(0);
  enableM4Driver();
  delay(1);
  DualG2HighPowerMotorShield::_offsetM4 = getM4CurrentReading();
}

// Get voltage offset of M1,M2,M3,M4 current readings.
void DualG2HighPowerMotorShield::calibrateCurrentOffsets()
{
  setSpeeds( 0, 0,0,0);
  enableDrivers();
  delay(1);
  DualG2HighPowerMotorShield::_offsetM1 = getM1CurrentReading();
  DualG2HighPowerMotorShield::_offsetM2 = getM2CurrentReading();
  DualG2HighPowerMotorShield::_offsetM3 = getM3CurrentReading();
  DualG2HighPowerMotorShield::_offsetM4 = getM4CurrentReading();
}


// Return M1 current value in milliamps using the gain value for the specific version.
unsigned int DualG2HighPowerMotorShield::getM1CurrentMilliamps(int gain)
{
  // 5V / 1024 ADC counts / gain mV per A
  // The 24v14, 18v18 and 24v18 results in 244 mA per count.
  // The 18v22 results in 488 mA per count.
  unsigned int mAPerCount = 5000000/1024/gain;
  int reading = (getM1CurrentReading() - DualG2HighPowerMotorShield::_offsetM1);
  if (reading > 0)
  {
    return reading * mAPerCount;
  }
  return 0;
}

// Return M2 current value in milliamps using the gain value for the specific version.
unsigned int DualG2HighPowerMotorShield::getM2CurrentMilliamps(int gain)
{
  // 5V / 1024 ADC counts / gain mV per A
  // The 24v14, 18v18 and 24v18 results in 244 mA per count.
  // The 18v22 results in 488 mA per count.
  unsigned int mAPerCount = 5000000/1024/gain;
  int reading = (getM2CurrentReading() - DualG2HighPowerMotorShield::_offsetM2);
  if (reading > 0)
  {
    return reading * mAPerCount;
  }
  return 0;
}

// Return M3 current value in milliamps using the gain value for the specific version.
unsigned int DualG2HighPowerMotorShield::getM3CurrentMilliamps(int gain)
{
  // 5V / 1024 ADC counts / gain mV per A
  // The 24v14, 18v18 and 24v18 results in 244 mA per count.
  // The 18v22 results in 488 mA per count.
  unsigned int mAPerCount = 5000000/1024/gain;
  int reading = (getM3CurrentReading() - DualG2HighPowerMotorShield::_offsetM3);
  if (reading > 0)
  {
    return reading * mAPerCount;
  }
  return 0;
}

// Return M4 current value in milliamps using the gain value for the specific version.
unsigned int DualG2HighPowerMotorShield::getM4CurrentMilliamps(int gain)
{
  // 5V / 1024 ADC counts / gain mV per A
  // The 24v14, 18v18 and 24v18 results in 244 mA per count.
  // The 18v22 results in 488 mA per count.
  unsigned int mAPerCount = 5000000/1024/gain;
  int reading = (getM4CurrentReading() - DualG2HighPowerMotorShield::_offsetM4);
  if (reading > 0)
  {
    return reading * mAPerCount;
  }
  return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Return M1 current value in milliamps for 24v14 version.
unsigned int DualG2HighPowerMotorShield24v14::getM1CurrentMilliamps()
{
  return DualG2HighPowerMotorShield::getM1CurrentMilliamps(20);
}

// Return M2 current value in milliamps for 24v14 version.
unsigned int DualG2HighPowerMotorShield24v14::getM2CurrentMilliamps()
{
  return DualG2HighPowerMotorShield::getM2CurrentMilliamps(20);
}

// Return M3 current value in milliamps for 24v14 version.
unsigned int DualG2HighPowerMotorShield24v14::getM3CurrentMilliamps()
{
  return DualG2HighPowerMotorShield::getM3CurrentMilliamps(20);
}

// Return M4 current value in milliamps for 24v14 version.
unsigned int DualG2HighPowerMotorShield24v14::getM4CurrentMilliamps()
{
  return DualG2HighPowerMotorShield::getM4CurrentMilliamps(20);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Return M1 current value in milliamps for 18v18 version.
unsigned int DualG2HighPowerMotorShield18v18::getM1CurrentMilliamps()
{
  return DualG2HighPowerMotorShield::getM1CurrentMilliamps(20);
}

// Return M2 current value in milliamps for 18v18 version.
unsigned int DualG2HighPowerMotorShield18v18::getM2CurrentMilliamps()
{
  return DualG2HighPowerMotorShield::getM2CurrentMilliamps(20);
}

// Return M1 current value in milliamps for 24v18 version.
unsigned int DualG2HighPowerMotorShield24v18::getM1CurrentMilliamps()
{
  return DualG2HighPowerMotorShield::getM1CurrentMilliamps(20);
}

// Return M2 current value in milliamps for 24v18 version.
unsigned int DualG2HighPowerMotorShield24v18::getM2CurrentMilliamps()
{
  return DualG2HighPowerMotorShield::getM2CurrentMilliamps(20);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Return M1 current value in milliamps for 18v22 version.
unsigned int DualG2HighPowerMotorShield18v22::getM1CurrentMilliamps()
{
  return DualG2HighPowerMotorShield::getM1CurrentMilliamps(10);
}

// Return M2 current value in milliamps for 18v22 version.
unsigned int DualG2HighPowerMotorShield18v22::getM2CurrentMilliamps()
{
  return DualG2HighPowerMotorShield::getM2CurrentMilliamps(10);
}

// Return M3 current value in milliamps for 18v22 version.
unsigned int DualG2HighPowerMotorShield18v22::getM3CurrentMilliamps()
{
  return DualG2HighPowerMotorShield::getM3CurrentMilliamps(10);
}

// Return M4 current value in milliamps for 18v22 version.
unsigned int DualG2HighPowerMotorShield18v22::getM4CurrentMilliamps()
{
  return DualG2HighPowerMotorShield::getM4CurrentMilliamps(10);
}
