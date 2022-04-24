#pragma once

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || \
    defined(__AVR_ATmega328PB__) || defined (__AVR_ATmega32U4__) || \
	defined(__AVR_ATmega16U4__) || defined(__AVR_ATmega1280__) || \
    defined(__AVR_ATmega2560__)
  // Timers generally available for all boards.
  #define DUALG2HIGHPOWERMOTORSHIELD_TIMER1_AVAILABLE
#endif

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  // Additional timers for an Arduino Mega.
  #define DUALG2HIGHPOWERMOTORSHIELD_TIMER2_AVAILABLE
  #define DUALG2HIGHPOWERMOTORSHIELD_TIMER3_AVAILABLE
  #define DUALG2HIGHPOWERMOTORSHIELD_TIMER4_AVAILABLE
  #define DUALG2HIGHPOWERMOTORSHIELD_TIMER5_AVAILABLE
#endif


#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
// Statement above will allow very dated arduino models to compile

class DualG2HighPowerMotorShield
{
  public:
    // CONSTRUCTORS
    DualG2HighPowerMotorShield();
	
	/* //Default Pin Selection
    DualG2HighPowerMotorShield(unsigned char M1nSLEEP,
                               unsigned char M1DIR,
                               unsigned char M1nFAULT,
                               unsigned char M1CS,
                               unsigned char M2nSLEEP,
                               unsigned char M2DIR,
                               unsigned char M2nFAULT,
                               unsigned char M2CS							   
							   unsigned char M3nSLEEP,
                               unsigned char M3DIR,
                               unsigned char M3nFAULT,
                               unsigned char M3CS,
                               unsigned char M4nSLEEP,
                               unsigned char M4DIR,
                               unsigned char M4nFAULT,
                               unsigned char M4CS); */
							   
	/* //User Defined Pin Selection
    DualG2HighPowerMotorShield(unsigned char M1nSLEEP,
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
                               unsigned char M4CS);
							   
	//User Defined Pin Selection only for motor driver 2
    DualG2HighPowerMotorShield(unsigned char M3nSLEEP,
                               unsigned char M3DIR,
                               unsigned char M3PWM,
                               unsigned char M3nFAULT,
                               unsigned char M3CS,
                               unsigned char M4nSLEEP,
                               unsigned char M4DIR,
                               unsigned char M4PWM,
                               unsigned char M4nFAULT,
                               unsigned char M4CS); */

    // PUBLIC METHODS
    void init(); // Initialize TIMERS, set the PWM to 20kHZ.
    void setM1Speed(int speed); // Set speed for M1.
    void setM2Speed(int speed); // Set speed for M2.
	void setM3Speed(int speed); // Set speed for M3.
    void setM4Speed(int speed); // Set speed for M4.
	
    void setSpeeds(int m1Speed, int m2Speed, int m3speed, int m4speed); // Set speed for M1,M2,M3,M4.
    unsigned char getM1Fault(); // Get fault reading from M1.
    unsigned char getM2Fault(); // Get fault reading from M2.
	 unsigned char getM3Fault(); // Get fault reading from M3.
    unsigned char getM4Fault(); // Get fault reading from M4.
	
    void flipM1(boolean flip); // Flip the direction of the speed for M1.
    void flipM2(boolean flip); // Flip the direction of the speed for M2.
	void flipM3(boolean flip); // Flip the direction of the speed for M3.
    void flipM4(boolean flip); // Flip the direction of the speed for M4.
	
    void enableM1Driver(); // Enables the MOSFET driver for M1.
    void enableM2Driver(); // Enables the MOSFET driver for M2.
	void enableM3Driver(); // Enables the MOSFET driver for M3.
    void enableM4Driver(); // Enables the MOSFET driver for M4.	
    void enableDrivers(); // Enables the MOSFET drivers for both M1,M2,M3,M4.
	
    void disableM1Driver(); // Puts the MOSFET driver for M1 into sleep mode.
    void disableM2Driver(); // Puts the MOSFET driver for M2 into sleep mode.
	void disableM3Driver(); // Puts the MOSFET driver for M3 into sleep mode.
    void disableM4Driver(); // Puts the MOSFET driver for M4 into sleep mode.	
    void disableDrivers(); // Puts the MOSFET drivers for both M1 and M2 into sleep mode.
	
    unsigned int getM1CurrentReading();
    unsigned int getM2CurrentReading();
	unsigned int getM3CurrentReading();
    unsigned int getM4CurrentReading();
	
    void calibrateM1CurrentOffset();
    void calibrateM2CurrentOffset();
	void calibrateM3CurrentOffset();
    void calibrateM4CurrentOffset();
    void calibrateCurrentOffsets();
	
    unsigned int getM1CurrentMilliamps(int gain);
    unsigned int getM2CurrentMilliamps(int gain);
	unsigned int getM3CurrentMilliamps(int gain);
    unsigned int getM4CurrentMilliamps(int gain);
  protected:
    unsigned int _offsetM1;
    unsigned int _offsetM2;
	unsigned int _offsetM3;
    unsigned int _offsetM4;

  private:
    unsigned char _M1PWM;
    static const unsigned char _M1PWM_TIMER1_PIN = 9; //For Arduino Uno
	static const unsigned char _M1PWM_TIMER2_PIN = 9; //For Arduino Mega
    unsigned char _M2PWM;
    static const unsigned char _M2PWM_TIMER1_PIN = 10; //For Arduino Uno
	static const unsigned char _M2PWM_TIMER2_PIN = 10; //For Arduino Mega
	unsigned char _M3PWM;
    static const unsigned char _M3PWM_TIMER5_PIN = 45;
    unsigned char _M4PWM;
    static const unsigned char _M4PWM_TIMER5_PIN = 46;
	
	
    unsigned char _M1nSLEEP;
    unsigned char _M2nSLEEP;
	unsigned char _M3nSLEEP;
    unsigned char _M4nSLEEP;
	
    unsigned char _M1DIR;
    unsigned char _M2DIR;
	unsigned char _M3DIR;
    unsigned char _M4DIR;
	
    unsigned char _M1nFAULT;
    unsigned char _M2nFAULT;
	unsigned char _M3nFAULT;
    unsigned char _M4nFAULT;
	
    unsigned char _M1CS;
    unsigned char _M2CS;
	unsigned char _M3CS;
    unsigned char _M4CS;
	
    static boolean _flipM1;
    static boolean _flipM2;
	static boolean _flipM3;
    static boolean _flipM4;

};

class DualG2HighPowerMotorShield24v14 : public DualG2HighPowerMotorShield
{
  public:
    using DualG2HighPowerMotorShield::DualG2HighPowerMotorShield;
    unsigned int getM1CurrentMilliamps(); // Get current reading for M1.
    unsigned int getM2CurrentMilliamps(); // Get current reading for M2.
	unsigned int getM3CurrentMilliamps(); // Get current reading for M3.
    unsigned int getM4CurrentMilliamps(); // Get current reading for M4.
};

class DualG2HighPowerMotorShield18v18 : public DualG2HighPowerMotorShield
{
  public:
    using DualG2HighPowerMotorShield::DualG2HighPowerMotorShield;
    unsigned int getM1CurrentMilliamps(); // Get current reading for M1.
    unsigned int getM2CurrentMilliamps(); // Get current reading for M2.
	unsigned int getM3CurrentMilliamps(); // Get current reading for M3.
    unsigned int getM4CurrentMilliamps(); // Get current reading for M4.
};

class DualG2HighPowerMotorShield24v18 : public DualG2HighPowerMotorShield
{
  public:
    using DualG2HighPowerMotorShield::DualG2HighPowerMotorShield;
    unsigned int getM1CurrentMilliamps(); // Get current reading for M1.
    unsigned int getM2CurrentMilliamps(); // Get current reading for M2.
	unsigned int getM3CurrentMilliamps(); // Get current reading for M3.
    unsigned int getM4CurrentMilliamps(); // Get current reading for M4.
};

class DualG2HighPowerMotorShield18v22 : public DualG2HighPowerMotorShield
{
  public:
    using DualG2HighPowerMotorShield::DualG2HighPowerMotorShield;
    unsigned int getM1CurrentMilliamps(); // Get current reading for M1.
    unsigned int getM2CurrentMilliamps(); // Get current reading for M2.
	unsigned int getM3CurrentMilliamps(); // Get current reading for M3.
    unsigned int getM4CurrentMilliamps(); // Get current reading for M4.
};
