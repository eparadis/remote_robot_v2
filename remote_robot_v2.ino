#include <PID_v1.h>    // http://playground.arduino.cc/Code/PIDLibrary
#include <PinChangeInt.h>  // http://code.google.com/p/arduino-pinchangeint/wiki

// encoders
const byte pinQA_I = 2;
const byte pinQA_Q = 5;
const byte pinQB_I = 3;
const byte pinQB_Q = 7;

// motors
const byte pinAen = 11;
const byte pinAphase = 8;
const byte pinBen = 10;
const byte pinBphase = 12;

double LeftPIDInput, LeftPIDOutput, LeftPIDSetpoint;
double RightPIDInput, RightPIDOutput, RightPIDSetpoint;
PID leftPID(&LeftPIDInput, &LeftPIDOutput, &LeftPIDSetpoint, 5, 1, 0, DIRECT);
PID rightPID(&RightPIDInput, &RightPIDOutput, &RightPIDSetpoint, 5, 1, 0, DIRECT);

volatile byte x, px, y, py;
volatile byte x2, px2, y2, py2;
volatile int LeftEnc = 0;
volatile int RightEnc = 0;
unsigned long msgTime = 0;

unsigned long speedSampleTime = 0;  // keeping track of when to do speed calculation
const unsigned long speedSamplePeriod = 200;  // in milliseconds

int prevLeftEnc = 0, prevRightEnc = 0;
int LeftSpeed, RightSpeed;

void setup() {
  pinMode(pinQA_I, INPUT );
  pinMode(pinQA_Q, INPUT );
  pinMode(pinQB_I, INPUT );
  pinMode(pinQB_Q, INPUT );
  pinMode(13, OUTPUT);
  x = digitalRead(pinQA_I);  // read these to get the initial states for the ISRs
  y = digitalRead(pinQA_Q);
  x2 = digitalRead(pinQB_I);
  y2 = digitalRead(pinQB_Q);
  
  PCintPort::attachInterrupt(pinQA_I, &ISR_A_I, CHANGE);
  PCintPort::attachInterrupt(pinQA_Q, &ISR_A_Q, CHANGE);
  PCintPort::attachInterrupt(pinQB_I, &ISR_B_I, CHANGE);
  PCintPort::attachInterrupt(pinQB_Q, &ISR_B_Q, CHANGE);
  
  
  pinMode(pinAphase, OUTPUT);
  pinMode(pinBphase, OUTPUT);
  pinMode(pinAen, OUTPUT);
  pinMode(pinBen, OUTPUT);
  
  //attachInterrupt(0, left_encoder_ISR, CHANGE);  // we test the value of the pin in the ISR
  //attachInterrupt(1, right_encoder_ISR, CHANGE);  // ditto
  
  LeftPIDSetpoint = 0;
  RightPIDSetpoint = 0;
  leftPID.SetOutputLimits(-255, 255);
  rightPID.SetOutputLimits(-255, 255);
  leftPID.SetSampleTime(speedSamplePeriod);
  rightPID.SetSampleTime(speedSamplePeriod);
  leftPID.SetMode(AUTOMATIC);  // turn PID on
  rightPID.SetMode(AUTOMATIC);  
  
  Serial.begin( 9600 );
}

void loop()
{
  if( millis() - speedSampleTime > speedSamplePeriod ) // every so often
  {
    speedSampleTime = millis();
    
    // first calculate speed
    LeftSpeed = LeftEnc - prevLeftEnc;
    prevLeftEnc = LeftEnc;  // you could lose counts here
    LeftPIDInput = LeftSpeed;
    
    // Right motor
    RightSpeed = RightEnc - prevRightEnc;
    prevRightEnc = RightEnc;  // you could lose counts here
    RightPIDInput = RightSpeed;
  }
  
  // this can be called just as often as we like
  leftPID.Compute();
  if( LeftPIDSetpoint == 0)
    SetLeftMotorSpeed( 0 );
  else
    SetLeftMotorSpeed( LeftPIDOutput );
  rightPID.Compute();
  if( RightPIDSetpoint == 0)
    SetRightMotorSpeed( 0 );
  else
    SetRightMotorSpeed( RightPIDOutput );


  if( millis() - msgTime > 250 )
  {
    msgTime = millis();
    Serial.print( "LE "); Serial.print( LeftEnc); Serial.print( " " );
    Serial.print( "RE " ); Serial.print( RightEnc ); Serial.print( " " );
    Serial.print( "LS " ); Serial.print( LeftSpeed ); Serial.print( " " );
    Serial.print( "RS " ); Serial.print( RightSpeed ); Serial.print( " " );
    Serial.print( "LSo " ); Serial.print( LeftPIDOutput ); Serial.print( " " );
    Serial.print( "RSo " ); Serial.print( RightPIDOutput ); Serial.print( " " );
    Serial.print( "LKp " ); Serial.print( leftPID.GetKp() ); Serial.print( " " );
    Serial.print( "LKi " ); Serial.print( leftPID.GetKi() ); Serial.print( " " );
    Serial.print( "LKd " ); Serial.print( leftPID.GetKd() ); Serial.print( " " );

    Serial.print( "Di0 " ); Serial.print( ReadSharpSensor(A0) ); Serial.print( " " );
    Serial.print( "Di1 " ); Serial.print( ReadSharpSensor(A1) ); Serial.print( " " );
    Serial.print( "Di2 " ); Serial.print( ReadSharpSensor(A2) ); Serial.print( " " );
    
    Serial.print( "\r\n");
  }
  
  if( Serial.available() > 0 )
  {
    byte c = Serial.read();
    if( c == 'L') // set the left motor speed
    {
      LeftPIDSetpoint = Serial.parseFloat();
    } else
    if( c == 'R') // set the right motor speed
    {
      RightPIDSetpoint = Serial.parseFloat();
    } else
    if( c == 'M') // set both motor speeds
    {
      LeftPIDSetpoint = Serial.parseFloat();
      RightPIDSetpoint = Serial.parseFloat();
    } else
    if( c == 'Z') // reset the encoders to zero
    {
      LeftEnc = prevLeftEnc = 0;
      RightEnc = prevRightEnc = 0;
    } else
    if( c == 'S') // stop the robot immediately
    {
      LeftPIDSetpoint = 0;
      RightPIDSetpoint = 0;
      //analogWrite( pinAen, 0); // this only happens for a single loop pass so its ineffective!
      //analogWrite( pinBen, 0);
    } else
    if( c == 'K') // set the PID constants for both left and right speed controllers
    {
      leftPID.SetMode(MANUAL);
      rightPID.SetMode(MANUAL);
      double P = Serial.parseFloat();
      double I = Serial.parseFloat();
      double D = Serial.parseFloat();
      leftPID.SetTunings( P, I, D);
      rightPID.SetTunings( P, I, D);
      leftPID.SetMode(AUTOMATIC);
      rightPID.SetMode(AUTOMATIC);
    }
    
    // if its not one of the above commands, just swallow the bytes
    c = Serial.read();  // as long as there are bytes available that aren't satisfying the above, we boil them off here
    
  }
  

}

// left encoder interrupt
void ISR_A_I()
{ 
  px = x;
  x = digitalRead(pinQA_I);
  //y = digitalRead(pinQA_Q);
  
  digitalWrite( 13, y);  // debugging LED
  
  if( px == LOW && x == HIGH ) { 
    // x high transition
    if( y ) LeftEnc ++;
    else LeftEnc --;
  }
  
  if( px == HIGH && x == LOW ) {
    // x low transition
    if( y ) LeftEnc --;
    else LeftEnc ++;
  }
}

void ISR_A_Q()
{
  //x = digitalRead(pinQA_I);
  py = y;
  y = digitalRead(pinQA_Q);
  
  if( py == LOW && y == HIGH ) {
    // y high transition
    if( x ) LeftEnc --;
    else LeftEnc ++;
  }
  
  if( py == HIGH && y == LOW ) {
    // y low transision
    if( x ) LeftEnc ++;
    else LeftEnc --;
  }
}

// right encoder interrupt
void ISR_B_I()
{
  px2 = x2;
  x2 = digitalRead(pinQB_I);
  //y2 = digitalRead(pinQB_Q);
  
  if( px2 == LOW && x2 == HIGH ) { 
    // x high transition
    if( y2 ) RightEnc ++;
    else RightEnc --;
  }
  
  if( px2 == HIGH && x2 == LOW ) {
    // x low transition
    if( y2 ) RightEnc --;
    else RightEnc ++;
  }
}

void ISR_B_Q()
{
  //x2 = digitalRead(pinQB_I);
  py2 = y2;
  y2 = digitalRead(pinQB_Q);
  
  if( py2 == LOW && y2 == HIGH ) {
    // y high transition
    if( x2 ) RightEnc --;
    else RightEnc ++;
  }
  
  if( py2 == HIGH && y2 == LOW ) {
    // y low transision
    if( x2 ) RightEnc ++;
    else RightEnc --;
  }
}

// command a motor speed with a signed integer 
void SetLeftMotorSpeed( int spd)
{  
  if( spd == 0)
  {
    analogWrite(pinAen, 0);
  } else if( spd < 0 )
  {
    digitalWrite(pinAphase, LOW);
    analogWrite(pinAen, (-spd) );
  } else // speed > 0
  {
    digitalWrite(pinAphase, HIGH);
    analogWrite(pinAen, spd );
  }
}
  
// command a motor speed with a signed integer 
void SetRightMotorSpeed( int spd)
{
  //spd = constrain( spd, -255, 255);
    
  if( spd == 0)
  {
    analogWrite(pinBen, 0);
  } else if( spd < 0 )
  {
    digitalWrite(pinBphase, LOW);
    analogWrite(pinBen, (-spd) );
  } else // speed > 0
  {
    digitalWrite(pinBphase, HIGH);
    analogWrite(pinBen, spd );
  }
}

// measure the voltage of a sharp GP2Y0A21 IR distance sensor and return the distance in mm
float ReadSharpSensor( int pin)
{
  int raw = analogRead(pin);
  if( raw > 600)
    raw = 600;
  float calc = 270.0 / (float) raw * 1024.0 / 5.0;
  if( calc < 100.0 )
    return 100.0;
  else if( calc > 600.0 )
    return 600.0;
  else
    return calc;
}
