#include "max6675.h"


//Pins
int ktcSO = 8;
int ktcCS = 9;
int ktcCLK = 10;
int heater = 7;
int pwr = 13;
//Variables
int target;
int counter=0;

//Object
MAX6675 ktc(ktcCLK, ktcCS, ktcSO);


class PID
{
  public:
  
  float calculate(int);
  PID();
  void reset();
  
  private: 
  int slowNow = 25;//degrees away from target
  float kp=1/slowNow; float ki= 0.005; float kd= 0.01;
  float errorThresh=10; 
  int error;
  double errorTotal=0;
  int lastError=0;
  int dError;
};

PID::PID(){ }

void PID::reset() {
  errorTotal= 0; 
  lastError= 0; 
}

float PID::calculate(int target=0)
{
  error = target - ktc.readCelsius();
  if(abs(error)>errorThresh){ errorTotal+=error;}
  else { errorTotal=0;}
  dError = error - lastError;
  
  if(kp*error + ki*errorTotal + kd*dError > 1){return 1;}
  else if(kp*error + ki*errorTotal + kd*dError < 0){return 0;}
  else {return (kp*error + ki*errorTotal + kd*dError);}
}

//Objects
PID pid;

//SETUP//
void setup() {
  //Power
  pinMode(pwr,OUTPUT); 
  pinMode(heater, OUTPUT);
  digitalWrite(pwr,HIGH);
  
  Serial.begin(9600);
  target = 680;  // 660 is melting point of aluminum
  // give the MAX a little time to settle
  delay(500);
}

////////
//LOOP//
////////
void loop() {
//PID Loop
 counter = (int)(100 *pid.calculate(target));

 for(int i=0; i<100; i++)
 {
 if(counter>0){digitalWrite(heater,HIGH);}
 else{digitalWrite(heater,LOW);}
  counter--;
  delay(5);//cycles every half second
 }

//Temp on Serial Monitor
  Serial.print("Deg C = ");
  Serial.println(ktc.readCelsius());

}
