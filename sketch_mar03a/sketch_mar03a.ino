const int trigger1 = 8; //Trigger pin of 1st Sesnor
const int echo1 = 12; //Echo pin of 1st Sesnor
const int trigger2 = 7; //Trigger pin of 2nd Sesnor
const int echo2 = 11;//Echo pin of 2nd Sesnor
const int IRSensorR = 2;
const int IRSensorL = 3; 
int led1 = 4;
int led2 = 5;
int led3 = 6;
int led4 = 9;
 
 int distL,distR,statusSensorL, statusSensorR;

#include <NewPing.h>

#define SONAR_NUM 2     // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(8, 12, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(7, 11, MAX_DISTANCE)
};

void setup() {
  Serial.begin(9600);  
  pinMode(trigger1, OUTPUT); 
  pinMode(echo1, INPUT); 
  pinMode(trigger2, OUTPUT); 
  pinMode(echo2, INPUT); 
  pinMode(IRSensorR, INPUT);
  pinMode(IRSensorL, INPUT); 
  // put your setup code here, to run once:
 //initialize digital pin as output
 pinMode(led1, OUTPUT);
 pinMode(led2, OUTPUT);
 pinMode(led3, OUTPUT);
 pinMode(led4, OUTPUT);
} 

void time_calc_play(int dis1, int dis2,int i,int j)
{
    int t=0;
    while(true)
    {
      dis1=sonar[i].ping_cm();
      dis2=sonar[j].ping_cm();
    
      if((dis1>2 && dis1<8) || (dis2>2 && dis2<8))
      {
        delay(100);
        Serial.println ("Play/Pause"); 
        Blink();
      }
      break;
    }
}

void time_calc_vol(int dis1, int dis2, int i, int j)
{
    if((dis1>8 && dis1<25) || (dis2>8 && dis2<25))
    {  
          
      dis2 =sonar[j].ping_cm();
      while(dis2>8 && dis2<40)
      { 
        Serial.println ("Volume Increased");
        Blink();
        break;
      } 
      delay(100);
      
      dis1 =sonar[i].ping_cm();     
      while (dis1>8 && dis1<40) //Hand pushed in 
      {
         Serial.println ("Volume Decreased");
         Blink();
         break;
      }
    }
}

void Forward(int x, int y,  char a)
{
  if(a=='y')
  {
    int t = 0;
    while(true)
    {
      int x  = digitalRead (IRSensorR);
      
      if( x == 0 )
       {
        int y  = digitalRead (IRSensorL);
         if((x == 0) && (y == 0))
           break;
           
        Serial.println("Forward 3sec");
        ForwardLED();
        delay(500);
        break;
       }
       else
       {
          t++;
          Serial.print("F-");
          Serial.println(t);
          if(t==200)
          {
           time_calc_play(distL,distR,0,1);
           break;
          }
       }
    }
  }
}

void Rewind(int x,int y,char a)
{
  if(a=='y')
  {
    int t = 0;
    while(true)
    {
      int y  = digitalRead (IRSensorL);
      if( y == 0 )
       {
        int x  = digitalRead (IRSensorR);
        if((x == 0) && (y == 0))
           break;
           
        Serial.println("Rewind 3sec");
        RewindLED();
        delay(500);
        break;
       }
       else
       {
          t++;
          Serial.print("B-");
          Serial.println(t);
          if(t==200)
          {
           time_calc_play(distL,distR,0,1);
           break;
          }
       }
       
    }
  }
}

void ForwardLED()
{
  digitalWrite(led4, HIGH);
 delay(100);
  digitalWrite(led3, HIGH);
 delay(100);
  digitalWrite(led2, HIGH);
 delay(100);
  digitalWrite(led1, HIGH);
 delay(100);

 digitalWrite(led4, LOW);
 delay(100);
 digitalWrite(led3, LOW);
 delay(100);
 digitalWrite(led2, LOW);
 delay(100);
 digitalWrite(led1, LOW);
 delay(100);
}

void RewindLED()
{
  digitalWrite(led1, HIGH);//it mean to give 5v(high) to pins.here ,the led will be on.
 delay(100);//1000 = 1 second
 digitalWrite(led2, HIGH);
 delay(100);
 digitalWrite(led3, HIGH);
 delay(100);
  digitalWrite(led4, HIGH);
 delay(100);

 digitalWrite(led1, LOW);//it mean to give 0v(low) to pin.here, led will be off
 delay(100);
 digitalWrite(led2, LOW);
 delay(100);
 digitalWrite(led3, LOW);
 delay(100);
 digitalWrite(led4, LOW);
 delay(100);
}

void Blink()
{
  digitalWrite(led1, HIGH);//it mean to give 5v(high) to pins.here ,the led will be on.
  digitalWrite(led2, HIGH);
  digitalWrite(led3, HIGH);
  digitalWrite(led4, HIGH);
 
  delay(200);

 digitalWrite(led1, LOW);//it mean to give 0v(low) to pin.here, led will be off
 digitalWrite(led2, LOW);
 digitalWrite(led3, LOW);
 digitalWrite(led4, LOW);
}
 
void loop() { //infinite loopy
  
  distL =sonar[0].ping_cm(); //get distance of left sensor
  distR =sonar[1].ping_cm(); //get distance of right sensor
  statusSensorR  = digitalRead (IRSensorR);
  statusSensorL  = digitalRead (IRSensorL);

    //Control Modes
    //VolUp/Down - Control Mode
    time_calc_vol(distL,distR,0,1);
 
  if(statusSensorL != 0)
  {
  //Forward-Rewind - Control Mode
    Forward(statusSensorR,statusSensorL,'y');
  }
  if(statusSensorR != 0)
  {
    Rewind(statusSensorR,statusSensorL,'y');
  }
    
  delay(200);
   // put your main code here, to run repeatedly
}
