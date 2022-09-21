#include <SoftwareSerial.h> 
#include <NewPing.h> //ultrasonic library
#include <Servo.h> //servo library
#define RightmotorF 7  //digitalpin 7 for right motor forward
#define RightmotorB 8  //digital pin 8 for right motor Backward
#define LeftmotorF 5 //digitalpin 5 for left motor forward
#define LeftmotorB 6 //digital pin 6 for right motor Backward
Servo myservo;  
int LeftDistance=0;
int RightDistance=0;
int distance=0;
int val;
String message="";

int FrontDistance=0;
NewPing sonar(2, 3, 400); //(trig,echo,maxdistance)
SoftwareSerial BT(10,11);  //Assigning arduino's (RXD,TXD)
void setup()
{
  // Setup LED

  pinMode(RightmotorF, OUTPUT);//declaring these pins as output to control them
    pinMode(RightmotorB, OUTPUT);
    pinMode(LeftmotorF,OUTPUT);
    pinMode(LeftmotorB,OUTPUT);

  Serial.begin(9600);
 myservo.attach(12);//telling the code that servo is at digital pin 12
  
  BT.begin(115200);
  
  BT.print("$$$");//Bluetooth stuff dont change
 
  delay(100);
  
  BT.println("U,9600,N");
 
  BT.begin(9600);
   serv(512);//setting the servo at initial position (Change this accordingly)
}

void loop()
{
 
 
  
  if(BT.available())
  {
 
  //Through comparing characters we will communicate, you can change this in app too
    char GetBT = (char)BT.read();
 
  
       if(GetBT=='d'){digitalWrite(RightmotorF, HIGH);//for forward movement both R  & L forwards are high and backward low
        digitalWrite(RightmotorB,  LOW); digitalWrite(LeftmotorF,HIGH); digitalWrite(LeftmotorB,LOW); } 
   else if(GetBT=='s'){ digitalWrite(RightmotorF, LOW);
        digitalWrite(RightmotorB, LOW); digitalWrite(LeftmotorF,LOW); digitalWrite(LeftmotorB,LOW);} 
  else if(GetBT=='a'){digitalWrite(RightmotorF, LOW);
        digitalWrite(RightmotorB, HIGH); digitalWrite(LeftmotorF,LOW); digitalWrite(LeftmotorB,HIGH); }
  else if(GetBT=='g'){ digitalWrite(RightmotorF, LOW);
       digitalWrite(RightmotorB, HIGH); digitalWrite(LeftmotorF,HIGH); digitalWrite(LeftmotorB,LOW);}
   else if(GetBT=='f'){ digitalWrite(RightmotorF, HIGH);
     digitalWrite(RightmotorB, LOW); digitalWrite(LeftmotorF,LOW); digitalWrite(LeftmotorB,HIGH);
   } 
  
     
  if(GetBT=='o')
   {
     while(1)
    {
      
     
      scan();//checking the distance
   
      FrontDistance=distance;
      
     // if(FrontDistance>20|| FrontDistance==0)
        if(FrontDistance>20)
      {
      
       Forward(); 
      }
      else
       {
        movestop();
        navigate();
       
         
         
      }
        
    
    if(BT.available())
    {
          char newBT = (char)BT.read();
        
        if(newBT=='t'){movestop();break;}
    }
     }
   
   
  
   }
   else if(GetBT=='k')
   {
     scan();
   //  Serial.println(distance);
     BT.print(distance);
   }
   
   
    
   }
    
   
 
    

  

 

   
    
    
  
  
  if(Serial.available())
  {
 //this part can be used to test through serial monitor for the motors
    char al=(char)Serial.read();
      
       //  int uS = sonar.ping();
    //   Serial.print("Ping: ");
    //   Serial.print(uS / US_ROUNDTRIP_CM);
     //  Serial.println("cm");
     if(al=='d'){ digitalWrite(RightmotorF, HIGH);
        digitalWrite(RightmotorB,  LOW); digitalWrite(LeftmotorF,HIGH); digitalWrite(LeftmotorB,LOW);} 
   else if(al=='s'){digitalWrite(RightmotorF, LOW);
        digitalWrite(RightmotorB, LOW); digitalWrite(LeftmotorF,LOW); digitalWrite(LeftmotorB,LOW);} 
  else if(al=='a'){digitalWrite(RightmotorF, LOW);
        digitalWrite(RightmotorB, HIGH); digitalWrite(LeftmotorF,LOW); digitalWrite(LeftmotorB,HIGH);  }
  else if(al=='g'){ digitalWrite(RightmotorF, LOW);
       digitalWrite(RightmotorB, HIGH); digitalWrite(LeftmotorF,HIGH); digitalWrite(LeftmotorB,LOW);}
   else if(al=='f'){ digitalWrite(RightmotorF, HIGH);
     digitalWrite(RightmotorB, LOW); digitalWrite(LeftmotorF,LOW); digitalWrite(LeftmotorB,HIGH);
      
   
   }
   
    
//    Serial.print(al);
   
   
   
  
    
  }
 
 
}
void serv(int a)
{
    val=map(a,0,1023,0,179);
  myservo.write(val);
  delay(1000);
}
void Forward()
{
  digitalWrite(RightmotorF, LOW);
        digitalWrite(RightmotorB, HIGH); digitalWrite(LeftmotorF,LOW); digitalWrite(LeftmotorB,HIGH);
}
void Backward()
{
 
        digitalWrite(RightmotorF, HIGH);
        digitalWrite(RightmotorB,  LOW); digitalWrite(LeftmotorF,HIGH); digitalWrite(LeftmotorB,LOW);
}
void Right()
{
  digitalWrite(RightmotorF, HIGH);
     digitalWrite(RightmotorB, LOW); digitalWrite(LeftmotorF,LOW); digitalWrite(LeftmotorB,HIGH);
}
void Left()
{
  
     digitalWrite(RightmotorF, LOW);
       digitalWrite(RightmotorB, HIGH); digitalWrite(LeftmotorF,HIGH); digitalWrite(LeftmotorB,LOW);
}  
void movestop()
{
  digitalWrite(RightmotorF, LOW);
        digitalWrite(RightmotorB, LOW); digitalWrite(LeftmotorF,LOW); digitalWrite(LeftmotorB,LOW);
   
}
void scan()
{
 int uS = sonar.ping();
      
  distance=(uS / US_ROUNDTRIP_CM); 
 delay(500);
}

  void navigate()
{
   
    serv(1023);                                //Move the servo to the left (my little servos didn't like going to 180 so I played around with the value until it worked nicely)
                                          //Wait half a second for the servo to get there
    scan();                                           //Go to the scan function
     LeftDistance = distance;                          //Set the variable LeftDistance to the distance on the left
    
 
   serv(10);                                  //Move the servo to the right
                                        //Wait half a second for the servo to get there
    scan();                                           //Go to the scan function
    RightDistance = distance;                         //Set the variable RightDistance to the distance on the right
    
    
    if(abs(RightDistance - LeftDistance) < 5)
    {
      Backward();                                  //Go to the moveBackward function
      delay(200);                                      //Pause the program for 200 milliseconds to let the robot reverse
     Right();                                     //Go to the moveRight function
      delay(100);       //Pause the program for 200 milliseconds to let the robot turn right
   serv(512);
  }
    else if(RightDistance < LeftDistance)                  //If the distance on the right is less than that on the left then...
    {
    Left();                                      //Go to the moveLeft function
     delay(100);       //Pause the program for half a second to let the robot turn
   serv(512);
  }
    else if(LeftDistance < RightDistance)             //Else if the distance on the left is less than that on the right then...
    {
     Right();                                     //Go to the moveRight function
     delay(100);                                      //Pause the program for half a second to let the robot turn
   serv(512);
  }
}
