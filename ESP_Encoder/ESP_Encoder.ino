#include <math.h>
#define outputA 2
#define outputB 4

 int counter = 0; 
 int aState;
 int aLastState;  

 String heading;
 float head_convert;
 float revolution;
 float change=0;
 float prev_distance=0;
 float error_x=0;
 float error_y=0;
 float dx=0;
 float dy=0;
 float x=0;
 float y=0;

 void setup() { 
   pinMode (outputA,INPUT);
   pinMode (outputB,INPUT);
   
   Serial.begin (115200);
   // Reads the initial state of the outputA
   aLastState = digitalRead(outputA);   

  //  attachInterrupt(digitalPinToInterrupt(outputA), encoder_readISR, CHANGE);
  //  attachInterrupt(digitalPinToInterrupt(outputB), encoder_readISR, CHANGE);
 }  

 void encoder_read() 
 { 
   aState = digitalRead(outputA); // Reads the "current" state of the outputA
   // If the previous and the current  state of the outputA are different, that means a Pulse has occured
   if (aState != aLastState){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(outputB) != aState) { 
       counter ++;
     } else {
       counter --;
     }
   } 
   aLastState = aState; // Updates the previous state of the outputA with the current state
   Serial.println(counter);
 }

void get_position(float heading)
{
  revolution = 1.1*counter/1040;
  float distance_cm = revolution*(4*PI);
  change     = distance_cm - prev_distance;

  dx = cos(heading)*change;
  dy = sin(heading)*change;

  error_x += dx - cos(heading-error_x)*change;
  error_y += dy - sin(heading-error_y)*change;

  x += dx;
  y += dy;
  
  prev_distance = distance_cm;

  Serial.print(x);
  Serial.print(" ");
  Serial.println(y);
}

 void loop()
 {
    if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    if(data == "1"){
      counter = 0;
    }
  }
     encoder_read();
  // if (Serial.available()>0)
  // {
  //   heading = Serial.readStringUntil('\n');
     
  //   // Serial.println(heading);
  //   head_convert = heading.toFloat();
  //   get_position(head_convert);
  // }
 }
