/*
Modified code by Kiran Sreekumar, made with reference to a thread by shmadam 
on arduino forum. https://forum.arduino.cc/index.php?topic=557964.0

Made for AMT213D-V to work with Arduino Mega 2560. Code can be re-adjusted to work with Uno or nano
by choosing not to use serial monitor for outputs. Maybe use i2c to send data to say, a raspberry pi.
*/

#define RX        19                //For Arduino Mega Serial 1
#define TX        18

#define Re    3
#define De    5

#define Transmit    HIGH
#define Receive     LOW

uint8_t In1;                        //bytes read
uint8_t In2;
uint8_t In3;

uint16_t degb;                      // position data after combining
float deg;                          //position in degrees
int command;

command = 'T';                      //'T' requests position from the module. 
                                    //see datasheets for other possible commands.
int f,p=0;
float a,ap=-1;
int del=0;                          //change(in degrees) required to trigger output. Use if you want to display angle only when motor turns.

void setup() 
{
  Serial.begin(9600);
  while (!Serial) {
    ;
    }
  Serial.println("Serial ready for transmitting");
   
  pinMode(Re, OUTPUT);
  pinMode(De, OUTPUT);   
 
  RS485Receive(); 

  Serial1.begin(2000000);          //it works only at this speed. Don't use software serial.

}

void loop()   
{
    float a=GetAngle();
    if (abs(a-ap)>del)             //looking for 'del' change in angle
    {
      Serial.print("Angle = ");
      Serial.println(a);
      ap=a;
    }
}

void RS485Transmit()
{
  digitalWrite(Re, LOW);   
  digitalWrite(De, HIGH);   
}

void RS485Receive()
{
  digitalWrite(Re, HIGH);   
  digitalWrite(De, LOW); 
}

float GetAngle()
{ 
              
    RS485Transmit();  
    Serial1.write(command);        // Send command byte to encoder 
    delay(10);
    
    RS485Receive();
    delay(50);                     //increase delay in-case of unstable output   
 
  if (Serial1.available())         //first byte data read from encoder: Usually the command itself gets returned back. Remove this block if not.
   In1 = Serial1.read();    
    
  if (Serial1.available())       
   In2 = Serial1.read();           //Low-Byte data from encoder
   
  if (Serial1.available())       
   In3 = Serial1.read();           //High-Byte data from encoder
   
  In3=In3<<2;                      //removing the error bits
  In3=In3>>2;                      //adding zeroes instead of error bits
  degb=In3;                        //using a 16 bit variable to store the high byte
  degb=degb<<8;                    //making space for the low byte
  degb=degb|In2;                   //adding the low byte by using or
  
  //sensor returns 14 bit readings: 0 to 16383
  //we can convert this to degrees
  deg=degb*0.02197265625;          //360 --> 16383   quick-math
  
  return deg;                      //returning final angle
 }
 
