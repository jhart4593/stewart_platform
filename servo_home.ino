/* Go to the Serial Settings tab in the Maestro Control Center and apply these settings:

   Serial mode: UART, fixed baud rate
   Baud rate: 9600
   CRC disabled

*/

#include <PololuMaestro.h>
#include <SoftwareSerial.h>

SoftwareSerial maestroSerial(8,9);
MicroMaestro maestro(maestroSerial);

//**CONSTANTS**
//-------------
float abs_0 = 4000;   //us (microsecond) position of absolute 0 degrees is 1000us, maestro uses units of 0.25us so 4x us position
float abs_90 = 6667;  //us (microsecond) position of absolute 90 degrees is 1667us, maestro uses units of 0.25us so 4x us position

//**USER DEFINED VALUES**
//----------------------

// angle range for each servo going CCW
// each servo can move from absolute 0 to absolute 270 degrees
// this defines a different range 
float range[6][2] = { { -35, 55 }, { 35, -55 }, // 1, 2
                      { -35, 55 }, { 35, -55 }, // 3, 4
                      { -35, 55 }, { 35, -55 }}; // 5, 6 

// angle offset value for each servo 
// redefines the position of the lower range of each servo
// Example: a servo with a range [0, 90] and an offset value of 5 would have its new 0 degree position where the old 5 degree position was

float offset[6] = {10.0, -10.0, // channel #0 and channel #1
                   10.0, -10.0, // channel #2 and channel #3
                   10.0, -10.0}; // channel #4 and channel #5

void setup() {
  // Set the baud rate for the Serial Monitor
  Serial.begin(115200);
  // Set the serial baud rate.
  maestroSerial.begin(9600);

  int pos;
  for (int idx = 0; idx < 6; idx++){
    maestro.setSpeed(idx, 0); //sets unlimited speed to servo idx
    maestro.setAcceleration(idx, 0); //sets unlimited acceleration to servo idx
    pos = map(offset[idx],range[idx][0],range[idx][1],abs_0,abs_90); //finds maestro servo position value for offset
    maestro.setTarget(idx, pos); //sets servo idx positions
    delay(1000);
  }

}

void loop() {
    
}
