/* Go to the Serial Settings tab in the Maestro Control Center and apply these settings:

   Serial mode: UART, fixed baud rate
   Baud rate: 9600
   CRC disabled

*/

#include <PololuMaestro.h>
#include <SoftwareSerial.h>
#include <SPI.h>
//#include <Pixy2.h>
#include <Pixy2SPI_SS.h>

SoftwareSerial maestroSerial(8,9);
MicroMaestro maestro(maestroSerial);
//Pixy2 pixy;
Pixy2SPI_SS pixy;

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

float X, Y;       // X and Y coordinates of calibration markers
float r_platform; //max platform radius

float X_total = 0, Y_total = 0;  // the total summation of X and Y values
float r_platform_total = 0;           // the total summation of r_max values
int j = 0;

void setup() {
  // Set the baud rate for the Serial Monitor
  Serial.begin(115200);
  // Set the serial baud rate.
  maestroSerial.begin(9600);
  // Initialize pixy camera
  pixy.init();
  pixy.setLamp(1,1);

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
  for (int k = 0; k < 100; k++) { // runs 100 loops and gets average center location and r platform value
    // grab blocks!
    pixy.ccc.getBlocks();

    // If there are 4 markers detected, then collect and print the data
    if (pixy.ccc.numBlocks == 4) {
      //sets current X and Y co-ords of the center
      X = 0;
      Y = 0;
      for (int i = 0; i < 4; i++) {
        X = X + pixy.ccc.blocks[i].m_x;  // absolute X location of the center
        Y = Y + pixy.ccc.blocks[i].m_y;  // absolute Y location of the center
      }

      X = X / 4;
      Y = Y / 4;

      r_platform = 0;
      for (int i = 0; i < 4; i++) {
        // adds magnitude of vector pointing from the center of the platform to the center of the marker
        r_platform = r_platform + sqrt(pow(pixy.ccc.blocks[i].m_x - X, 2) + pow(pixy.ccc.blocks[i].m_y - Y, 2));  
        // adds magnitude of vector pointing from the center of the marker to the corner of the marker
        r_platform = r_platform + sqrt(pow(pixy.ccc.blocks[i].m_width, 2) + pow(pixy.ccc.blocks[i].m_height, 2))*0.5;
      }

      r_platform = r_platform / 4;  //calculates average r_max

      X_total = X_total + X;
      Y_total = Y_total + Y;
      r_platform_total = r_platform_total + r_platform;

      j++;

    }
    // If there are not 4 balls detected, then print so
    else {
      Serial.println((String) "Not 4 detected");
    }
  }

  X_total = X_total / j;
  Y_total = Y_total / j;
  r_platform_total = r_platform_total / j;

  Serial.println((String) "CENTER: [" + int(X_total) + ", " + int(Y_total) + "]");  //prints location of the center
  Serial.println((String) "r platform: " + int(r_platform_total));                  //prints magnitude of r

  while (1) {}
}
