/*

MinIMU-9-Arduino-AHRS
Pololu MinIMU-9 + Arduino AHRS (Attitude and Heading Reference System)

Copyright (c) 2011 Pololu Corporation.
http://www.pololu.com/

MinIMU-9-Arduino-AHRS is based on sf9domahrs by Doug Weibel and Jose Julio:
http://code.google.com/p/sf9domahrs/

sf9domahrs is based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose
Julio and Doug Weibel:
http://code.google.com/p/ardu-imu/

MinIMU-9-Arduino-AHRS is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published by the
Free Software Foundation, either version 3 of the License, or (at your option)
any later version.

MinIMU-9-Arduino-AHRS is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
more details.

You should have received a copy of the GNU Lesser General Public License along
with MinIMU-9-Arduino-AHRS. If not, see <http://www.gnu.org/licenses/>.

*/

// set mode=ppm or mode=mouse
// todo - put a switch on the case that sets the mode
String mode="ppm";

/////////////////////////////////////////////////////////////////////
// PPM transmitter init 
// - uses ArduinoRCLib - http://sourceforge.net/p/arduinorclib/wiki/Home/
// - outputs on pin ppmpin (9 usually). Pin 9 goes to tip, ground goes to ground
// - send analog 1000 to 2000 for full range of PPM input into an RC transmitter
#include <PPMOut.h>
#include <Timer1.h>
#define CHANNELS 2
int ppmpin=9;
uint16_t g_input[CHANNELS];                   // Input buffer in microseconds
uint8_t  g_work[PPMOUT_WORK_SIZE(CHANNELS)];  // we need to have a work buffer for the PPMOut class
rc::PPMOut g_PPMOut(CHANNELS, g_input, g_work, CHANNELS);
/////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////
// init mouse
const int buttonPin=4;
int mousePressed=0;
int buttonState=0;
bool debug=false;
bool scrollOn=false;
unsigned long recenterTime;
unsigned long clickWaitTime;
float centerX=0;
float centerY=0;
float centerZ=0;
int sensX=80;
int sensY=80;
int readCount=0;
bool readyMove=false;
/////////////////////////////////////////////////////////////////////


// Uncomment the below line to use this axis definition: 
   // X axis pointing forward
   // Y axis pointing to the right 
   // and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// Uncomment the below line to use this axis definition: 
   // X axis pointing forward
   // Y axis pointing to the left 
   // and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
//int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

// tested with Arduino Uno with ATmega328 and Arduino Duemilanove with ATMega168

#include <Wire.h>

// LSM303 accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer 

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// L3G4200D gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303 magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 library to find the right values for your board
#define M_X_MIN -3049
#define M_Y_MIN -2980
#define M_Z_MIN -3782
#define M_X_MAX 3714
#define M_Y_MAX 3261
#define M_Z_MAX 1890

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data, 
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

#define STATUS_LED 13 

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values 
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0}; 
float errorYaw[3]= {0,0,0};

unsigned int counter=0;
byte gyro_sat=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
}; 
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here


float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};
 
void setup()
{ 
	///////////////////////////////////////////////////////////////
	// setup PPM transmitter
	rc::Timer1::init();
	for (uint8_t i = 0;  i < CHANNELS; ++i) {
		g_input[i] = 0;
	}
	g_PPMOut.setPulseLength(448);   // pulse length in microseconds
	g_PPMOut.setPauseLength(10448); // length of pause after last channel in microseconds
	g_PPMOut.start(ppmpin);
	///////////////////////////////////////////////////////////////



	///////////////////////////////////////////////////////////////
	// setup mouse
	pinMode(buttonPin, INPUT);
	if (mode=="mouse"){
		Mouse.begin();
	}
	///////////////////////////////////////////////////////////////



  if (debug) Serial.begin(9600);
  pinMode (STATUS_LED,OUTPUT);  // Status LED
  
  I2C_Init();

  if (debug) Serial.println("Pololu MinIMU-9 + Arduino AHRS");

  digitalWrite(STATUS_LED,LOW);
  delay(1500);
 
  Accel_Init();
  Compass_Init();
  Gyro_Init();
  
  delay(20);
  
  for(int i=0;i<32;i++)    // We take some readings...
    {
    Read_Gyro();
    Read_Accel();
    for(int y=0; y<6; y++)   // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
    }
    
  for(int y=0; y<6; y++)
    AN_OFFSET[y] = AN_OFFSET[y]/32;
    
  AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];
  
  //Serial.println("Offset:");
  for(int y=0; y<6; y++)
    if (debug) Serial.println(AN_OFFSET[y]);
  
  delay(2000);
  digitalWrite(STATUS_LED,HIGH);
    
  timer=millis();
  delay(20);
  counter=0;
}

void loop() //Main Loop
{
	if((millis()-timer)>=25)  // Main loop runs at 50Hz
	{
		counter++;
		timer_old = timer;
		timer=millis();
		if (timer>timer_old)
			G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
		else
			G_Dt = 0;
		
		// *** DCM algorithm
		// Data adquisition
		Read_Gyro();   // This read gyro data
		Read_Accel();     // Read I2C accelerometer
		
		if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
			{
			counter=0;
			Read_Compass();    // Read I2C magnetometer
			Compass_Heading(); // Calculate magnetic heading  
			}
		
		// Calculations...
		Matrix_update(); 
		Normalize();
		Drift_correction();
		Euler_angles();
		// ***

		//printdata();
		float thisYaw=ToDeg(yaw);
		float thisPitch=ToDeg(pitch);
		float thisRoll=ToDeg(roll);

		if (debug){
			Serial.print("yaw -> ");
			Serial.print(thisYaw);
			Serial.print("   ");
			Serial.print(yaw);
			Serial.print("roll -> ");
			Serial.print(thisRoll);
			Serial.print("   ");
			Serial.print(roll);
			Serial.print("      pitch -> ");
			Serial.print(thisPitch);
			Serial.print("   ");
			Serial.println(pitch);
		}

		if (readyMove && mousePressed==0){
			if (scrollOn){
				// in scroll mode, scroll up/down
				int mouseX=map(thisYaw-centerX, -90, 90, -sensX, sensX);
				int mouseY=map(thisPitch-centerY, -90, 90, 3, -3);
				Mouse.move(0, 0, mouseY);
			} else {
				// here is where the action happens - gyro->external device
				if (mode=="mouse"){
					int mouseX=map(thisYaw-centerX, -90, 90, -sensX, sensX);
					int mouseY=map(thisPitch-centerY, -90, 90, -sensY, sensY);
					Mouse.move(mouseX, mouseY, 0);
				}

				if (mode=="ppm"){
					int mouseX=map(thisYaw-centerX, -90, 90, 2000, 1000);
					int mouseY=map(thisPitch-centerY, -90, 90, 1000, 2000);
					g_input[0]=mouseX;
					g_input[1]=mouseY;
					g_PPMOut.update();
				}
			}

			if (mode=="mouse"){
				if ((thisRoll-centerZ)<-30 || (thisRoll-centerZ)>30){
					// something has gone horribly wrong, stop mousing for now
					readyMove=false;
				} else if ((thisRoll-centerZ)>17 && millis()>clickWaitTime){
					Mouse.click();
					clickWaitTime=millis()+2000;
				} else if ((thisRoll-centerZ)<-17 && millis()>clickWaitTime){
					// head tilt right, activate/deactivate scroll mode
					if (scrollOn==true){
						// scroll was on, turn it off
						scrollOn=false;
					} else {
						// turn on scroll mode
						scrollOn=true;
					}
					clickWaitTime=millis()+2000;
				}
			}
		} else {
			// skip the first x readings, give it time to center after program starts
			if (readCount>50){
				if ((thisRoll-centerZ)<-30 || (thisRoll-centerZ)>30){
					// just keep waiting until the roll makes sense
				} else {
					recenter(thisYaw, thisPitch, thisRoll);
					readyMove=true;
				}
			} else {
				readCount++;
			}
		}

		buttonState=digitalRead(buttonPin);
		if (mousePressed==0 && buttonState==HIGH){
			mousePressed=1;
			recenterTime=millis();
			if (debug) Serial.println("  Mouse Pressed");
		} else if (mousePressed==1 && buttonState==LOW){
			mousePressed=0;
			if (millis()>recenterTime+4000){
				// mouse was held for the recenter length of time, recenter as x degrees from 180
				recenter(thisYaw, thisPitch, thisRoll);
			} else {
				if (mode=="mouse"){
					// nothing special was attempted, just click the mouse
					Mouse.click();
				}
			}

			if (debug){
				Serial.print("  ");
				Serial.print((millis()-recenterTime)/1000);
				Serial.print("Mouse Released, ");
				Serial.print((millis()-recenterTime)/1000);
				Serial.println("s");
			}
		}
	}
}

void recenter(float thisYaw, float thisPitch, float thisRoll){
	centerX=thisYaw;
	centerY=thisPitch;
	centerZ=thisRoll;
	if (true) {
		Serial.print("recentering - x:y=");
		Serial.print(centerX);
		Serial.print(":");
		Serial.println(centerY);
	}
}
