#include <MD_MAX72xx.h>
#include <SPI.h>

constexpr uint8_t MotorPWM = 3; //sends PWM signals to motor
unsigned long Duration = 0; //used in distance calculation, denotes duration of sound from distance and back in (unknown unit of seconds)
float Distance = 0; //distance from the sensor the the object being detected
constexpr uint8_t SonicPinPulseInput = 5; //sent to sonic sensor in 10us bursts
constexpr uint8_t SonicPinEcho = 6; //reads the duration of the signal from the sensor
uint8_t Angle = 0; //angle of the servo
unsigned int degreeSig = map(Angle, 0, 180, 500, 2500); ; //maps a degree of range 1-180 onto a range 600-2400 to add to the timing for the motor, somehow
#include <math.h>
bool radarMap[8][8];  // stores all currently detected points


#define HARDWARE_TYPE MD_MAX72XX::FC16_HW //tells the code what display im using



#define MAX_DEVICES 1 //i only got one display

#define DATA_PIN 7 //data pin for the ddisplay
#define CS_PIN   8 //tells the display to listen
#define CLK_PIN  9  //tells the display to move on

#define DEG_TO_RAD 0.0174532925 //conversion factor for turning degrees into radians
#define MAX_DISTANCE_MM 60 //0.5m
MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN);
void setup() {

Serial.begin(9600);
pinMode(SonicPinPulseInput, OUTPUT);
pinMode(SonicPinEcho, INPUT);
pinMode(MotorPWM, OUTPUT);
 mx.begin();
mx.control(MD_MAX72XX::INTENSITY, 5);
mx.clear();
} 

void loop() {
while (Angle <=179) {
delay(20); //delay for motor PWM period
digitalWrite(MotorPWM, HIGH);
delayMicroseconds(degreeSig);  //send pulse between 600 and 2400 us to servo to adjust angle
digitalWrite(MotorPWM, LOW);
digitalWrite(SonicPinPulseInput, HIGH); //beginning of sensor pulse
delayMicroseconds(10); //10 us pulse
digitalWrite(SonicPinPulseInput, LOW); //end sensor pulse
Duration = pulseIn(SonicPinEcho, HIGH, 38000); //Datasheet claims this measures the time of return pulse in microseconds, no clue how


if (Duration >= 38000){ //this value is taken from the sensors datasheet, no clue where they pulled this out of but its presumably 2 meters
Serial.println("range greater than 2m");
Angle += 1;
degreeSig = map(Angle, 0, 180, 600, 2400);

//For plotter mode: Serial.println(0);
//zero is used as error value as it should never appear under any circumstances in normal operation
}

else {
Distance = Duration*0.01721; //distance calculation, see the datasheet for this sensor to understand the math behind it (this assumes temp=20c which is a fine estimate considering the sensor isnt very accurate anyway)
//Serial.println("Range is: ");
Serial.println(Distance);
//Serial.println("cm");
// For plotter mode: Serial.println(Distance);
Angle += 1;
degreeSig = map(Angle, 0, 180, 600, 2400);
updateDisplay(Angle, Distance);
}
}
if (Angle == 180) {
  Angle = 0;
  degreeSig = map(Angle, 0, 180, 600, 2400);

}
}
void updateDisplay(int angle, int distance) {
    const int originX = 3;
    const int originY = 7;
    const int maxRadius = 7;
    float angleRad = angle * DEG_TO_RAD;

    mx.clear(); //clears

    // --- Draw radar line ---
    for (int r = 0; r <= maxRadius; r++) {
        int x = originX + round(r * cos(angleRad));
        int y = originY - round(r * sin(angleRad));
        x = constrain(x, 0, 7);
        y = constrain(y, 0, 7);

        mx.setPoint(y, x, true); // radar line pixel
        radarMap[y][x] = false;  // radar line clears any lingering object here
    }

    // --- Add new object dot for this angle ---
    int rObj = map(distance, 0, MAX_DISTANCE_MM, 1, maxRadius); // avoid 0
    int px = originX + round(rObj * cos(angleRad));
    int py = originY - round(rObj * sin(angleRad));
    px = constrain(px, 0, 7);
    py = constrain(py, 0, 7);

    // Store in buffer
    radarMap[py][px] = true;

    // --- Draw all lingering points ---
    for (int y = 0; y < 8; y++) {
        for (int x = 0; x < 8; x++) {
            if (radarMap[y][x]) {
                mx.setPoint(y, x, true);
            }
        }
    }
}

//Sonic connections as follows: Pin1 to 5v, pin2 to 5, pin3 to 6, pin4 to ground
//Motor connections as follows: Pin1 to 5v, pin2 to 4, pin3 to ground
//from motor datasheet: Position "0" (1.5 ms pulse) is middle, "90" (~2ms pulse) is all the way to the right, "-90" (~1ms pulse) is all the way to the left.