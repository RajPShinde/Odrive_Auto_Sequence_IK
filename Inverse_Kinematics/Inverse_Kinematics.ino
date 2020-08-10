
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// ODrive object
ODriveArduino odrive(Serial1);

double link1 = 20.5;
double link2 = 20;
const int degree = 270;
double initA = 0;
double initB = 0;
char c = "n";

void setup() {
  // ODrive uses 115200 baud
  Serial1.begin(115200);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open

  Serial.println("ODriveArduino");
  Serial.println(" ");
  Serial.println("Initialize Joints Manually");
  delay(10000);
  Serial.println("Press any Key to Continue");
  while (!Serial.available()) {
  }

  // Run calibration sequence
  int requested_state;
      
//  requested_state = ODriveArduino::AXIS_STATE_ENCODER_INDEX_SEARCH;
//  Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
//  odrive.run_state(0, requested_state, true);

  requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
  odrive.run_state(0, requested_state, false); // don't wait

//  requested_state = ODriveArduino::AXIS_STATE_ENCODER_INDEX_SEARCH;
//  Serial << "Axis" << 1 << ": Requesting state " << requested_state << '\n';
//  odrive.run_state(1, requested_state, true);

  requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  Serial << "Axis" << 1 << ": Requesting state " << requested_state << '\n';
  odrive.run_state(1, requested_state, false); // don't wait

  Serial1 << "r axis0.encoder.pos_estimate\n";
  Serial << odrive.readFloat() << '\t';
  initA = odrive.readFloat();
  Serial1 << "r axis1.encoder.pos_estimate\n";
  Serial << odrive.readFloat() << '\t';
  initB = odrive.readFloat();

  Serial.println("Ready!");
  Serial.println("Send the character '0' or '1' to run closed loop control)");
  Serial.println("Send the character 's' to exectue test move");
  Serial.println("Send the character 'b' to read bus voltage");
  Serial.println("Send the character 'p' to read motor positions in a 10s loop");
  Serial.println("Send the character 'w' to perform Gait");
  Serial.println("Send the character 'u' to perform Situps");
//  Serial1 << "t 0 "<<initA-(45-a)*270<<"\n";
}

void loop() {
//  if (Serial.available()) {
//    c = Serial.read();
//    Serial.println("Received Command");
//  }
//    c="?w";

        // Walk
//    if (c == 'w') {
//      walk();
sitUps();
//    }

    // Situps
    if (c == 'u') {
      sitUps();
    }
    
    //Do nothing
    else if (c == 'n') {
      
    }
    
    // Run calibration sequence
    else if (c == '0' || c == '1') {
      int motornum = c-'0';
      int requested_state;
      requested_state = ODriveArduino::AXIS_STATE_ENCODER_INDEX_SEARCH;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive.run_state(motornum, requested_state, true);

      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive.run_state(motornum, requested_state, false); // don't wait
    }

    // Sinusoidal test move
    else if (c == 's') {
      Serial.println("Executing test move");
      for (float ph = 0.0f; ph < 6.28318530718f; ph += 0.01f) {
        float pos_m0 = 20000.0f * cos(ph);
        float pos_m1 = 20000.0f * sin(ph);
        odrive.SetPosition(0, pos_m0);
        odrive.SetPosition(1, pos_m1);
        delay(5);
      }
    }

    // Read bus voltage
    else if (c == 'b') {
      Serial1 << "r vbus_voltage\n";
      Serial << "Vbus voltage: " << odrive.readFloat() << '\n';
    }

    // print motor positions in a 10s loop
    else if (c == 'p') {
      static const unsigned long duration = 10000;
      unsigned long start = millis();
      while(millis() - start < duration) {
        for (int motor = 0; motor < 2; ++motor) {
          Serial1 << "r axis" << motor << ".encoder.pos_estimate\n";
          Serial << odrive.readFloat() << '\t';
        }
        Serial << '\n';
      }
    }
}

double zangle(double x, double y, double z)
{
  //theta 1
  return (57.3*atan(z/(28-y)));
}

double aangle(double x, double y, double z)
{
  //theta 2
  double r=sqrt(x*x+(y-28)*(y-28)+z*z);
  return (57.3*(acos(((link1*link1)+(r*r)-(link2*link2))/(2*link1*r)) -(atan(x/(28-y)))));
}

double bangle(double x, double y, double z)
{
  //theta 3
  double r=sqrt(x*x+(y-28)*(y-28)+z*z);
  return (57.3*acos(((link1*link1)+(link2*link2)-(r*r))/(2*link1*link2)));
}

void walk()
{
 for(double x=-9.0; x<=9.0; x=x+0.1)
  {
   double y=sqrt(120-(x*x))-5;
   setangle(zangle(x,y,0),aangle(x,y,0),bangle(x,y,0));
  
  }
 for(double x=9.0; x>=-9.0; x=x-0.1)
  {
    double y=sqrt(120-(x*x))-5;
   setangle(zangle(x,y,0),aangle(x,y,0),bangle(x,y,0));

  } 
}

void sitUps()
{
   odrive.SetPosition(1, initB+2000);
   delay(3000);
   odrive.SetPosition(1, initB-2000);
// for(double y=0; y<=7.0; y=y+0.01)
//  {
//   setangle(zangle(0,y,0),aangle(0,y,0),bangle(0,y,0));
//   delay(1);
  
//  }
// for(double y=7.0; y>=0; y=y-0.01)
//  {
//   setangle(zangle(0,y,0),aangle(0,y,0),bangle(0,y,0));
//  delay(1);
//  } 
}

void setangle(int z,int a, int b)
{
//  zs.writeMicroseconds(1500-(11*z));
//  as.writeMicroseconds(2200-(15*a));
//  bs.writeMicroseconds(1300+(15*(90-b)));
  odrive.SetPosition(0, initA-(45-a)*270);
//Serial1 << "t 0 "<<(45-a)*270<<"\n";
//Serial1 << "t 1 "<<initA-(45-a)*270<<"\n";
  odrive.SetPosition(1, initB+(90-b)*270);
//  Serial.println( initA-(45-a)*270);
//  Serial.println( initB+(90-b)*270);
//  odrive.SetPosition(1, initB+());
//  Serial.println(a);
//  Serial.println(b);
// odrive.SetPosition(0, initA+2000);
//  odrive.SetPosition(1, initB+2000);
}
