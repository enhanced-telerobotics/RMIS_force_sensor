// Sketch to publish force or sensor data to serial to be read by pySerial in ROS
// or publish sensor value to ROS from Arduino using predefined calibration matrices

#define Test  //Print data to terminal
// #define CAL  //Calibration with C++ and Nano 17

#define FORCE  //Print calculated force data
// #define SENSOR  //Print raw sensor data

// #define LEFT
// #define RIGHT

#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>

#ifdef Test
  bool zeroed_flag = false;
  BLA::Matrix<7> SL = { 0, 0, 0, 0, 0, 0, 1 };
  BLA::Matrix<7> SR = { 0, 0, 0, 0, 0, 0, 1 };

  BLA::Matrix<7> SLinit = { 0, 0, 0, 0, 0, 0, 1};
  BLA::Matrix<7> SRinit = { 0, 0, 0, 0, 0, 0, 1};

  BLA::Matrix<3> FL = { 0, 0, 0 };
  BLA::Matrix<3> FR = { 0, 0, 0 };

  BLA::Matrix<3, 7> leftCalMat = {-1.86156908, -0.69974926, -0.83595665, 1.4406213, 0.9142526, 0.97022077, -0.06659354, 
                                  -0.27929164, -0.61079086, 0.25906565, 0.17555927, -0.40035626, 0.81424779, 0.00842615, 
                                  -3.73124721, -3.65761623, -3.06262283, 2.97177982, 4.49316102, 3.5187753, -0.12811767};

  BLA::Matrix<3, 7> rightCalMat = {-1.95464252, -1.07146008, -0.75347653, 1.82946937, 0.99155047, 0.746254, -0.04229878, 
                                   0.04004619, -0.59810355, 0.5931482, 0.08852037, -0.64212389, 0.48293662, 0.01472948, 
                                   -3.78002743, -3.8956064, -3.46143299, 3.32560038, 3.67704427, 3.29307808, 0.04997841};


  const int pinRead[] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11};
#endif

#ifdef CAL
  #define BAUD 256000
  #define delayTime 100

  #ifdef LEFT
    const int Read[] = {A0, A1, A2, A3, A4, A5};
  #endif
  #ifdef RIGHT
    const int Read[] = {A6, A7, A8, A9, A10, A11};
  #endif

  float V[] = {0, 0, 0, 0, 0, 0};
  bool receive_flag = true;
  bool send_flag = false;
  char char_flag = 'n';
#endif

void setup() {

#ifdef Test
  Serial.begin(57600);

    for (int i = 0; i < 12; i++) {
      if(i < 6){
        SLinit(i) = analogRead(pinRead[i]) * (3.3 / 1023.0);
      }
      else{
        SRinit(i-6) = analogRead(pinRead[i]) * (3.3 / 1023.0);
      }
    }

#endif

#ifdef CAL
    Serial.begin(BAUD);
    for (int i = 0; i < 6; i++){
      pinMode(Read[i], INPUT);
    }
#endif
}
void loop() {
#ifdef Test
  if (!zeroed_flag) {
    for (int i = 0; i < 12; i++) {
      if(i < 6){
        SLinit(i) = analogRead(pinRead[i]) * (3.3 / 1023.0);
      }
      else{
        SRinit(i-6) = analogRead(pinRead[i]) * (3.3 / 1023.0);
      }
    }
    zeroed_flag = true;
  }
  for (int i = 0; i < 12; i++) {
    if(i < 6){
      SL(i) = analogRead(pinRead[i]) * (3.3 / 1023.0);
    }
    else{
      SR(i-6) = analogRead(pinRead[i]) * (3.3 / 1023.0);
    }
  }
  FL = leftCalMat * (SL - SLinit);
  FR = rightCalMat * (SR - SRinit);

  #ifdef SENSOR
    for (int i = 0; i < 6; i++) {
      Serial.print(SL(i));
      Serial.print(", ");
    }
    for (int i = 0; i < 6; i++) {
      Serial.print(SR(i));
      while (i < 5){
        Serial.print(", ");
        break;
      }
    }
    Serial.print('\n');
  #endif

  #ifdef FORCE
    for (int i = 0; i < 3; i++) {
      Serial.print(FL(i));
      Serial.print(", \t");
    }
    for (int i = 0; i < 3; i++) {
      Serial.print(FR(i));
      while (i < 2){
        Serial.print(", \t");
        break;
      }
    }
    Serial.print('\n');
  #endif
  delay(16.66);
#endif

#ifdef CAL
  for (int i = 0; i < 6; i++){
    V[i] = analogRead(Read[i]) * (3.3 / 1023);
  }

  if (receive_flag && Serial.available() > 0) {
    char_flag = Serial.read();
    if (char_flag == 'x') {
      receive_flag = false;
    }
  }
  else if (!receive_flag) {
    for (int i = 0; i < 6; i++) {
      Serial.print(V[i], 2);
      if (i < 5) {
        Serial.print(",");
        }
      else Serial.println();
    }
    receive_flag = true;
  }
#endif
}