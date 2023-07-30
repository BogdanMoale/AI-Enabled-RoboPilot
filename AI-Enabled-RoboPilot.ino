#include <QTRSensors.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <NewPing.h>

//DECLARAREA TUTUROR VARIABILELOR SI CONSTANTELOR UTILIZATE PENTRU ALGORITMI
#define OTrigPin 7 //pinul TRIG de la senzorul cu ultrasunete HC SR04
#define IEchoPin 8//pinul Echo
#define MAX_DISTANCE 200
#define MAX_SPEED 190 // setare viteza motor DC pentru algoritmul de evitare a obstacolelor
#define MAX_SPEED_OFFSET 20

NewPing sonar(OTrigPin, IEchoPin, MAX_DISTANCE);// senzorul cu ultrasunete

double Kp=0.17;//0.9
double Kd=0;
double Ki=0.34;//0.6

#define vitezaMaxMotDreapta 250 // viteza maxima robot
#define vitezaMaxMotStanga 250
#define vitezaBazaMotDreapta 250
#define vitezaBazaMotStanga 250//viteza maxima atunci cand robotul este perfect pe linie
#define safeSpeed 250// viteza pentru algoritmul de rezolvare labirint atunci cand merge a doua oara
signed int defaultSpeed = 220;
int dutyCycle = 0;


#define NUM_SENSORS    6    // numarul de senzori utilizati
#define NUM_PER_SENSOR 4
#define EMITTER_PIN    2

#define Senzor3   3
#define Senzor4   4
#define Senzor5   5
#define Senzor2   2
#define Senzor1   1
#define Senzor0   0

int S3;
int S4;
int S5;
int S2;
int S1;
int S0;

int leftNudge;
int replaystage;
int rightNudge;

#define leapTime 150// timp mort

#define Mot1RotFwd1  6
#define Mot1RotBckw1  9

#define Mot2RotFwd2 3
#define Mot2RotBckw2 5

#define led 13



int distance1 = 100;
int speedSet = 0;


int distantaLimita = 25;
int distantaStanga;
int distantaDreapta;
long timp;
Servo motorServo;
byte Byte;

int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval;
char choice;
char contcommand;
int modecontrol = 0;
int power = 0;
const int distancelimit = 20; //Distance limit for obstacles in front
const int sidedistancelimit = 15; //Minimum distance in cm t
int distance;
int numcycles = 0;
char turndirection; //Gets 'l', 'r' or 'f' depending on which direction is obstacle free
const int turntime = 900; //Time the robot spends turning (miliseconds)
int thereis;



QTRSensorsAnalog qtrrc((unsigned char[]) {
  A0, A1, A2, A3, A4, A5
} , NUM_SENSORS, NUM_PER_SENSOR, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

char path[100] = {};
int pathLength;
int readLength;

void vitezaMotorStanga(int viteza) {
  if (viteza > 0) {
    analogWrite(Mot1RotFwd1, viteza);
    analogWrite(Mot1RotBckw1, 0);
  }
  else {
    analogWrite(Mot1RotFwd1, 0);
    analogWrite(Mot1RotBckw1, -viteza);
  }
}

void vitezaMotorDreapta(int viteza) {
  if (viteza > 0) {
    analogWrite(Mot2RotFwd2, viteza);
    analogWrite(Mot2RotBckw2, 0);
  } else {
    analogWrite(Mot2RotFwd2, 0);
    analogWrite(Mot2RotBckw2, -viteza);
  }
}

void Inainte() {
  vitezaMotorStanga(defaultSpeed);
  vitezaMotorDreapta(defaultSpeed);

}

void Inapoi() {
  vitezaMotorStanga(-255);
  vitezaMotorDreapta(-255);
}

void Stanga() {
  vitezaMotorStanga(0);
  vitezaMotorDreapta(255);
}

void Dreapta() {
  vitezaMotorStanga(255);
  vitezaMotorDreapta(0);
}

void Stop() {
  vitezaMotorStanga(0);
  vitezaMotorDreapta(0);
  //de adaugat oprirea senzorilor
}

void RotesteStanga() {
  vitezaMotorStanga(-255);
  vitezaMotorDreapta(255);
}

void RotesteDreapta() {
  vitezaMotorStanga(255);
  vitezaMotorDreapta(-255);
}


void go() {
  digitalWrite (Mot1RotFwd1, HIGH);
  digitalWrite (Mot1RotBckw1, LOW);
  digitalWrite (Mot2RotFwd2, HIGH);
  digitalWrite (Mot2RotBckw2, LOW);
}

void backwards() {
  digitalWrite (Mot1RotFwd1 , LOW);
  digitalWrite (Mot1RotBckw1, HIGH);
  digitalWrite (Mot2RotFwd2, LOW);
  digitalWrite (Mot2RotBckw2, HIGH);
}

int watch() {
  long howfar;
  digitalWrite(OTrigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(OTrigPin, HIGH);
  delayMicroseconds(15);
  digitalWrite(OTrigPin, LOW);
  howfar = pulseIn(IEchoPin , HIGH);
  return (howfar / 2) / 29.1;
  //howfar=howfar*0.01657; //how far away is the object in cm
  //return round(howfar);
}

void turnleft(int t) {
  digitalWrite (Mot1RotFwd1, LOW);
  digitalWrite (Mot1RotBckw1, HIGH);
  digitalWrite (Mot2RotFwd2, HIGH);
  digitalWrite (Mot2RotBckw2, LOW);
  delay(t);
}

void turnright(int t) {
  digitalWrite (Mot1RotFwd1, HIGH);
  digitalWrite (Mot1RotBckw1, LOW);
  digitalWrite (Mot2RotFwd2, LOW);
  digitalWrite (Mot2RotBckw2, HIGH);
  delay(t);
}

void stopmove() {
  digitalWrite (Mot1RotFwd1 , LOW);
  digitalWrite (Mot1RotBckw1, LOW);
  digitalWrite (Mot2RotFwd2, LOW);
  digitalWrite (Mot2RotBckw2, LOW);
}

void watchsurrounding() { //Meassures distances to the right, left, front, left diagonal, right diagonal and asign them in cm to the variables rightscanval,
  //leftscanval, centerscanval, ldiagonalscanval and rdiagonalscanval (there are 5 points for distance testing)
  centerscanval = watch();
  if (centerscanval < distancelimit) {
    stopmove();
  }
  motorServo.write(120);
  delay(100);
  ldiagonalscanval = watch();
  if (ldiagonalscanval < distancelimit) {
    stopmove();
  }
  motorServo.write(160); //Didn't use 180 degrees because my servo is not able to take this angle
  delay(300);
  leftscanval = watch();
  if (leftscanval < sidedistancelimit) {
    stopmove();
  }
  motorServo.write(120);
  delay(100);
  ldiagonalscanval = watch();
  if (ldiagonalscanval < distancelimit) {
    stopmove();
  }
  motorServo.write(80); //I used 80 degrees because its the central angle of my 160 degrees span (use 90 degrees if you are moving your servo through the whole 180 degrees)
  delay(100);
  centerscanval = watch();
  if (centerscanval < distancelimit) {
    stopmove();
  }
  motorServo.write(40);
  delay(100);
  rdiagonalscanval = watch();
  if (rdiagonalscanval < distancelimit) {
    stopmove();
  }
  motorServo.write(0);
  delay(100);
  rightscanval = watch();
  if (rightscanval < sidedistancelimit) {
    stopmove();
  }

  motorServo.write(70); //Finish looking around (look forward again)
  delay(300);
}


char decide() {
  watchsurrounding();
  if (leftscanval > rightscanval && leftscanval > centerscanval) {
    choice = 'l';
  }
  else if (rightscanval > leftscanval && rightscanval > centerscanval) {
    choice = 'r';
  }
  else {
    choice = 'f';
  }
  return choice;
}

void ObstacleAvoiding() {
  go();  // if nothing is wrong go forward using go() function above.
  ++numcycles;
  if (numcycles > 130) { //Watch if something is around every 130 loops while moving forward
    watchsurrounding();
    if (leftscanval < sidedistancelimit || ldiagonalscanval < distancelimit) {
      turnright(turntime);
    }
    if (rightscanval < sidedistancelimit || rdiagonalscanval < distancelimit) {
      turnleft(turntime);
    }
    numcycles = 0; //Restart count of cycles
  }
  distance = watch(); // use the watch() function to see if anything is amotorServo (when the robot is just moving forward and not looking around it will test the distance in front)
  if (distance < distancelimit) { // The robot will just stop if it is completely sure there's an obstacle amotorServo (must test 25 times) (needed to ignore ultrasonic sensor's false signals)
    ++thereis;
  }
  if (distance > distancelimit) {
    thereis = 0;
  } //Count is restarted
  if (thereis > 25) {
    stopmove(); // Since something is amotorServo, stop moving.
    turndirection = decide(); //Decide which direction to turn.
    switch (turndirection) {
      case 'l':
        turnleft(turntime);
        break;
      case 'r':
        turnright(turntime);
        break;
      case 'f':
        ; //Do not turn if there was actually nothing amotorServo
        break;
    }
    thereis = 0;
  }
}


void comparareDistante() {
  if (distantaStanga > distantaDreapta) {
    Stanga();
    Serial.println("Actiune:Stanga");
    delay(1450);
  }
  else if (distantaDreapta > distantaStanga) {
    Dreapta();
    Serial.println("Actiune:Dreapta");
    delay(1650);
  }
}

long Distanta() {
  digitalWrite(OTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(OTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(OTrigPin, LOW);
  timp = pulseIn(IEchoPin, HIGH);
  return (timp / 2) / 29.1;//formula luata din HC-SR04 datasheet
}


void Obstacol() {
  Inainte();
  int distantaInainte = Distanta();
  if (distantaInainte < distantaLimita ) {
    Stop();
    Serial.println("Obstacol detectat.Actiune:Calculez");
    motorServo.write(15);
    delay(550);
    distantaDreapta = Distanta();
    Serial.print("Distanta Dreapta=");
    Serial.println(distantaDreapta);
    delay(500);
    motorServo.write(165);
    delay(700);
    distantaStanga = Distanta();
    Serial.print("Distanta Stanga=");
    Serial.println(distantaStanga);
    delay(500);
    motorServo.write(90);
    delay(200);
    comparareDistante();
    int distMinInapoi = Distanta();
    if ( distMinInapoi <= 20) {
      Inapoi();
      Serial.println("Actiune:Inapoi");
      delay(1800);
      Stanga();
      delay(1750);
    }
  }
  else {
    Serial.println(" Niciun obstacol detectat Actiune:Inainte");
    Inainte();
  }
}

void myPID() { //algoritm complex de urmarit linia in sensul acelor de ceasornic (robotul trebuie sa mearga pe lineie in sensul acelor de ceasornic)
  S3  = analogRead(Senzor3);
  S4    = analogRead(Senzor4);
  S5     = analogRead(Senzor5);
  S2 = analogRead(Senzor2);
  S1   = analogRead(Senzor1);
  S0    = analogRead(Senzor0);

  if (S5 < 200 && S0 < 200 &&
      (S3 > 200 || S2 > 200) ) {
    straight();
  }

  else if ((S0 < 100) && (S4 < 100)) {
    analogWrite(Mot1RotFwd1, safeSpeed);
    analogWrite(Mot2RotFwd2, safeSpeed);
    digitalWrite(Mot1RotBckw1, LOW);
    digitalWrite(Mot2RotBckw2, LOW);
  }
  else if ((S0 > 200) && (S4 < 200)) {
    analogWrite(Mot1RotFwd1, safeSpeed);
    digitalWrite(Mot2RotFwd2, LOW);
    digitalWrite(Mot1RotBckw1, LOW);
    analogWrite(Mot2RotBckw2, safeSpeed);
  }

  else if ((S0 < 100) && (S4 > 200)) {
    digitalWrite(Mot1RotFwd1, LOW);
    analogWrite(Mot2RotFwd2, safeSpeed);
    analogWrite(Mot1RotBckw1, safeSpeed);
    digitalWrite(Mot2RotBckw2, LOW);

  }

  else {
    digitalWrite(Mot1RotFwd1, LOW);
    digitalWrite(Mot2RotFwd2, LOW);
    digitalWrite(Mot1RotBckw1, LOW);
    digitalWrite(Mot2RotBckw2, LOW);
  }
}


void myPIDwithRotation()
{
  S3 = analogRead(Senzor3);
  S4 = analogRead(Senzor4);
  S5  = analogRead(Senzor5);
  S2 = analogRead(Senzor2);
  S1 = analogRead(Senzor1);
  S0  = analogRead(Senzor0);

  if (S5 < 200 && S0 < 200 && (S3 > 200 || S2 > 200) ) {//daca nu este nicio intersectie si cel putin un senzor a detectat linia dute inainte
    straight();
  }
  else {
    leftHandWallWOMemorry();
  }
}

void linie() { //algoritm simplu de urmarit linia cu doi senzori
  int s1 = analogRead(A0);
  int s2 = analogRead(A4);

  if ((s1 < 100) && (s2 < 100)) {
    digitalWrite(Mot1RotFwd1, HIGH);
    digitalWrite(Mot2RotFwd2, HIGH);
    digitalWrite(Mot1RotBckw1, LOW);
    digitalWrite(Mot2RotBckw2, LOW);
  }
  else if ((s1 > 200) && (s2 < 200)) {
    digitalWrite(Mot1RotFwd1, HIGH);
    digitalWrite(Mot2RotFwd2, LOW);
    digitalWrite(Mot1RotBckw1, LOW);
    digitalWrite(Mot2RotBckw2, HIGH);
  }

  else if ((s1 < 100) && (s2 > 200)) {
    digitalWrite(Mot1RotFwd1, LOW);
    digitalWrite(Mot2RotFwd2, HIGH);
    digitalWrite(Mot1RotBckw1, HIGH);
    digitalWrite(Mot2RotBckw2, LOW);

  }

  else {
    digitalWrite(Mot1RotFwd1, LOW);
    digitalWrite(Mot2RotFwd2, LOW);
    digitalWrite(Mot1RotBckw1, LOW);
    digitalWrite(Mot2RotBckw2, LOW);
  }
}

void setup()
{
  Serial.begin(9600);
  pinMode(Mot1RotFwd1, OUTPUT);
  pinMode(Mot1RotBckw1, OUTPUT);
  pinMode(Mot2RotFwd2, OUTPUT);
  pinMode(Mot2RotBckw2, OUTPUT);
  pinMode(OTrigPin, OUTPUT);// set the OTrigPin pin to output (Send sound waves)
  pinMode(IEchoPin, INPUT);
  motorServo.attach(12);
  motorServo.write(70);
  pinMode(A0, INPUT);
  pinMode(A2, INPUT);

  pinMode(Senzor3, INPUT);
  pinMode(Senzor4, INPUT);
  pinMode(Senzor5, INPUT);
  pinMode(Senzor2, INPUT);
  pinMode(Senzor1, INPUT);
  pinMode(Senzor0, INPUT);


  Serial.println("Calibrare senzori!");
  digitalWrite(13, HIGH);
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  for (int i = 0; i < 300; i++) //calibrarea senzoriilor.
  {
    qtrrc.calibrate();
   
  }
  digitalWrite(13, LOW);
  Serial.println("Calibrare terminata!");

}
int lastError = 0;

void myMaze() {
  S3  = analogRead(Senzor3);
  S4    = analogRead(Senzor4);
  S5     = analogRead(Senzor5);
  S2 = analogRead(Senzor2);
  S1   = analogRead(Senzor1);
  S0    = analogRead(Senzor0);

  if (S5 < 200 && S0 < 200 &&
      (S3 > 200 || S2 > 200) ) {
    straight();
  }
  else {
    leftHandWall();
  }

}

void leftHandWallWOMemorry() { //functie care nu memoreaza interesectiile
  if ( S5 > 200 && S0 > 200) {
    analogWrite(Mot1RotFwd1, safeSpeed);
    digitalWrite(Mot1RotBckw1, LOW);
    analogWrite(Mot2RotFwd2, safeSpeed);
    digitalWrite(Mot2RotBckw2, LOW);
    delay(leapTime);
    S3  = analogRead(Senzor3);
    S4    = analogRead(Senzor4);
    S5     = analogRead(Senzor5);
    S2 = analogRead(Senzor2);
    S1   = analogRead(Senzor1);
    S0    = analogRead(Senzor0);

    if (S5 < 200 && S0 < 200) {
      turnLeft2();
    }

  }

  if (S5 > 200) { // if you can turn left then turn left
    analogWrite(Mot1RotFwd1, safeSpeed);
    digitalWrite(Mot1RotBckw1, LOW);
    analogWrite(Mot2RotFwd2, safeSpeed);
    digitalWrite(Mot2RotBckw2, LOW);
    delay(leapTime);
    S3  = analogRead(Senzor3);
    S4    = analogRead(Senzor4);
    S5     = analogRead(Senzor5);
    S2 = analogRead(Senzor2);
    S1   = analogRead(Senzor1);
    S0    = analogRead(Senzor0);

    if (S5 < 200 && S0 < 200) {
      turnLeft1();
      //return;
    }
  }

  if (S0 > 200) {
    analogWrite(Mot1RotFwd1, safeSpeed);
    digitalWrite(Mot1RotBckw1, LOW);
    analogWrite(Mot2RotFwd2, safeSpeed);
    digitalWrite(Mot2RotBckw2, LOW);
    delay(20);
    S3  = analogRead(Senzor3);
    S4    = analogRead(Senzor4);
    S5     = analogRead(Senzor5);
    S2 = analogRead(Senzor2);
    S1   = analogRead(Senzor1);
    S0    = analogRead(Senzor0);

    if (S5 > 200) {
      delay(leapTime - 30);
      S3  = analogRead(Senzor3);
      S4    = analogRead(Senzor4);
      S5     = analogRead(Senzor5);
      S2 = analogRead(Senzor2);
      S1   = analogRead(Senzor1);
      S0    = analogRead(Senzor0);

      if (S0 < 200 && S5 < 200) {
        turnLeft2();
      }
    }
    delay(leapTime - 30);
    S3  = analogRead(Senzor3);
    S4    = analogRead(Senzor4);
    S5     = analogRead(Senzor5);
    S2 = analogRead(Senzor2);
    S1   = analogRead(Senzor1);
    S0    = analogRead(Senzor0);
    if (S5 < 200 && S3 < 200 &&
        S2 < 200 && S0 < 200) {
      turnRight2();
      return;
    }
    straight();
  }
  S3  = analogRead(Senzor3);
  S4    = analogRead(Senzor4);
  S5     = analogRead(Senzor5);
  S2 = analogRead(Senzor2);
  S1   = analogRead(Senzor1);
  S0    = analogRead(Senzor0);
  if (S5 < 200 && S3 < 200 && S2 < 200
      && S0 < 200 && S4 < 200 && S1 < 200) {
    turnAround1();
  }

}

void leftHandWall() {// functie care memoreaza interesectiile
  if ( S5 > 200 && S0 > 200) {
    digitalWrite(Mot1RotFwd1, HIGH);
    digitalWrite(Mot1RotBckw1, LOW);
    digitalWrite(Mot2RotFwd2, HIGH);
    digitalWrite(Mot2RotBckw2, LOW);
    delay(leapTime);
    S3  = analogRead(Senzor3);
    S4    = analogRead(Senzor4);
    S5     = analogRead(Senzor5);
    S2 = analogRead(Senzor2);
    S1   = analogRead(Senzor1);
    S0    = analogRead(Senzor0);

    if (S5 > 200 || S0 > 200) {
      done();
    }
    if (S5 < 200 && S0 < 200) {
      turnLeft();
    }

  }

  if (S5 > 200) { // if you can turn left then turn left
    digitalWrite(Mot1RotFwd1, HIGH);
    digitalWrite(Mot1RotBckw1, LOW);
    digitalWrite(Mot2RotFwd2, HIGH);
    digitalWrite(Mot2RotBckw2, LOW);
    delay(leapTime);
    S3  = analogRead(Senzor3);
    S4    = analogRead(Senzor4);
    S5     = analogRead(Senzor5);
    S2 = analogRead(Senzor2);
    S1   = analogRead(Senzor1);
    S0    = analogRead(Senzor0);

    if (S5 < 200 && S0 < 200) {
      turnLeft();
    }
    else {
      done();
    }
  }

  if (S0 > 200) {
    digitalWrite(Mot1RotFwd1, HIGH);
    digitalWrite(Mot1RotBckw1, LOW);
    digitalWrite(Mot2RotFwd2, HIGH);
    digitalWrite(Mot2RotBckw2, LOW);
    delay(20);
    S3  = analogRead(Senzor3);
    S4    = analogRead(Senzor4);
    S5     = analogRead(Senzor5);
    S2 = analogRead(Senzor2);
    S1   = analogRead(Senzor1);
    S0    = analogRead(Senzor0);

    if (S5 > 200) {
      delay(leapTime - 30);
      S3  = analogRead(Senzor3);
      S4    = analogRead(Senzor4);
      S5     = analogRead(Senzor5);
      S2 = analogRead(Senzor2);
      S1   = analogRead(Senzor1);
      S0    = analogRead(Senzor0);

      if (S0 > 200 && S5 > 200) {
        done();
      }
      else {
        turnLeft();
        return;
      }
    }
    delay(leapTime - 30);
    S3  = analogRead(Senzor3);
    S4    = analogRead(Senzor4);
    S5     = analogRead(Senzor5);
    S2 = analogRead(Senzor2);
    S1   = analogRead(Senzor1);
    S0    = analogRead(Senzor0);
    if (S5 < 200 && S3 < 200 &&
        S2 < 200 && S0 < 200) {
      turnRight();
      return;
    }
    path[pathLength] = 'S';
    Serial.println("S");
    pathLength++;
    Serial.print("Path length: ");
    Serial.println(pathLength);
    if (path[pathLength - 2] == 'B') {
      Serial.println("shortening path");
      shortPath();
    }
    straight();
  }
  S3  = analogRead(Senzor3);
  S4    = analogRead(Senzor4);
  S5     = analogRead(Senzor5);
  S2 = analogRead(Senzor2);
  S1   = analogRead(Senzor1);
  S0    = analogRead(Senzor0);
  if (S5 < 200 && S3 < 200 && S2 < 200
      && S0 < 200 && S4 < 200 && S1 < 200) {
    turnAround();
  }




}
void done() {
  digitalWrite(Mot1RotFwd1, LOW);
  digitalWrite(Mot1RotBckw1, LOW);
  digitalWrite(Mot2RotFwd2, LOW);
  digitalWrite(Mot2RotBckw2, LOW);
  replaystage = 1;
  path[pathLength] = 'D';
  pathLength++;
  while (analogRead(Senzor5) > 200) {
    digitalWrite(led, LOW);
    delay(150);
    digitalWrite(led, HIGH);
    delay(150);
  }
  delay(500);
  replay();
}

void turnLeft() {

  while (analogRead(Senzor2) > 200 || analogRead(Senzor3) > 200) {
    digitalWrite(Mot1RotFwd1, LOW);
    digitalWrite(Mot1RotBckw1, HIGH);
    digitalWrite(Mot2RotFwd2, HIGH);
    digitalWrite(Mot2RotBckw2, LOW);
    delay(2);
    digitalWrite(Mot1RotFwd1, LOW);
    digitalWrite(Mot1RotBckw1, LOW);
    digitalWrite(Mot2RotFwd2, LOW);
    digitalWrite(Mot2RotBckw2, LOW);
    delay(1);
  }

  while (analogRead(Senzor2) < 200) {
    digitalWrite(Mot1RotFwd1, LOW);
    digitalWrite(Mot1RotBckw1, HIGH);
    digitalWrite(Mot2RotFwd2, HIGH);
    digitalWrite(Mot2RotBckw2, LOW);
    delay(2);
    digitalWrite(Mot1RotFwd1, LOW);
    digitalWrite(Mot1RotBckw1, LOW);
    digitalWrite(Mot2RotFwd2, LOW);
    digitalWrite(Mot2RotBckw2, LOW);
    delay(1);
  }

  if (replaystage == 0) {
    path[pathLength] = 'L';
    Serial.println("L");
    pathLength++;
    Serial.print("Path length: ");
    Serial.println(pathLength);
    if (path[pathLength - 2] == 'B') {
      Serial.println("shortening path");
      shortPath();
    }
  }
}


void turnLeft2() {

  while (analogRead(Senzor2) > 200 || analogRead(Senzor3) > 200) {
    digitalWrite(Mot1RotFwd1, LOW);
    digitalWrite(Mot1RotBckw1, HIGH);
    digitalWrite(Mot2RotFwd2, HIGH);
    digitalWrite(Mot2RotBckw2, LOW);
    delay(2);
    digitalWrite(Mot1RotFwd1, LOW);
    digitalWrite(Mot1RotBckw1, LOW);
    digitalWrite(Mot2RotFwd2, LOW);
    digitalWrite(Mot2RotBckw2, LOW);
    delay(1);
  }

  while (analogRead(Senzor2) < 200) {
    digitalWrite(Mot1RotFwd1, LOW);
    digitalWrite(Mot1RotBckw1, HIGH);
    digitalWrite(Mot2RotFwd2, HIGH);
    digitalWrite(Mot2RotBckw2, LOW);
    delay(2);
    digitalWrite(Mot1RotFwd1, LOW);
    digitalWrite(Mot1RotBckw1, LOW);
    digitalWrite(Mot2RotFwd2, LOW);
    digitalWrite(Mot2RotBckw2, LOW);
    delay(1);
  }

}

void turnRight() {
  while (analogRead(Senzor2) > 200) {
    digitalWrite(Mot1RotFwd1, HIGH);
    digitalWrite(Mot1RotBckw1, LOW);
    digitalWrite(Mot2RotFwd2, LOW);
    digitalWrite(Mot2RotBckw2, HIGH);
    delay(2);
    digitalWrite(Mot1RotFwd1, LOW);
    digitalWrite(Mot1RotBckw1, LOW);
    digitalWrite(Mot2RotFwd2, LOW);
    digitalWrite(Mot2RotBckw2, LOW);
    delay(1);
  }
  while (analogRead(Senzor2) < 200) {
    digitalWrite(Mot1RotFwd1, HIGH);
    digitalWrite(Mot1RotBckw1, LOW);
    digitalWrite(Mot2RotFwd2, LOW);
    digitalWrite(Mot2RotBckw2, HIGH);
    delay(2);
    digitalWrite(Mot1RotFwd1, LOW);
    digitalWrite(Mot1RotBckw1, LOW);
    digitalWrite(Mot2RotFwd2, LOW);
    digitalWrite(Mot2RotBckw2, LOW);
    delay(1);
  }
  while (analogRead(Senzor3) < 200) {
    digitalWrite(Mot1RotFwd1, HIGH);
    digitalWrite(Mot1RotBckw1, LOW);
    digitalWrite(Mot2RotFwd2, LOW);
    digitalWrite(Mot2RotBckw2, HIGH);
    delay(2);
    digitalWrite(Mot1RotFwd1, LOW);
    digitalWrite(Mot1RotBckw1, LOW);
    digitalWrite(Mot2RotFwd2, LOW);
    digitalWrite(Mot2RotBckw2, LOW);
    delay(1);
  }

  if (replaystage == 0) {
    path[pathLength] = 'R';
    Serial.println("R");
    pathLength++;
    Serial.print("Path length: ");
    Serial.println(pathLength);
    if (path[pathLength - 2] == 'B') {
      Serial.println("shortening path");
      shortPath();
    }
  }

}


void turnRight2() {
  while (analogRead(Senzor2) > 200) {
    digitalWrite(Mot1RotFwd1, HIGH);
    digitalWrite(Mot1RotBckw1, LOW);
    digitalWrite(Mot2RotFwd2, LOW);
    digitalWrite(Mot2RotBckw2, HIGH);
    delay(2);
    digitalWrite(Mot1RotFwd1, LOW);
    digitalWrite(Mot1RotBckw1, LOW);
    digitalWrite(Mot2RotFwd2, LOW);
    digitalWrite(Mot2RotBckw2, LOW);
    delay(1);
  }
  while (analogRead(Senzor2) < 200) {
    digitalWrite(Mot1RotFwd1, HIGH);
    digitalWrite(Mot1RotBckw1, LOW);
    digitalWrite(Mot2RotFwd2, LOW);
    digitalWrite(Mot2RotBckw2, HIGH);
    delay(2);
    digitalWrite(Mot1RotFwd1, LOW);
    digitalWrite(Mot1RotBckw1, LOW);
    digitalWrite(Mot2RotFwd2, LOW);
    digitalWrite(Mot2RotBckw2, LOW);
    delay(1);
  }
  while (analogRead(Senzor3) < 200) {
    digitalWrite(Mot1RotFwd1, HIGH);
    digitalWrite(Mot1RotBckw1, LOW);
    digitalWrite(Mot2RotFwd2, LOW);
    digitalWrite(Mot2RotBckw2, HIGH);
    delay(2);
    digitalWrite(Mot1RotFwd1, LOW);
    digitalWrite(Mot1RotBckw1, LOW);
    digitalWrite(Mot2RotFwd2, LOW);
    digitalWrite(Mot2RotBckw2, LOW);
    delay(1);
  }

}


void straight() {
  unsigned int sensors[6];
  int setPoint = 2500;
  int position = qtrrc.readLine(sensors);

  int error = position - setPoint;
  int derivative = error - lastError;
  long integral = integral + error;

  int PID_corection = Kp * error + Ki * integral + Kd * (error - lastError);
  lastError = error;

  int VitezaMotDreapta = vitezaBazaMotDreapta + PID_corection;
  int VitezaMotStanga = vitezaBazaMotStanga - PID_corection;

  if (VitezaMotDreapta > vitezaMaxMotDreapta )
    VitezaMotDreapta = vitezaMaxMotDreapta; // impiedica motoarele sa mearga cu viteza mai mare decat cea maxima
  if (VitezaMotStanga > vitezaMaxMotStanga )
    VitezaMotStanga = vitezaMaxMotStanga;
  if (VitezaMotDreapta < 0)
    VitezaMotDreapta = 0; //
  if (VitezaMotStanga < 0) VitezaMotStanga = 0; // keep the motor speed positive
  {


    analogWrite(Mot2RotFwd2, VitezaMotDreapta);
    digitalWrite(Mot1RotBckw1, LOW);
    analogWrite(Mot1RotFwd1, VitezaMotStanga);
    digitalWrite(Mot2RotBckw2, LOW);

  }

}

void turnAround() {
  digitalWrite(Mot1RotFwd1, HIGH);
  digitalWrite(Mot1RotBckw1, LOW);
  digitalWrite(Mot2RotFwd2, HIGH);
  digitalWrite(Mot2RotBckw2, LOW);
  delay(200);
  while (analogRead(Senzor3) < 200) {
    digitalWrite(Mot1RotFwd1, LOW);
    digitalWrite(Mot1RotBckw1, HIGH);
    digitalWrite(Mot2RotFwd2, HIGH);
    digitalWrite(Mot2RotBckw2, LOW);
    delay(100);
    digitalWrite(Mot1RotFwd1, LOW);
    digitalWrite(Mot1RotBckw1, LOW);
    digitalWrite(Mot2RotFwd2, LOW);
    digitalWrite(Mot2RotBckw2, LOW);
    delay(2);
  }
  path[pathLength] = 'B';
  pathLength++;
  straight();
  Serial.println("B");
  Serial.print("Path length: ");
  Serial.println(pathLength);
}


void turnAround1() {
  digitalWrite(Mot1RotFwd1, HIGH);
  digitalWrite(Mot1RotBckw1, LOW);
  digitalWrite(Mot2RotFwd2, HIGH);
  digitalWrite(Mot2RotBckw2, LOW);
  delay(200);
  while (analogRead(Senzor3) < 200) {
    digitalWrite(Mot1RotFwd1, LOW);
    digitalWrite(Mot1RotBckw1, HIGH);
    digitalWrite(Mot2RotFwd2, HIGH);
    digitalWrite(Mot2RotBckw2, LOW);
    delay(100);
    digitalWrite(Mot1RotFwd1, LOW);
    digitalWrite(Mot1RotBckw1, LOW);
    digitalWrite(Mot2RotFwd2, LOW);
    digitalWrite(Mot2RotBckw2, LOW);
    delay(2);
  }
}

void shortPath() {
  int shortDone = 0;
  if (path[pathLength - 3] == 'L' && path[pathLength - 1] == 'R') {
    pathLength -= 3;
    path[pathLength] = 'B';
    //Serial.println("test1");
    shortDone = 1;
  }

  if (path[pathLength - 3] == 'L' && path[pathLength - 1] == 'S' && shortDone == 0) {
    pathLength -= 3;
    path[pathLength] = 'R';
    //Serial.println("test2");
    shortDone = 1;
  }

  if (path[pathLength - 3] == 'R' && path[pathLength - 1] == 'L' && shortDone == 0) {
    pathLength -= 3;
    path[pathLength] = 'B';
    //Serial.println("test3");
    shortDone = 1;
  }


  if (path[pathLength - 3] == 'S' && path[pathLength - 1] == 'L' && shortDone == 0) {
    pathLength -= 3;
    path[pathLength] = 'R';
    //Serial.println("test4");
    shortDone = 1;
  }

  if (path[pathLength - 3] == 'S' && path[pathLength - 1] == 'S' && shortDone == 0) {
    pathLength -= 3;
    path[pathLength] = 'B';
    //Serial.println("test5");
    shortDone = 1;
  }
  if (path[pathLength - 3] == 'L' && path[pathLength - 1] == 'L' && shortDone == 0) {
    pathLength -= 3;
    path[pathLength] = 'S';
    //Serial.println("test6");
    shortDone = 1;
  }

  path[pathLength + 1] = 'D';
  path[pathLength + 2] = 'D';
  pathLength++;
  Serial.print("Path length: ");
  Serial.println(pathLength);
  printPath();
}




void printPath() {
  Serial.println("Optimized path: ");
  int x;
  while (x <= pathLength) {
    Serial.print(path[x]);
    x++;
  }
  //Serial.println("+++++++++++++++++");
}


void replay() {
  S3  = analogRead(Senzor3);
  S4    = analogRead(Senzor4);
  S5     = analogRead(Senzor5);
  S2 = analogRead(Senzor2);
  S1   = analogRead(Senzor1);
  S0    = analogRead(Senzor0);
  if (S5 < 200 && S0 < 200) {
    straight();
  }
  else {
    if (path[readLength] == 'D') {
      analogWrite(Mot1RotFwd1, safeSpeed);
      digitalWrite(Mot1RotBckw1, LOW);
      analogWrite(Mot2RotFwd2, safeSpeed);
      digitalWrite(Mot2RotBckw2, LOW);
      delay(100);
      digitalWrite(Mot1RotFwd1, LOW);
      digitalWrite(Mot1RotBckw1, LOW);
      digitalWrite(Mot2RotFwd2, LOW);
      digitalWrite(Mot2RotBckw2, LOW);
      endMotion();
    }
    if (path[readLength] == 'L') {
      analogWrite(Mot1RotFwd1, safeSpeed);
      digitalWrite(Mot1RotBckw1, LOW);
      analogWrite(Mot2RotFwd2, safeSpeed);
      digitalWrite(Mot2RotBckw2, LOW);
      delay(leapTime);
      turnLeft();
    }
    if (path[readLength] == 'R') {
      analogWrite(Mot1RotFwd1, safeSpeed);
      digitalWrite(Mot1RotBckw1, LOW);
      analogWrite(Mot2RotFwd2, safeSpeed);
      digitalWrite(Mot2RotBckw2, LOW);
      delay(leapTime);
      turnRight();
    }
    if (path[readLength] == 'S') {
      analogWrite(Mot1RotFwd1, safeSpeed);
      digitalWrite(Mot1RotBckw1, LOW);
      analogWrite(Mot2RotFwd2, safeSpeed);
      digitalWrite(Mot2RotBckw2, LOW);
      delay(leapTime);
      straight();
    }

    readLength++;
  }

  replay();

}

void endMotion() {
  digitalWrite(led, LOW);
  delay(500);
  digitalWrite(led, HIGH);
  delay(200);
  digitalWrite(led, LOW);
  delay(200);
  digitalWrite(led, HIGH);
  delay(500);
  endMotion();
}

void VoceStanga() {
  int distanceL = 0;
  distanceL = lookLeft1();
  if (distanceL >= 9) {
    Stanga();
    delay(1000);
    Stop();
    delay(1000);
  }
  else {
    Serial.print("Obstacol detectat la:");
    Serial.print(distanceL);
    Serial.println("cm");
  }
}

void VoceDreapta() {
  int distanceR = 0;
  distanceR = lookRight1();
  if (distanceR >= 10) {
    Dreapta();
    delay(1000);
    Stop();
    delay(1000);
  }
  else {
    Serial.print("Obstacol detectat la:");
    Serial.print(distanceR);
    Serial.println("cm");
  }
}

void voceInainte() {
  int distanceF = 0;
  distanceF = readPing1();
  if (distanceF <= 10) {
    Serial.print("Obstacol detectat la:");
    Serial.print(distanceF);
    Serial.println("cm");
    moveStop1();
  }
  else {
    Inainte();
    Serial.println("Niciun obstacol detectat");
  }
}




void EvitareObstacol() {
  int distanceR = 0;
  int distanceL =  0;
  delay(40);

  if (distance1 <= 20)
  {
    moveStop1();
    delay(100);
    Inapoi1();
    delay(300);
    moveStop1();
    delay(200);
    distanceR = lookRight1();
    delay(200);
    distanceL = lookLeft1();
    delay(200);

    if (distanceR >= distanceL)
    {
      turnRight1();
      moveStop1();
    } else
    {
      turnLeft1();
      moveStop1();
    }
  } else
  {
    moveForward1();
  }
  distance1 = readPing1();
}

int lookRight1()
{
  motorServo.write(10);
  delay(500);
  int distance1 = readPing1();
  delay(100);
  motorServo.write(70);
  return distance1;
}

int lookLeft1()
{
  motorServo.write(140);
  delay(500);
  int distance1 = readPing1();
  delay(100);
  motorServo.write(70);
  return distance1;
  delay(100);
}

int readPing1() {
  delay(70);
  int cm = sonar.ping_cm();
  if (cm == 0)
  {
    cm = 250;
  }
  return cm;
}

void moveStop1() {
  digitalWrite (Mot1RotFwd1 , LOW);
  digitalWrite (Mot1RotBckw1, LOW);
  digitalWrite (Mot2RotFwd2, LOW);
  digitalWrite (Mot2RotBckw2, LOW);
}

void moveForward1() {
  vitezaMotorDreapta(250);
  vitezaMotorStanga(250);
}

void Inapoi1() {
  vitezaMotorStanga(-255);
  vitezaMotorDreapta(-255);
}

void turnRight1() {
  vitezaMotorStanga(250);
  vitezaMotorDreapta(-250);
  delay(300);
  vitezaMotorStanga(250);
  vitezaMotorDreapta(250);
}

void turnLeft1() {
  vitezaMotorDreapta(250);
  vitezaMotorStanga(-250);
  delay(300);
  vitezaMotorStanga(250);
  vitezaMotorDreapta(250);
}




void loop() {

  if (Serial.available() > 0) {
    Byte = Serial.read();
  }
  switch (Byte) {
    case 51://inainte
      //Serial.write("Functia Inainte");// pentru test
      Inainte();
      break;
    case 52://inapoi
      //Serial.write("Functia Inapoi");
      Inapoi();
      break;
    case 53://stanga
      //Serial.write("Functia Stanga");
      Stanga();
      break;
    case 54://dreapta
      //Serial.write("Functia Dreapta");
      Dreapta();
      break;
    case 55://opreste
      //Serial.write("Stop");
      Stop();
      break;
    case 56://urmareste linia
      //Serial.write("Functia LiniePID");
      //myPIDwithRotation();
      myPID();
      break;
    case 57://linie
      //Serial.write("Functia Linie");
      linie();
      break;
    case 58://rotire dreapta
      //Serial.write("Functia RotesteDreapta");
      RotesteDreapta();
      break;
    case 59://rotire stanga
      //Serial.write("Functia RotesteStanga");
      RotesteStanga();
      break;
    case 60:
      //Serial.write("Evitare Obstacole");
      //EvitareObstacol();
      ObstacleAvoiding();
      break;
    case 61:
      //Serial.write("Maze");
      myMaze();
      break;
    
      case 'p':
      Kp = Byte;
      break;
      case 'i':
      Ki = Byte;
      break;
      case 'd':
      Kp = Byte;
      break;
    
    case 'a':
      voceInainte();
      break;
    case 'b':
      VoceStanga();
      break;
    case 'c':
      VoceDreapta();
      break;
    default:
      //vitezaMotorDreapta(Byte);
      //vitezaMotorStanga(Byte);
      break;
  }
}




