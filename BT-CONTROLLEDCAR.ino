const byte C1 = 2;
const byte C2 = 3;
const byte in1 = 7;
const byte in2 = 8;
const byte ena = 6;

const byte C3 = 5;
const byte C4 = 4;
const byte in3 = 9;
const byte in4 = 10;
const byte enb = 11;

volatile int count1 = 0;
volatile int count2 = 0;

const int dataLength = 2;
double data[dataLength];
bool reverseMotor1 = false;
bool reverseMotor2 = false;

const double constValue = 0.111; // Cambiar este valor según la resolución del encoder y el tipo de motor

unsigned long lastTime = 0, sampleTime = 100;
double u1 = 0.0, u2 = 0.0;
volatile byte ant = 0;
volatile byte act = 0;
volatile int n = 0;

void setup() {
  Serial.begin(9600);
  
  pinMode(C1, INPUT);
  pinMode(C2, INPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(ena, 0);

  pinMode(C3, INPUT);
  pinMode(C4, INPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enb, 0);

  attachInterrupt(digitalPinToInterrupt(C1), updateCount1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C2), updateCount1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C3), updateCount2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C4), updateCount2, CHANGE);

  lastTime = millis();
}

void loop() {
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');
    if (inputString.length() >= 2) {
      data[0] = inputString.substring(0, inputString.length() - 1).toFloat();
      char direction = inputString.charAt(inputString.length() - 1);
      if (direction == 'i') {
        reverseMotor1 = true;
      } else if (direction == 'd') {
        reverseMotor2 = true;
      }
    } else {
      data[0] = inputString.toFloat();
    }

    if (!reverseMotor1) {
      if (data[0] > 0) {
        anticlockwise(in2, in1, ena, data[0]);
      } else {
        clockwise(in2, in1, ena, abs(data[0]));
      }
    } else {
      if (data[0] > 0) {
        clockwise(in2, in1, ena, data[0]);
      } else {
        anticlockwise(in2, in1, ena, abs(data[0]));
      }
      reverseMotor1 = false;
    }

    if (!reverseMotor2) {
      if (data[0] > 0) {
        anticlockwise(in4, in3, enb, data[0]);
      } else {
        clockwise(in4, in3, enb, abs(data[0]));
      }
    } else {
      if (data[0] > 0) {
        clockwise(in4, in3, enb, data[0]);
      } else {
        anticlockwise(in4, in3, enb, abs(data[0]));
      }
      reverseMotor2 = false;
    }
  }

  if (millis() - lastTime >= sampleTime) {
    computeVelocity();
    Serial.print("Velocity 1: ");
    Serial.println(u1);
    Serial.print("Velocity 2: ");
    Serial.println(u2);
  }
}


void updateCount1() {
  byte newState = (digitalRead(C1) << 1) | digitalRead(C2);
    ant=act;                           
    act=PIND & 12;         
                           
    if(ant==0  && act== 4)  n++;
    if(ant==4  && act==12)  n++;
    if(ant==8  && act== 0)  n++;
    if(ant==12 && act== 8)  n++;
    
    if(ant==0 && act==8)  n--; 
    if(ant==4 && act==0)  n--;
    if(ant==8 && act==12) n--;
    if(ant==12 && act==4) n--;
}

void updateCount2() {
  byte newState = (digitalRead(C3) << 1) | digitalRead(C4);
    ant=act;                           
    act=PIND & 12;         
                           
    if(ant==0  && act== 4)  n++;
    if(ant==4  && act==12)  n++;
    if(ant==8  && act== 0)  n++;
    if(ant==12 && act== 8)  n++;
    
    if(ant==0 && act==8)  n--; 
    if(ant==4 && act==0)  n--;
    if(ant==8 && act==12) n--;
    if(ant==12 && act==4) n--;
}

void computeVelocity() {
  u1 = (constValue * count1) / (millis() - lastTime);
  u2 = (constValue * count2) / (millis() - lastTime);
  lastTime = millis();
  count1 = 0;
  count2 = 0;
}

void clockwise(int pin1, int pin2,int analogPin, int pwm)
{
  digitalWrite(pin1, LOW);  
  digitalWrite(pin2, HIGH);      
  analogWrite(analogPin,pwm);
}

void anticlockwise(int pin1, int pin2,int analogPin, int pwm)
{
  digitalWrite(pin1, HIGH);  
  digitalWrite(pin2, LOW);      
  analogWrite(analogPin,pwm);
}
