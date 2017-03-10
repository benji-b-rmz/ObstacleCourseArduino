/*
AUTHORS : Benji Ramirez & Eric Valenzuela
CREATED : 02/25/2016
MODIFIED: 05/09/2016
*/

#include <math.h>
#include <ChainableLED.h>
#include <motordriver_4wd.h>
#include <seeed_pwm.h>

int button_state = 0;   // state of push button
int left_dirn = 1, right_dirn = 1;

double dtheta = 0.0, theta = PI / 2.0, x = 50.0, y = 50.0, dx = 0.0, dy = 0.0;
double theta_trn;

long left_encoder_count = 0, right_encoder_count = 0;

const int SEARCH = 3;

float Rsensor; //Resistance of sensor in K
int lightCounter = 0;
int light [SEARCH];

// maze boundaries
const int FINISH_Y = 1980;
const int FINISH_X = 160;

// light thresholds
const int LOWTHRESH = 400;
const int UPTHRESH = 690;

// pins
const int PING_PIN = 11;
const int LIGHT_PIN = A0;
const int BUTTON_PIN = SCL;

// numerics
const double PIE = 3.14159265;
const double PIE_O2 = PIE / 2.0;
const double PIE2 = PIE * 2.0;

// dead reck
#define D          158.0
#define NUM_LEDS   1
#define RW         42.5  // radius wheel
#define TPR        72

void go(int);
ChainableLED leds(0, 1, NUM_LEDS);

enum // transisiton states
{
  FWD, REV, TRN, FIRST_Y, SECOND_Y, SETUP, SWP, LGT, STP
} state;

enum // led colors
{
  RED_, GREEN_, YELLOW_, BLUE_, ORANGE_, PURPLE_, PINK_, BROWN_
} colors;

void setup()
{
  pinMode(BUTTON_PIN, INPUT);
  attachInterrupt(1, RightEncoder, CHANGE);
  attachInterrupt(0, LeftEncoder, CHANGE);

  leds.init();
  setColor(GREEN_);

  MOTOR.init();
  MOTOR.setSpeedDir1(10, DIRF); // go straight
  MOTOR.setSpeedDir2(10, DIRR);

  state = FWD;
  right_encoder_count = left_encoder_count = 0;
}

void loop()
{
  int dist_cm = measure(PING_PIN);
  int sensorValue = analogRead(LIGHT_PIN);
  Rsensor = (float)(1023 - sensorValue) * 10 / sensorValue;

  if (digitalRead(BUTTON_PIN) == 1)
  {
    button_state = 1;
  }

  if (state == FWD || state == SECOND_Y)
  {
    dtheta = 0; 
  }
  else if (state == TRN || state == FIRST_Y)
  {
    if (right_dirn == -1)
    {
      dtheta = -1 * (PIE / 2);
    }
    else
    {
      dtheta = (PIE / 2);
    }
  }

  theta = fmod(theta + dtheta, PIE2);
  theta_trn = theta_trn + dtheta;

  dx = PIE * RW * cos(theta) * ((double) (left_encoder_count + right_encoder_count) / TPR);
  x = x + dx;

  dy = PIE * RW * sin(theta) * ((double) (left_encoder_count + right_encoder_count) / TPR);
  y = y + dy;

  // debug(dist_cm, sensorValue, Rsensor);

  switch (state)
  {
    case FWD:
      if (button_state == 1)
      {
        signalBackward();
        state = REV;
      }
      else
      {
        resetEncoders();
      }

      if (y >= FINISH_Y)
      {
        hardRight();
        button_state = 0;
        state = FIRST_Y;
      }
      break;

    case REV:
      if (dist_cm > 20)
      {
        signalRight();
        button_state = 0;
        state = TRN;
      }
      else
      {
        signalLeft();
        button_state = 0;
        state = TRN;
      }
      break;

    case TRN:
      signalForward();
      state = FWD;
      break;

    case FIRST_Y:
      forwardFast();
      state = SECOND_Y;
    break;

    case SECOND_Y:
      forwardFast();
      if (x >= FINISH_X)
      {
        state = SETUP;
      }
    break;

    case SETUP:
       searchLeft();
       state = SWP;
    break;

    case SWP:
    if(lightCounter > SEARCH)
    {
      int max_ = -1;  
      int index = 0; 
      for(int i = 1; i <= SEARCH; i++)
      {
          if(max_< light[i])
          {
            max_ = light[i];
            index = i;
          }
      }  
    
      turnback(index);
      state = LGT;
      lightCounter = 0; // reset counter
    }
    else
    {
      light[lightCounter++] = sensorValue;  
      searchLeft();
    }
    break;
      
    case LGT:
      if(sensorValue < UPTHRESH)
      {
         searchForward();
         state = SWP;
      }
      else
      {
         state = STP;
      }
    break;
    
    case STP:
      MOTOR.setStop1();
      MOTOR.setStop2();
      state = STP;
      break;

    default:
      break;
  }
}

/***********************
 * MAZE FUNCTIONS
 ************************/
void signalForward()
{
  left_dirn = 1;
  right_dirn = 1;
  resetEncoders();
  MOTOR.setSpeedDir1(12, DIRF);
  MOTOR.setSpeedDir2(14, DIRR); 
}

void signalBackward()
{
  left_dirn = -1;
  right_dirn = -1;
  resetEncoders();
  MOTOR.setSpeedDir1(15, DIRR);
  MOTOR.setSpeedDir2(15, DIRF);
  delay(100);
}

void signalLeft()
{
  left_dirn = -1;
  right_dirn = 1;
  resetEncoders();
  MOTOR.setSpeedDir1(9, DIRR); // go back a bit
  MOTOR.setSpeedDir2(10, DIRF);
  delay(25); 
  theta_trn = 0.0; // note
  left_dirn = -1;
  right_dirn = 1;
  resetEncoders();
  MOTOR.setSpeedDir1(43, DIRR);
  MOTOR.setSpeedDir2(38, DIRR);
  delay(410);
}

void signalRight()
{
  left_dirn = -1;
  right_dirn = 1;
  resetEncoders();
  MOTOR.setSpeedDir1(9, DIRR); // go back a bit
  MOTOR.setSpeedDir2(10, DIRF);
  delay(25);
  theta_trn = 0.0; // note
  left_dirn = 1;
  right_dirn = -1;
  resetEncoders();
  MOTOR.setSpeedDir1(38, DIRF);
  MOTOR.setSpeedDir2(43, DIRF);
  delay(340);
}

/***********************
 * LIGHT FUNCTIONS
 ************************/
void searchLeft()
{
  MOTOR.setSpeedDir1(20, DIRR);
  MOTOR.setSpeedDir2(15, DIRR);
  delay(200);
  MOTOR.setStop1();
  MOTOR.setStop2();
  delay(15);  
}

void turnback(int index)
{
  for(int i = 1; i <= (SEARCH-index); i++)
  {
    MOTOR.setSpeedDir1(28, DIRF);
    MOTOR.setSpeedDir2(25, DIRF);
    delay(175);  
    MOTOR.setStop1();
    MOTOR.setStop2();
    delay(15);
  }
}

void searchForward()
{
  MOTOR.setSpeedDir1(38, DIRF);
  MOTOR.setSpeedDir2(35, DIRR); 
  delay(40);
}

void forwardFast()
{
  left_dirn = 1;
  right_dirn = 1;
  resetEncoders();
  MOTOR.setSpeedDir1(30, DIRF);
  MOTOR.setSpeedDir2(30, DIRR); 
}

void hardRight()
{
  theta_trn = 0.0; // note
  left_dirn = 1;
  right_dirn = -1;
  resetEncoders();
  MOTOR.setSpeedDir1(38, DIRF);
  MOTOR.setSpeedDir2(43, DIRF);
  delay(285);
}  
 
/***********************
 * ENCODERS FUNCTIONS
 ************************/
void resetEncoders()
{
  right_encoder_count = left_encoder_count = 0;
}

void LeftEncoder()
{
  left_encoder_count = left_encoder_count + left_dirn;
}

void RightEncoder()
{
  right_encoder_count = right_encoder_count + right_dirn;
}

/***********************
 * RANGE SENSOR FUNCTIONS
 ************************/

int measure(int pingPin)
{
  long duration, inches, cm;

  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  // convert the time into a distance
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);
  delay(100);

  return cm;
}

long microsecondsToInches(long microseconds)
{
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
}

/***********************
 * LED/COLOR FUNCTION
 ************************/
void setColor(int color)
{
  switch (color)
  {
    case RED_:
      leds.setColorRGB(0, 100, 0, 0);
      break;
    case GREEN_:
      leds.setColorRGB(0, 0, 100, 0);
      break;
    case YELLOW_:
      leds.setColorRGB(0, 100, 100, 0);
      break;
    case BLUE_:
      leds.setColorRGB(0, 0, 0, 100);
      break;
    case ORANGE_:
      leds.setColorRGB(0, 100, 50, 0);
      break;
    case PURPLE_:
      leds.setColorRGB(0, 100, 0, 100);
      break;
    case PINK_:
      leds.setColorRGB(0, 100, 40, 40);
      break;
    case BROWN_:
      leds.setColorRGB(0, 100, 100, 100);
      break;
    default:
      break;
  }
}

/***********************
 * PRINT/DISPLAY FUNCTION 
 ************************/
void debug(int dist_cm, int sensorValue, float Rsensor)
{
  Serial.print("cm: ");
  Serial.println(dist_cm);
  Serial.println();

  Serial.print("left_encoder_count: ");
  Serial.print(left_encoder_count);
  Serial.print(", right_encoder_count: ");
  Serial.println(right_encoder_count);
  Serial.println();

  Serial.print("theta: ");
  Serial.print(theta * 180.0 / PI);
  Serial.print("dtheta: ");
  Serial.print(dtheta * 180.0 / PI);
  Serial.print("  x: ");
  Serial.print(x);
  Serial.print("  y: ");
  Serial.println(y);
  Serial.println();

  Serial.println("the analog read data is ");
  Serial.println(sensorValue);
  Serial.println("the sensor resistance is ");
  Serial.println(Rsensor,DEC);//show the light intensity on the serial monitor;
  Serial.println();
}
