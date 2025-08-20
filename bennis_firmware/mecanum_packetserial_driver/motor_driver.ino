/***************************************************************
   Motor driver definitions

   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.

   *************************************************************/

void initMotorController()
{
  pinMode(RightMotorDirPin1, OUTPUT);
  pinMode(RightMotorDirPin2, OUTPUT);
  pinMode(speedPinL, OUTPUT);

  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT);
  pinMode(speedPinR, OUTPUT);

  pinMode(RightMotorDirPin1B, OUTPUT);
  pinMode(RightMotorDirPin2B, OUTPUT);
  pinMode(speedPinLB, OUTPUT);

  pinMode(LeftMotorDirPin1B, OUTPUT);
  pinMode(LeftMotorDirPin2B, OUTPUT);
  pinMode(speedPinRB, OUTPUT);

  setMotorSpeeds(Speeds{0,0,0,0});
}

void setMotorSpeed(int i, int spd)
{
  unsigned char reverse = 0;

  if (spd < 0)
  {
    spd = -spd;
    reverse = 1;
  }
  if (spd > 255)
    spd = 255;

  if (i == LEFT)
  {
    if (reverse == 0)
    {
      digitalWrite(LeftMotorDirPin1, LOW);
      digitalWrite(LeftMotorDirPin2, HIGH);
    }
    else
    {
      digitalWrite(LeftMotorDirPin1, HIGH);
      digitalWrite(LeftMotorDirPin2, LOW);
    }
    analogWrite(speedPinL, spd);
  }
  else if (i == RIGHT)
  {
    if (reverse == 0)
    {
      digitalWrite(RightMotorDirPin1, LOW);
      digitalWrite(RightMotorDirPin2, HIGH);
    }
    else
    {
      digitalWrite(RightMotorDirPin1, HIGH);
      digitalWrite(RightMotorDirPin2, LOW);
    }
    analogWrite(speedPinR, spd);
  }
  else if (i == LEFT_B)
  {
    if (reverse == 0)
    {
      digitalWrite(LeftMotorDirPin1B, LOW);
      digitalWrite(LeftMotorDirPin2B, HIGH);
    }
    else
    {
      digitalWrite(LeftMotorDirPin1B, HIGH);
      digitalWrite(LeftMotorDirPin2B, LOW);
    }
    analogWrite(speedPinLB, spd);
  }
  else if (i == RIGHT_B)
  {
    if (reverse == 0)
    {
      digitalWrite(RightMotorDirPin1B, LOW);
      digitalWrite(RightMotorDirPin2B, HIGH);
    }
    else
    {
      digitalWrite(RightMotorDirPin1B, HIGH);
      digitalWrite(RightMotorDirPin2B, LOW);
    }
    analogWrite(speedPinRB, spd);
  }
}

void setMotorSpeeds(Speeds s)
{
  setMotorSpeed(LEFT, s.fl);
  setMotorSpeed(RIGHT, s.fr);
  setMotorSpeed(LEFT_B, s.rl);
  setMotorSpeed(RIGHT_B, s.rr);
}
