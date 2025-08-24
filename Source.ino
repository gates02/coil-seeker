#include <math.h>
#include <AccelStepper.h>

#define MOTOR_STEPS 2048 // Number of steps required for one revolution (360 degrees)

int hallPin0 = A0; // x, right side when the charging port is facing down
int hallPin1 = A1; // y
int hallPin2 = A2; // -x
int hallPin3 = A3; // -y
int pressPin = A4; // Measures the voltage of a 20k ohm resistor connected to parallel pressure sensors

AccelStepper stepper1(AccelStepper::FULL4WIRE, 7, 9, 8, 10); // Left motor, pin order may vary by driver, requires testing
AccelStepper stepper2(AccelStepper::FULL4WIRE, 3, 5, 4, 6); // Right motor

int analogValue[4];
int pressValue; // Pressure sensor value
int max[4]; // Maximum value within the time interval
int min[4]; // Minimum value within the time interval
double amp[4]; // Amplitude (fluctuation) within the time interval, max-min
double vx; // Velocity in the x-direction
double vy; // Velocity in the y-direction
double mag; // Constant for normalizing vx, vy
unsigned long lastReadTime = 0;
const unsigned long INTERVAL = 100;

long sum[4]={0,0,0,0}; // Sum of Hall sensor values within the interval
long n=0; // Number of summed values
double avg[4]; // Average within the interval so far
double preavg[4]={0,0,0,0};
int nPeak[4]={0,0,0,0}; // Number of values that deviate significantly from the average
bool ifCharge[3]={1,1,1}; // true if charging
int cntCharge=0; // Increments by 1 in the Charge state, becomes 0 when reset
bool isPressed=false;
bool prevIsPressed = false; 
bool endFlags[5]={false}; // In order: right, top, left, bottom, total OR. true if an edge is reached

void checkEnd();
void calcDirection();
void calcPreChargeDirection();
void initMaxMin();
void refreshMaxMin();
void moveMotor(int unitstep); // Moves by unitstep*vx or unitstep*vy
void fixMovement(); // Adjusts motor's moveTo to prevent overshooting when an edge is reached

void setup() {
  Serial.begin(9600);
  stepper1.setMaxSpeed(500); // 500 steps per second (adjustable)
  stepper1.setAcceleration(500); // 500 steps/sec^2 (adjustable)
  stepper1.setCurrentPosition(0); // Set current position to 0

  stepper2.setMaxSpeed(500);
  stepper2.setAcceleration(500);
  stepper2.setCurrentPosition(0);
}


void loop() {
  pressValue = analogRead(pressPin); // Read the voltage of the 20k ohm resistor, a larger value means a heavier load

  isPressed = (pressValue >= 200 && pressValue <= 1000);
  if (isPressed && !prevIsPressed) {
    cntCharge = 0; // Reset every time an object is placed on it
  }
  prevIsPressed = isPressed;

  if (!isPressed) {
    delay(100);
    return;
  }

  analogValue[0] = analogRead(hallPin0);  // 0~1023 range
  analogValue[1] = analogRead(hallPin1); 
  analogValue[2] = analogRead(hallPin2);
  analogValue[3] = analogRead(hallPin3);
  
  checkEnd(); // Check if an edge is reached
  if (endFlags[4]){ // true if any of the four edges are reached
    fixMovement();
  }

  // Drive the motors
  stepper1.run(); // Run one step at a time
  stepper2.run();

  // Check charging status, check the average value and the number of outliers in the interval
  for(int i=0;i<4;i++){
    sum[i]+=analogValue[i];
  } 
  n+=1;

  for(int i=0;i<4;i++){
    if( preavg[i]!=0 && ((preavg[i]-analogValue[i]>5)||(analogValue[i]-preavg[i]>5)) ) nPeak[i]+=1; // The value 5 can be adjusted
  }
  refreshMaxMin();

  unsigned long currentTime = millis();
  if (currentTime - lastReadTime >= INTERVAL) {
    lastReadTime = currentTime; // Update timer
    
    int cntnPeak=0;
    for(int i=0;i<4;i++){
      if(nPeak[i]>(n/4)) cntnPeak++;
    }

    if (cntCharge >= 5) {
      vx = 0;
      vy = 0;
      moveMotor(0); // Complete stop
      return;
    }
    
    if(cntnPeak>2){ // Check if charging; if there are many values far from the average, it's charging
      calcDirection(); // Move towards the direction with the largest magnetic field change, determine vx, vy
      initMaxMin(); // Initialize max and min values for the interval
      moveMotor(20); // Set motor target with vx, vy
    
      ifCharge[2]=ifCharge[1];
      ifCharge[1]=ifCharge[0];
      ifCharge[0]=1;
      if(ifCharge[0]&&ifCharge[1]&&ifCharge[2]) cntCharge++;
    }
    
    else{ // When not charging
      calcPreChargeDirection(); // Move to find the phone before charging, determine vx, vy
      initMaxMin();
      moveMotor(1024);

      ifCharge[2]=ifCharge[1];
      ifCharge[1]=ifCharge[0];
      ifCharge[0]=0;
    }
    
    for(int i=0;i<4;i++){ 
      avg[i]=sum[i]/(double)n;
      preavg[i] = avg[i];
    }
    
    String str_print=String(amp[0])+','+String(amp[1])+','+String(amp[2])+','+String(amp[3])+','+String(cntCharge)+','+String(ifCharge[0]);
      
    for(int i=0;i<4;i++){
      sum[i]=0;
      nPeak[i]=0;
    }
    n=0;
    
    Serial.println(str_print);
  }
  
}

void initMaxMin(){
  for(int i=0;i<4;i++){
    max[i]=analogValue[i];
    min[i]=analogValue[i];
  }
}

void refreshMaxMin(){
  for(int i=0; i<4;i++){
    if(analogValue[i]>max[i])
      max[i]=analogValue[i];
    else if(analogValue[i]<min[i])
      min[i]=analogValue[i];
  }
}

static int endCount0 = 0;
void checkEnd(){
  if (analogValue[0]>600 || analogValue[0]<450){ // This is the bottom, value determined experimentally
    endCount0++;
    if (endCount0 >= 3) endFlags[3] = true; // If exceeded 3 consecutive times, recognize it as a wall
    endFlags[1]=false;
  }
  else if (analogValue[2]>600 || analogValue[2]<450){ // This is the top
    endFlags[1]=true;
    endFlags[3]=false;
  }
  else{
    endFlags[1]=false;
    endFlags[3]=false;
  }
  if (analogValue[1]>600 || analogValue[1]<450){ // This is the rightmost
    endFlags[0]=true;
    endFlags[2]=false;
  }
  else if (analogValue[3]>600 || analogValue[3]<450){ // This is the leftmost
    endFlags[2]=true;
    endFlags[0]=false;
  }
  else{
    endFlags[0]=false;
    endFlags[2]=false;
  }
  // OR operation for all flags
  endFlags[4]= endFlags[0] || endFlags[1] || endFlags[2] || endFlags[3];
}


  
void calcDirection(){
  float g[4] = {
    1.80,   // Sensor 0
    1.16,   // Sensor 1
    1.16,   // Sensor 2
    1.56    // Sensor 3 (reference)
  }; // g[i] can be changed

  for(int i=0;i<4;i++){
    double newAmp = max[i] - min[i];
    amp[i] = 0.6 * amp[i] + 0.4 * newAmp;
    amp[i] /= g[i];
  }


  const float dirX[4] = { 0, 1, 0, -1 };
  const float dirY[4] = { -1, 0, 1, 0 };

  double sumX = 0, sumY = 0;
  for (int i = 0; i < 4; i++) {
    double normAmp = amp[i];  // Sensitivity correction
    sumX += normAmp * dirX[i];
    sumY += normAmp * dirY[i];
  }
  vx = sumX;
  vy = sumY;
  
  mag = sqrt(vx * vx + vy * vy);
  
  if (mag > 1e-6) { // Prevent division by zero
    vx /= mag;
    vy /= mag;
  }

  double totalAmp = amp[0] + amp[1] + amp[2] + amp[3];
  double damping = constrain(1.0 - (totalAmp / 80.0), 0.05, 1.0); // The larger the sum of amps, the smaller the damping. 80.0 is adjustable
  vx *= damping;
  vy *= damping;

  // Apply deadzone (to prevent small jitters)
  if (abs(vx) < 0.03) vx = 0;
  if (abs(vy) < 0.03) vy = 0;

}

float cx;
float cy;

void calcPreChargeDirection(){
  for(int i=0;i<4;i++){
    amp[i]=max[i]-min[i];
  }

  vx=cx;
  vy=cy;
  if (ifCharge[0]==1&&ifCharge[1]==1&&ifCharge[2]==1){ // When transitioning from a charging state to a non-charging state
    vx=1;
    vy=1;
    cx=vx;
    cy=vy;
    cntCharge=0;
    return;
  }
  
  // Reflect when hitting a wall
  if (endFlags[0])      vx = -1;
  else if (endFlags[2]) vx =  1;

  if (endFlags[1])      vy = -1;
  else if (endFlags[3]) vy =  1;
  
  cx=vx;
  cy=vy;
}

void moveMotor(int unitstep){
  double v1=vx+vy;
  double v2=vx-vy;
  long steps1 = round(v1 * unitstep);
  long steps2 = round(v2 * unitstep);
  stepper1.moveTo(stepper1.currentPosition()+steps1); // Set the motor's target position. run() will move it towards the target.
  stepper2.moveTo(stepper2.currentPosition()+steps2);
}

void fixMovement(){ // If it hits a wall and is scheduled to move further into the wall, change the distance to move in that direction to 0
  long d1=stepper1.distanceToGo();
  long d2=stepper2.distanceToGo();
  float x=(d1+d2)/2.0;
  float y=(d1-d2)/2.0;
  if(endFlags[0] && x>0){
    d1=d1-x;
    d2=d2-x;
  }
  else if(endFlags[2] && x<0){
    d1=d1-x;
    d2=d2-x;
  }
  if(endFlags[1] && y>0){
    d1=d1-y;
    d2=d2+y;
  }
  else if(endFlags[3] && y<0){
    d1=d1-y;
    d2=d2+y;
  }
  stepper1.moveTo(stepper1.currentPosition()+d1); 
  stepper2.moveTo(stepper2.currentPosition()+d2);
}