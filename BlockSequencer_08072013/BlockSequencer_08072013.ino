#include <ClickButton.h>
#include <Bounce.h>


// copper tape switch info
byte switchPins[] = {
  6, 7, 8, 9, 10};
boolean switchStates[5];
boolean flag = false;

// encoder info
byte encoderPin1 = 2;
byte encoderPin2 = 3;
byte encoderSwitchPin = 4; //push button switch
volatile long encoderValue = 0;

long lastencoderValue = 0;

int lastMSB = 0;
int lastLSB = 0;

int sampleSize = 13;

/* Timer2 reload value, globally available */
unsigned int tcnt2;
Bounce bouncer1 = Bounce( encoderPin1 ,5 );

int buttonMode = 1;
int prevButtonMode = 1;
ClickButton encoderButton(encoderSwitchPin, LOW, CLICKBTN_PULLUP);

// RGB LED SELECTOR
int selectorRedPin = A4;
int selectorGreenPin = A5;
int selectorBluePin = A6;

int switchPinsRed[] = {A13, A7};  
int switchPinsGreen[] = {A14, A8};
int switchPinsBlue[] = {A15, A9};

int valToWriteRed = 255;
int valToWriteGreen = 255;
int valToWriteBlue = 255;


void setup() {

  // begin serial monitor
  Serial.begin(9600);

  // initialize encoder values and pullup resistors
  pinMode(encoderPin1, INPUT);
  pinMode(encoderPin2, INPUT);
  //pinMode(encoderSwitchPin, INPUT);
  digitalWrite(encoderPin1, HIGH);
  digitalWrite(encoderPin2, HIGH);
  //digitalWrite(encoderSwitchPin, HIGH); //turn pullup resistor on

  pinMode(selectorRedPin, OUTPUT);
  pinMode(selectorGreenPin, OUTPUT);
  pinMode(selectorBluePin, OUTPUT);  
  analogWrite(selectorRedPin, valToWriteRed);
  analogWrite(selectorGreenPin, valToWriteGreen);
  analogWrite(selectorBluePin, valToWriteBlue);

  encoderButton.maxPresses = 2;
  encoderButton.debounceTime = 20;
  encoderButton.multiclickTime = 250;

  // initialize copper tape switch pins
  for (byte thisSwitch=0; thisSwitch < sizeof(switchPins); thisSwitch++) {
    pinMode(switchPins[thisSwitch], INPUT);
  }

  // initialize other RGB LEDs
  for (int i = 0; i < sizeof(switchPinsRed); i++) {
    pinMode(switchPinsRed[i], OUTPUT);
    pinMode(switchPinsGreen[i], OUTPUT);
    pinMode(switchPinsBlue[i], OUTPUT);
    analogWrite(switchPinsRed[i], valToWriteRed);
    analogWrite(switchPinsGreen[i], valToWriteGreen);
    analogWrite(switchPinsBlue[i], valToWriteBlue);
  }

  /* First disable the timer overflow interrupt while we're configuring */
  TIMSK2 &= ~(1<<TOIE2);  

  /* Configure timer2 in normal mode (pure counting, no PWM etc.) */
  TCCR2A &= ~((1<<WGM21) | (1<<WGM20));  
  TCCR2B &= ~(1<<WGM22);  

  /* Select clock source: internal I/O clock */
  ASSR &= ~(1<<AS2);  

  /* Disable Compare Match A interrupt enable (only want overflow) */
  TIMSK2 &= ~(1<<OCIE2A);  

  /* Now configure the prescaler to CPU clock divided by 128 */
  TCCR2B |= (1<<CS22)  | (1<<CS20); // Set bits  
  TCCR2B &= ~(1<<CS21);             // Clear bit  

  /* We need to calculate a proper value to load the timer counter. 
   * The following loads the value 131 into the Timer 2 counter register 
   * The math behind this is: 
   * (CPU frequency) / (prescaler value) = 125000 Hz = 8us. 
   * (desired period) / 8us = 125. 
   * MAX(uint8) + 1 - 125 = 131; 
   */
  /* Save value globally for later reload in ISR */
  tcnt2 = 131;   

  /* Finally load end enable the timer */
  TCNT2 = tcnt2;  
  TIMSK2 |= (1<<TOIE2);  

}


void loop()
{
  // SET RGB LED VALUE
  if (digitalRead(switchPins[4])==HIGH) {
    valToWriteRed = int(map(encoderValue, 0, sampleSize, 0, 255));
    valToWriteGreen = int(map(encoderValue, sampleSize, 0, 0, 255));
    valToWriteBlue = int(map(encoderValue, sampleSize, 0, 30, 180));
    analogWrite(selectorRedPin, valToWriteRed);
    analogWrite(selectorGreenPin, valToWriteGreen);
    analogWrite(selectorBluePin, valToWriteBlue);
    Serial.print("valToWriteRed = ");
    Serial.print(valToWriteRed);
    Serial.print(" & valToWriteGreen = ");
    Serial.print(valToWriteGreen);
    Serial.print(" & valToWriteBlue = ");
    Serial.println(valToWriteBlue);
  } 
  else {
    analogWrite(selectorRedPin, 255);
    analogWrite(selectorGreenPin, 255);
    analogWrite(selectorBluePin, 255);
  }

  // determine on/off value of switches
  for (byte thisSwitch=0; thisSwitch < sizeof(switchPins); thisSwitch++) {
    boolean lastState = switchStates[thisSwitch];
    boolean newState = digitalRead(switchPins[thisSwitch]);


    if (lastState != newState) { // something has changed!
      flag = true;
      // turn on corresponding RGB LEDs
      toggleRGBLEDs(thisSwitch);
    }

    switchStates[thisSwitch] = newState;
  }


  // send data if there's a change
  if (flag) {
    Serial.print("S");
    for (byte thisSwitch=0; thisSwitch < sizeof(switchPins); thisSwitch++) {
      Serial.print(",");
      Serial.print(digitalRead(switchPins[thisSwitch]), DEC);

    }
    Serial.println(",");
    flag = false;
  } 

  encoderButton.Update();
  if (encoderButton.click != 0) buttonMode++;
  buttonMode = buttonMode%3;

  if (sampleSize > 0) {
    if (encoderValue > sampleSize) encoderValue = 0;
    if (encoderValue < 0) encoderValue = sampleSize;
  }
  //  if (sampleSize > 0 ) {
  if (lastencoderValue != encoderValue || encoderButton.click!=0) {
    Serial.print("R");
    Serial.print(",");
    Serial.print(buttonMode, DEC);
    Serial.print(",");
    Serial.print(encoderValue, DEC);
    Serial.println(",");
  }

  lastencoderValue = encoderValue;
  //  }



}

void toggleRGBLEDs(byte thisSwitch) {
  if (digitalRead(switchPins[thisSwitch])==HIGH && digitalRead(switchPins[4])==LOW && thisSwitch < 2) {
    Serial.print("valToWriteRed = ");
    Serial.print(valToWriteRed);
    Serial.print(" & valToWriteGreen = ");
    Serial.print(valToWriteGreen);
    Serial.print(" & valToWriteBlue = ");
    Serial.println(valToWriteBlue);
    analogWrite(switchPinsRed[thisSwitch], valToWriteBlue);
    analogWrite(switchPinsGreen[thisSwitch], valToWriteGreen);
    analogWrite(switchPinsBlue[thisSwitch], valToWriteRed);
  } 
  else if (digitalRead(switchPins[thisSwitch])==LOW && thisSwitch < 2) {
    analogWrite(switchPinsRed[thisSwitch], 255);
    analogWrite(switchPinsGreen[thisSwitch], 255);
    analogWrite(switchPinsBlue[thisSwitch], 255);
  }
}

//void serialEvent() {
//  while(Serial1.available()) {
//    sampleSize = (int)Serial1.read();
//    Serial.print("sample size = ");
//    Serial.println(sampleSize);
//  } 
//}


ISR(TIMER2_OVF_vect) {  
  /* Reload the timer */
  TCNT2 = tcnt2;  

  bouncer1.update();
  if(bouncer1.risingEdge()){
    if (digitalRead(encoderPin2))
      encoderValue++;
    else
      encoderValue--;
  }
}  






































