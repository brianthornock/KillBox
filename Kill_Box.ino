#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// Assign pins
// Digital/PB Pin Assignments
const unsigned int tapSwitch = 4; //PB4/Pin3
const unsigned int patternSwitch = A1; //PB0/Pin5
const unsigned int cv1 = 3; //PB3/Pin2
const unsigned int cv2 = 0; //PB2/Pin7
const unsigned int killSwitch = 1; //PB1/Pin6
const unsigned int syncSwitchDigital = 5; //PB5/Pin1
// Analog input pin
const unsigned int syncSwitch = A0;

//Tap Tempo Parameters
unsigned int defaultSampleTime; //Default rate on powerup
unsigned int sampleTime; //Sample period for kill rate
unsigned int prevSampleTime; //Previous time. Used for checking if we need to update sample period.
unsigned int tapTime; //Time between pulses
unsigned int prevTapTime; //Used for debouncing tap tempo switch
unsigned int prevTapDelay; //Used for debouncing tap tempo switch
uint8_t maxTaps = 10; //Maximum number of taps to count for tap tempo
uint8_t useTap; //Indicator of whether we have sufficient data to do tap tempo
uint8_t prevTaps = 0;
unsigned int prevTimes [10];
uint8_t tapFlag = LOW;
uint8_t pollTap = 0; // Whether to poll tap tempo in the loop because the button was pressed
long downTime = -1;         // time the tap button was pressed down
long upTime = -1;           // time the tap button was released
boolean ignoreUp = false;   // whether to ignore the tap button release because the click+hold was triggered
unsigned int tapPressTimeout = 500;
uint8_t tapTriggered = 0; // Keep track of whether a certain button press has already triggered a tap tempo event

unsigned int minTime; //Minimum time from tap pulse to pulse
unsigned int maxTime; //Maximum time from tap pulse to pulse
unsigned int tapTimeout; //Timeout for tap tempo

//Kill Switch Parameters
uint8_t killState = LOW; //Used for debouncing kill switch
uint8_t lastKillState = LOW; //Used for debouncing kill switch
unsigned int lastKillDebounceTime = 0; //Used for debouncing kill switch
unsigned int killDebounceDelay = 20; //Debounce time for kill switch
unsigned int manualKillDelay = 2000; //Press and hold for this many ms before entering manual mode
uint8_t manualMode = 1; //Status of whether we are in manual mode or not
uint8_t prevManualMode = 0;
float killPulseRatio = 0.5; //This is the ratio that the killed pulse occupies
uint8_t kill = LOW; // Whether we have pressed the kill switch or not
uint8_t killFlag = LOW;

//Tap Tempo Switch Parameters
uint8_t tapState = LOW; //State of the tap switch
uint8_t lastTapState = LOW; //Used for debouncing tap switch
unsigned int lastTapDebounceTime = 0; //Used for debouncing tap switch
uint8_t tapDebounceDelay = 20; //Debounce time for tap switch

//Pattern Parameters
uint8_t patternNumber; //Index for which patter we are using
uint8_t prevPatternNumber; //Index of previous pattern selection
uint8_t updatePatternNumber = LOW; //Indicator of whether we need to update pattern or not
uint8_t pattern[16]; //Array for keeping pattern definition
uint8_t numPatternSteps = 4; //How many steps a pattern has
unsigned int liveTime;
unsigned int killTime;

//Sync Parameters
uint8_t syncMode = 1; //Mode 1 is dual mono, 0 is alternating stereo kills
uint8_t continuousMode = 0; //this will stutter in time as long as the kill switch is engaged

// UI Polling parameters
unsigned int checkInterval = 200; // How many ms between polling mode and pattern switches
unsigned int killCheckInterval = 50; // How many ms between polling kill switch to see if it is still being pressed
unsigned int lastInterval; // Time of last UI polling
unsigned int lastKillInterval; // Time of last kill switch polling

void setup() {
  //Define what each pin is
  pinMode(killSwitch, INPUT);
  pinMode(syncSwitchDigital, INPUT);
  pinMode(tapSwitch, INPUT_PULLUP);
  pinMode(patternSwitch, INPUT);
  pinMode(cv1, OUTPUT);
  pinMode(cv2, OUTPUT);


  //Set up the initial state of the pins
  digitalWrite(killSwitch, LOW);
  //digitalWrite(syncSwitch, LOW);
  digitalWrite(tapSwitch, LOW);
  digitalWrite(patternSwitch, LOW);
  digitalWrite(cv1, LOW);
  digitalWrite(cv2, LOW);

  defaultSampleTime = round(60.0 / 180 * 1000); //duration in ms of a 120 bpm default tempo
  sampleTime = round(60.0 / 180 * 1000); //set this to the default
  prevSampleTime = round(60.0 / 180 * 1000); //set this to the default
  tapTime = 333;
  prevTapTime = 333;
  prevTapDelay = 0;

  killTime = round(sampleTime * killPulseRatio);
  liveTime = sampleTime - killTime;

  minTime = 50; // shortest allowable period in ms between changes in voltage level
  maxTime = 2000; // longest allowable period in ms between changes in voltage level
  tapTimeout = round(1.5 * maxTime); // How long to keep waiting for taps

  prevPatternNumber = 0;
  checkPattern();
  lastInterval = millis();

  //Set up the pin change interrupts for the kill and tap switches
  GIMSK = 0b00100000; //Enable pin change interrupts
  PCMSK = 0b00010010; //Enable PCIE on PCINT1 and PCINT4 for kill and tap switches
  sei(); //Start interrupt service
}



void loop() {

  if (killFlag == HIGH) {
    //Check to see if the kill switch has been pressed
    debounceKillSwitch();
    killFlag = LOW;
  }

  if (tapFlag == HIGH) {
    //Check to see if the tap switch has been pressed
    debounceTapTempo();
    pollTap = 1; // Check in on the switch at least once if it has been pressed
    tapFlag = LOW; // Reset this flag until the button gets pressed again
  }


   // Check the kill switch every killCheckInterval ms to see if it is still being pressed
  if (millis() - lastKillInterval > killCheckInterval) {
    debounceKillSwitch(); // Check the kill switch to see if it is being held or not

    if (pollTap == 1) {
      debounceTapTempo();
    }
    
    lastKillInterval = millis();
  }

  
  //If both the tap and kill switches are pressed at the same time, toggle continuous mode
  if (tapState == HIGH && kill == HIGH) {
    blinkLED(2,50); // Indicate to the user that we have changed modes successfully
    if (continuousMode == 1) {
      //If we go out of continuousMode, we need to restore what manual mode was before we left
      manualMode = prevManualMode;
      continuousMode = 0;
    }
    else {
      //If we go into continuous mode, buffer what manual mode was prior to going in and set manual mode to 0
      prevManualMode = manualMode;
      manualMode = 0;
      continuousMode = 1;
    }
  }//End if (tapState == HIGH && kill == HIGH)

 
  // Check the sync mode switch every checkInterval ms
  if (millis() - lastInterval > checkInterval) {
    syncMode = analogRead(syncSwitch) > 800; // Check the sync switch. Since this is on the reset pin, it's an analog read.

    //Check which preset is being used
    checkPattern();

    lastInterval = millis();
  }


  //Check tap tempo and adjust the tempo if needed
  if (tapState == HIGH) {
    //Check to see if tap tempo is used
    checkTapTempo();
    // If our tap time is super short, it's likely due to pulses while waiting for press/hold of tap button, so ignore
    if (tapTime > minTime){
      //Update the sample time based on above
      updateSampleTime();
    }
  }


  //Generate Control voltage if the kill switch is pressed
  if ((manualMode == 1) && (continuousMode == 0)) {
    killManualMode();
  } //End if (manualMode == 1)
  else if ((continuousMode == 1) && (manualMode == 0)) {
    killContinuousMode();
  } //End if (continuousMode == 1)
  else if (manualMode == 0 && continuousMode == 0) {
    //If we are in pattern mode then trigger the pattern if kill status went high
    if (kill == HIGH) {
      killPatternMode();
    }
  }// End pattern mode

}//End loop






//Interrupt handling
ISR (PCINT0_vect) {

  //Read the switch values
  uint8_t killRead1 = digitalRead(killSwitch);
  uint8_t tapRead1 = digitalRead(tapSwitch);

  delayMicroseconds(2000); //Brute force debouncing
  uint8_t killRead2 = digitalRead(killSwitch);
  uint8_t tapRead2 = digitalRead(tapSwitch);

  // The kill switch was either pressed or released, so check the switch state
  if (((killRead1 == HIGH) && (killRead2 == HIGH)) || ((killRead1 == LOW) && (killRead2 == LOW))) {
    killFlag = HIGH;
  }


  if ((tapRead1 == HIGH) && (tapRead2 == HIGH)) {
    tapFlag = HIGH;
  }
  // If the tap switch was just released, we don't want to do anything
  else if ((tapRead1 == LOW) && (tapRead2 == LOW)) {
    tapFlag = LOW;
  }
  
}



void checkTapTempo() {

  if (tapState == HIGH) {

    tapState = LOW;
    //Check to see if we already have a tap tempo history. If so, add this to
    //the history. If not, start a new count.
    if (prevTaps > 0) {
      int currTime = millis();
      int currDelay = currTime - prevTapDelay;
      // Check to make sure we didn't time out
      if (currDelay < tapTimeout) {
        //Set the flag for using tap tempo
        useTap = 1;

        // Create the temp array for storing times in
        unsigned int newPrevTimes [maxTaps];

        if (prevTaps < maxTaps) {

          //Fill up the new array with all the old values first
          for (int k = 0; k < prevTaps - 1; k++) {
            newPrevTimes[k] = prevTimes[k];
          }

          //Then add in the new value at the end
          newPrevTimes[prevTaps - 1] = currDelay;
          prevTaps++;

        } // End if prevTaps < maxTaps

        for (int nTime = 0; nTime < maxTaps; nTime++) {
          prevTimes[nTime] = newPrevTimes[nTime];
        }

      } // End if currDelay < tapTimeout
      else {
        //If we timeout, reset the counter and zero out the tempo array
        prevTaps = 1;

        for (int i = 0; i < maxTaps; i++) {
          prevTimes[i] = 0;
        }

        useTap = 0;
      } // End if tap has timed out
    } // End if prevTaps > 0
    // If we do not have any previous taps (first tap after timeout)
    else {
      prevTaps = 1;

      for (int i = 0; i < maxTaps; i++) {
        prevTimes[i] = 0;
      }

      useTap = 0;
    }

    if (useTap == 1 && prevTaps > 2) {
      //Calculate the average polling time, including the multiplier and the random switch
      int sum, loop, numVals;
      float avg;

      sum = avg = 0;
      numVals = 0;

      for (loop = 0; loop < prevTaps - 1; loop++) {
        if (prevTimes[loop] != 0) {
          sum += prevTimes[loop];
          numVals++;

          //digitalWrite(encSW, HIGH);
          //delayMicroseconds(prevTimes[loop]);
          //digitalWrite(encSW, LOW);
        }
      }
      avg = (float)sum / numVals;
      tapTime = round(avg);
    }
    else {
      //If we don't have the information to produce a tap tempo, stick with what we have
    }
    prevTapDelay = millis();
  }

}



void checkPattern() {
  unsigned int vIn = analogRead(patternSwitch);
  
  if (vIn > 925) {
    
    //Preset 1 is 4 stutters in time
    patternNumber = 1;
    if (syncMode == 1) { // Dual Mono Kills
      numPatternSteps = 8;
      //pattern = [3 3 3 3 0 0 0 0 0 0 0 0 0 0 0 0];
      pattern[0] = 3;
      pattern[1] = 0;
      pattern[2] = 3;
      pattern[3] = 0;
      pattern[4] = 3;
      pattern[5] = 0;
      pattern[6] = 3;
      pattern[7] = 0;
      pattern[8] = 0;
      pattern[9] = 0;
      pattern[10] = 0;
      pattern[11] = 0;
      pattern[12] = 0;
      pattern[13] = 0;
      pattern[14] = 0;
      pattern[15] = 0;
      //return;
    }
    else { // Stereo kills
      numPatternSteps = 4;
      //pattern = [1, 2, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
      pattern[0] = 1;
      pattern[1] = 2;
      pattern[2] = 1;
      pattern[3] = 2;
      pattern[4] = 0;
      pattern[5] = 0;
      pattern[6] = 0;
      pattern[7] = 0;
      pattern[8] = 0;
      pattern[9] = 0;
      pattern[10] = 0;
      pattern[11] = 0;
      pattern[12] = 0;
      pattern[13] = 0;
      pattern[14] = 0;
      pattern[15] = 0;
      //return;
    }
  }
  else if (vIn > 725 && vIn < 925) {
    // Preset 2 is 6 stutters in time
    patternNumber = 2;
    if (syncMode == 1) {
      numPatternSteps = 12;
      //pattern = [3 3 3 3 3 3 0 0 0 0 0 0 0 0 0 0];
      pattern[0] = 3;
      pattern[1] = 0;
      pattern[2] = 3;
      pattern[3] = 0;
      pattern[4] = 3;
      pattern[5] = 0;
      pattern[6] = 3;
      pattern[7] = 0;
      pattern[8] = 3;
      pattern[9] = 0;
      pattern[10] = 3;
      pattern[11] = 0;
      pattern[12] = 0;
      pattern[13] = 0;
      pattern[14] = 0;
      pattern[15] = 0;
      //return;
    }
    else {
      numPatternSteps = 6;
      //pattern = [1 2 1 2 1 2 0 0 0 0 0 0 0 0 0 0];
      pattern[0] = 1;
      pattern[1] = 2;
      pattern[2] = 1;
      pattern[3] = 2;
      pattern[4] = 1;
      pattern[5] = 2;
      pattern[6] = 0;
      pattern[7] = 0;
      pattern[8] = 0;
      pattern[9] = 0;
      pattern[10] = 0;
      pattern[11] = 0;
      pattern[12] = 0;
      pattern[13] = 0;
      pattern[14] = 0;
      pattern[15] = 0;
      //return;
    }
  }
  else if (vIn > 512 && vIn < 720) {
    //Preset 3 is 8 stutters in time
    patternNumber = 3;
    
    if (syncMode == 1) {
      numPatternSteps = 16;
      //pattern = [3 3 3 3 3 3 3 3 0 0 0 0 0 0 0 0];
      pattern[0] = 3;
      pattern[1] = 0;
      pattern[2] = 3;
      pattern[3] = 0;
      pattern[4] = 3;
      pattern[5] = 0;
      pattern[6] = 3;
      pattern[7] = 0;
      pattern[8] = 3;
      pattern[9] = 0;
      pattern[10] = 3;
      pattern[11] = 0;
      pattern[12] = 3;
      pattern[13] = 0;
      pattern[14] = 3;
      pattern[15] = 0;
      //return;
    }
    else {
      numPatternSteps = 8;
      //pattern = [1 2 1 2 1 2 1 2 0 0 0 0 0 0 0 0];
      pattern[0] = 1;
      pattern[1] = 2;
      pattern[2] = 1;
      pattern[3] = 2;
      pattern[4] = 1;
      pattern[5] = 2;
      pattern[6] = 1;
      pattern[7] = 2;
      pattern[8] = 0;
      pattern[9] = 0;
      pattern[10] = 0;
      pattern[11] = 0;
      pattern[12] = 0;
      pattern[13] = 0;
      pattern[14] = 0;
      pattern[15] = 0;
      //return;
    }
  }
  else if (vIn > 310 && vIn < 510) {
    //Preset 4 is two triplet stutters
    patternNumber = 4;
    if (syncMode == 1) {
      numPatternSteps = 16;
      //pattern = [3 3 3 0 3 3 3 0 0 0 0 0 0 0 0 0];
      pattern[0] = 3;
      pattern[1] = 0;
      pattern[2] = 3;
      pattern[3] = 0;
      pattern[4] = 3;
      pattern[5] = 0;
      pattern[6] = 0;
      pattern[7] = 0;
      pattern[8] = 3;
      pattern[9] = 0;
      pattern[10] = 3;
      pattern[11] = 0;
      pattern[12] = 3;
      pattern[13] = 0;
      pattern[14] = 0;
      pattern[15] = 0;
      //return;
    }
    else {
      numPatternSteps = 8;
      //pattern = [1 2 1 0 2 1 2 0 0 0 0 0 0 0 0 0];
      pattern[0] = 1;
      pattern[1] = 2;
      pattern[2] = 1;
      pattern[3] = 0;
      pattern[4] = 2;
      pattern[5] = 1;
      pattern[6] = 2;
      pattern[7] = 0;
      pattern[8] = 0;
      pattern[9] = 0;
      pattern[10] = 0;
      pattern[11] = 0;
      pattern[12] = 0;
      pattern[13] = 0;
      pattern[14] = 0;
      pattern[15] = 0;
      //return;
    }
  }
  else if (vIn > 100 && vIn < 300) {
    //Preset 5 is 6 stutters followed by two triplet stutters
    patternNumber = 5;
    numPatternSteps = 16;
    if (syncMode == 1) {
      //pattern = [3 3 3 3 3 3 0 0 3 3 3 0 3 3 3 0];
      pattern[0] = 3;
      pattern[1] = 0;
      pattern[2] = 3;
      pattern[3] = 0;
      pattern[4] = 3;
      pattern[5] = 0;
      pattern[6] = 0;
      pattern[7] = 0;
      pattern[8] = 3;
      pattern[9] = 0;
      pattern[10] = 3;
      pattern[11] = 0;
      pattern[12] = 3;
      pattern[13] = 0;
      pattern[14] = 0;
      pattern[15] = 0;
      //return;
    }
    else {
      //pattern = [1 2 1 2 1 2 0 0 1 2 1 0 2 1 2 0];
      pattern[0] = 1;
      pattern[1] = 2;
      pattern[2] = 0;
      pattern[3] = 0;
      pattern[4] = 1;
      pattern[5] = 2;
      pattern[6] = 0;
      pattern[7] = 0;
      pattern[8] = 1;
      pattern[9] = 2;
      pattern[10] = 1;
      pattern[11] = 0;
      pattern[12] = 2;
      pattern[13] = 1;
      pattern[14] = 2;
      pattern[15] = 0;
      //return;
    }
  }
  else if (vIn < 100) {
    //Preset 6 is ??
    patternNumber = 6;
    numPatternSteps = 16;
    pattern[0] = 3;
    pattern[1] = 0;
    pattern[2] = 3;
    pattern[3] = 0;
    pattern[4] = 0;
    pattern[5] = 0;
    pattern[6] = 0;
    pattern[7] = 0;
    pattern[8] = 3;
    pattern[9] = 0;
    pattern[10] = 3;
    pattern[11] = 0;
    pattern[12] = 0;
    pattern[13] = 0;
    pattern[14] = 0;
    pattern[15] = 0;
    //return;
  }

  if (patternNumber != prevPatternNumber) {
    blinkLED(1,100);
    prevPatternNumber = patternNumber;
  }

}



void updateSampleTime() {

  //If we are more than 5 ms off and not using pattern, update the sampleTime
  if ((useTap) && (abs(tapTime - prevTapTime) >= 10)) {
    prevTapTime = tapTime;
    sampleTime = tapTime / 4; //We divide the tap into 4 for proper kill sequencing

    killTime = round(sampleTime * killPulseRatio);
    liveTime = sampleTime - killTime;
  }
} // End updateSampleTime()



//Code for debouncing tap tempo switch
void debounceTapTempo() {
  int reading = digitalRead(tapSwitch);
  int reading2 = 2;

  // Button is pressed down
  if (reading == HIGH && lastTapState == LOW){
    downTime = millis();
  }

  // Check to see if we have met the press/hold threshold
  if (reading == HIGH && lastTapState == HIGH && (millis() - downTime > manualKillDelay)) {
    pollTap = 0; // Don't poll in the main thread until we have another button press triggered by interrupt
    tapState = LOW; // This condition doesn't count as a regular button press, so set the state LOW
    lastTapState = LOW; // Artificially set this back to LOW so that subsequent presses don't retrigger this condition since we turned off tap polling
    tapTriggered = 0;
    blinkLED(1,100); // Blink LED to confirm to user that mode was changed
    // We have met the press/hold threshold, so we change the manual mode
    if (manualMode == 1) {
      prevManualMode = 1;
      manualMode = 0;
    }
    else {
      prevManualMode = 0;
      manualMode = 1;
    }  
  }
  // If we haven't met the press/hold threshold, see if we have met the button press threshold
  else if (reading == HIGH && lastTapState == HIGH && (millis() - downTime > tapDebounceDelay) && (millis() - downTime < tapPressTimeout) && (tapTriggered == 0)) {
    // If the tap button has been held down for more than a certain amount of time, stop declaring a HIGH tap state
    tapState = HIGH;
    tapTriggered = 1;
    lastTapState = reading;
  }
  else {
    tapTriggered = 0;
    lastTapState = reading;
  }
}



void debounceKillSwitch() {

  int reading = digitalRead(killSwitch);
  int reading2 = 2;

  if (reading != lastKillState) {
    delay(killDebounceDelay);
    reading2 = digitalRead(killSwitch);
  }

  if (reading == reading2) {

    killState = reading;

    if (killState == HIGH) {
      kill = HIGH;
    }
    else {
      kill = LOW;
    } 
  }

  lastKillState = reading;
}



void killManualMode() {
  //If we are in manual mode then the control voltage is the same as the kill switch state and sync mode is ignored
  digitalWrite(cv1, kill);
  digitalWrite(cv2, kill);
}


void killContinuousMode() {
  //If we are in continuous mode, we need to just kill to the tempo as long as the kill switch is pressed
  if (kill == HIGH) {
    if (syncMode == 1) {
      //If we are in dual mono, kill both simultaneously
      digitalWrite(cv1, HIGH);
      digitalWrite(cv2, HIGH);
      delay(killTime);
      digitalWrite(cv1, LOW);
      digitalWrite(cv2, LOW);
      delay(liveTime);
    }
    else {
      //If we are in stereo mode, kill the two channels alternatingly
      digitalWrite(cv1, HIGH);
      digitalWrite(cv2, LOW);
      delay(round(killTime/2));
      digitalWrite(cv1, LOW);
      digitalWrite(cv2, LOW);
      delay(round(liveTime/2));
      digitalWrite(cv1, LOW);
      digitalWrite(cv2, HIGH);
      delay(round(killTime/2));
      digitalWrite(cv1, LOW);
      digitalWrite(cv2, LOW);
      delay(round(liveTime/2));
    }
  }
}



//Code to produce the voltage
void killPatternMode() {

  for (int i = 0; i < numPatternSteps; i++) {
    if (pattern[i] == 1) {
      //1 means we kill the tip signal, which is cv1
      digitalWrite(cv1, HIGH);
      digitalWrite(cv2, LOW);
    }
    else if (pattern[i] == 2) {
      //A 2 means we cut the ring signal, which is cv2
      digitalWrite(cv1, LOW);
      digitalWrite(cv2, HIGH);
    }
    else if (pattern[i] == 3) {
      //A three means that we cut both sides, so write the control voltage high
      digitalWrite(cv1, HIGH);
      digitalWrite(cv2, HIGH);
    }
    else {
      //If we have a zero, write both control voltages to low to pass signal
      digitalWrite(cv1, LOW);
      digitalWrite(cv2, LOW);
    }

    //Kill the signal for the kill time, then return both signals to live for the live time
    if (pattern[i] == 3) {
      delay(round(killTime/2));
      digitalWrite(cv1, LOW);
      digitalWrite(cv2, LOW);
      delay(round(liveTime/2));
    }
    else {
      delay(killTime);
      digitalWrite(cv1, LOW);
      digitalWrite(cv2, LOW);
      delay(liveTime);
    }
  }
}



void blinkLED(int numBlinks, int duration) {
  for (int i = 0; i < numBlinks; i++) {
    digitalWrite(cv1, HIGH);
    delay(duration);
    digitalWrite(cv1, LOW);
    delay(duration);
  }
}

void blinkDebugLED(int numBlinks, int duration) {
  for (int i = 0; i < numBlinks; i++) {
    digitalWrite(cv2, HIGH);
    delay(duration);
    digitalWrite(cv2, LOW);
    delay(duration);
  }
}
