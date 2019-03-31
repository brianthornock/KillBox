#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// Assign pins
const unsigned int tapSwitch = 1;
const unsigned int patternSwitch = 2;
const unsigned int cv1 = 4;
const unsigned int cv2 = 3;
const unsigned int killSwitch = 0;
const unsigned int syncSwitch = 5;

//Tap Tempo Parameters
unsigned int defaultSampleTime; //Default rate on powerup
unsigned int sampleTime; //Sample period for kill rate
unsigned int prevSampleTime; //Previous time. Used for checking if we need to update sample period.
unsigned int tapTime; //Time between pulses
unsigned int prevTapTime; //Used for debouncing tap tempo switch
unsigned int prevTapDelay; //Used for debouncing tap tempo switch
unsigned int maxTaps = 10; //Maximum number of taps to count for tap tempo
unsigned int useTap; //Indicator of whether we have sufficient data to do tap tempo

unsigned int minTime; //Minimum time from tap pulse to pulse
unsigned int maxTime; //Maximum time from tap pulse to pulse
unsigned int tapTimeout; //Timeout for tap tempo

//Kill Switch Parameters
unsigned int killState = LOW; //Used for debouncing kill switch
unsigned int lastKillState = LOW; //Used for debouncing kill switch
unsigned int lastKillDebounceTime = 0; //Used for debouncing kill switch
unsigned int killDebounceDelay = 10; //Debounce time for kill switch
unsigned int manualKillDelay = 2000; //Press and hold for entering manual mode
unsigned int manualMode = 0; //Status of whether we are in manual mode or not
unsigned float killPulseRatio = 0.2; //This is the ratio that the killed pulse occupies
unsigned int kill = LOW; // Whether we have pressed the kill switch or not

//Tap Tempo Switch Parameters
unsigned int tapState = LOW; //State of the tap switch
unsigned int lastTapState = LOW; //Used for debouncing tap switch
unsigned int lastTapDebounceTime = 0; //Used for debouncing tap switch
unsigned int tapDebounceDelay = 10; //Debounce time for tap switch

//Pattern Parameters
unsigned int patternNumber; //Index for which patter we are using
unsigned int prevPatternNumber; //Index of previous pattern selection
unsigned int updatePatternNumber = LOW; //Indicator of whether we need to update pattern or not
unsigned int pattern[16]; //Array for keeping pattern definition
unsigned int numPatternSteps = 4; //How many steps a pattern has

//Sync Parameters
unsigned int syncMode = 1; //Mode 1 is dual mono, 0 is alternating stereo kills
unsigned int continuousMode = 0; //this will stutter in time as long as the kill switch is engaged

void setup() {
  //Define what each pin is
  pinMode(killSwitch, INPUT_PULLUP);
  pinMode(syncSwitch, INPUT_PULLUP);
  pinMode(tapSwitch, INPUT_PULLUP);
  pinMode(patternSwitch, INPUT_PULLUP);
  pinMode(cv1, OUTPUT);
  pinMode(cv2, OUTPUT);


  //Set up the initial state of the pins
  digitalWrite(killSwitch, LOW);
  digitalWrite(syncSwitch, LOW);
  digitalWrite(tapSwitch, LOW);
  digitalWrite(patternSwitch, LOW);
  digitalWrite(cv1, LOW);
  digitalWrite(cv2, LOW);

  defaultSampleTime = round(60 / 180 * 1000); //duration in ms of a 120 bpm default tempo
  sampleTime = round(60 / 180 * 1000); //set this to the default
  prevSampleTime = round(60 / 180 * 1000); //set this to the default
  tapTime = 333;
  prevTapTime = 333;
  prevTapDelay = 0;

  minTime = 50; // 1/maxFreq*1000 shortest allowable period between changes in voltage level
  maxTime = 1600; // round(1 / minFreq * 1000); // longest allowable period between changes in voltage level
  tapTimeout = 1500; // How long to keep waiting for taps

  patternNumber = 1;
  prevPatternNumber = 1;
}



void loop() {

  //Check to see if the tap tempo switch has been pressed
  debounceTapTempo();

  //Check to see if the kill switch has been pressed
  debounceKillSwitch();

  //If both the tap and kill switches are pressed at the same time, toggle continuous mode
  if (tapState == HIGH && kill == HIGH) {
    continuousMode = !continuousMode;
    if (continuousMode == 0) {
      //If we go out of continuousMode, we need to restore what manual mode was before we left
      manualMode = prevManualMode;
      prevManualMode = manualMode;
    }
    else {
      //If we go into continuous mode, set manual mode to 0, but don't change previous manual mode indicator
      manualMode = 0;
    }
  }//End if (tapState == HIGH && kill == HIGH)


  //Check to see if tap tempo is used
  checkTapTempo();


  //Check which preset is being used
  if (manualMode == 0) {
    if (continuousMode == 0) {
      checkPattern();
    }

    //Update the sample time based on above
    updateSampleTime();
  }


  //Generate Control voltage if the kill switch is pressed
  if (manualMode == 1) {
    killManualMode();
  } //End if (manualMode == 1)
  else if (continuousMode == 1) {
    killContinuousMode();
  } //End if (continuousMode == 1)
  else {
    //If we are in pattern mode then trigger the pattern if kill status went high
    if (kill == HIGH) {
      killPatternMode();
    }
  }// End pattern mode

}//End loop



void checkTapTempo() {

  if (tapStatus == HIGH) {

    tapStatus = LOW;
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
          //blinkDebugLED(1, 2);


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
      //blinkDebugLED(1,3);

    }
    else {
      //If we don't have the information to produce a tap tempo, stick with what we have
    }
    prevTapDelay = millis();
  }

}



void checkPattern() {
  vIn = analogRead(patternIn);

  if (vIn > 4.5) {
    //Preset 1 is 4 stutters in time
    patternNumber = 1;
    numPatternSteps = 4;
    if (syncMode == 1) {
      pattern = [3 3 3 3 0 0 0 0 0 0 0 0 0 0 0 0];
    }
    else {
      pattern = [1 2 1 2 0 0 0 0 0 0 0 0 0 0 0 0];
    }
  }
  elseif (vIn > 3.5 && vIn < 4.5) {
    // Preset 2 is 6 stutters in time
    patternNumber = 2;
    numPatternSteps = 6;
    if (syncMode == 1) {
      pattern = [3 3 3 3 3 3 0 0 0 0 0 0 0 0 0 0];
    }
    else {
      pattern = [1 2 1 2 1 2 0 0 0 0 0 0 0 0 0 0];
    }
  }
  elseif (vIn > 2.5 && vIn < 3.5) {
    //Preset 3 is 8 stutters in time
    patternNumber = 3;
    numPatternSteps = 8;
    if (syncMode == 1) {
      pattern = [3 3 3 3 3 3 3 3 0 0 0 0 0 0 0 0];
    }
    else {
      pattern = [1 2 1 2 1 2 1 2 0 0 0 0 0 0 0 0];
    }
  }
  elseif (vIn > 1.5 && vIn < 2.5) {
    //Preset 4 is two triplet stutters
    patternNumber = 4;
    numPatternSteps = 8;
    if (syncMode == 1) {
      pattern = [3 3 3 0 3 3 3 0 0 0 0 0 0 0 0 0];
    }
    else {
      pattern = [1 2 1 0 2 1 2 0 0 0 0 0 0 0 0 0];
    }
  }
  elseif (vIn > 1.5 && vIn < 0.5) {
    //Preset 5 is 6 stutters followed by two triplet stutters
    patternNumber = 5;
    numPatternSteps = 16;
    if (syncMode == 1) {
      pattern = [3 3 3 3 3 3 0 0 3 3 3 0 3 3 3 0];
    }
    else {
      pattern = [1 2 1 2 1 2 0 0 1 2 1 0 2 1 2 0];
    }
  }
  elseif (vIn < 0.5) {
    //Preset 6 is ??
    patternNumber = 6;
  }

  if (prevPatternNumber != patternNumber) {
    updatePatternNumber = 1;
    prevPatternNumber = patternNumber;
  }

}



void updateSampleTime() {

  //If we are more than 5 ms off and not using pattern, update the sampleTime
  if (abs(tapTime - prevTapTime) >= 5) {
    prevTapTime = tapTime;
    sampleTime = tapTime / 4; //We divide the tap into 4 for proper kill sequencing
  }

  killTime = sampleTime * killPulseRatio;
  liveTime = sampleTime - killTime;

}



//Code for debouncing tap tempo switch
void debounceTapTempo() {
  int reading = digitalRead(tapSwitch);

  if (reading != lastTapState) {
    lastTapDebounceTime = millis();
  }

  if ((millis() - lastTapDebounceTime) > manualKillDelay) {
    manualMode = !manualMode;
    prevManualMode = manualMode;
  }
  else  if ((millis() - lastTapDebounceTime) > tapDebounceDelay) {

    if (reading != tapState) {

      tapState = reading;

      if (tapState == HIGH) {
        tapStatus = HIGH;
      }
    }
  }

  lastTapState = reading;
}



void debounceKillSwitch() {

  int reading = digitalRead(killSwitch);

  if (reading != lastkillState) {
    lastKillDebounceTime = millis();
  }

  if ((millis() - lastKillDebounceTime) > killDebounceDelay) {

    if (reading != killState) {

      killState = reading;

      if (killState == HIGH) {
        kill = HIGH;
        //blinkDebugLED(1, 1);
      }
      else {
        kill = LOW;
      }
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
      delay(killTime);
      digitalWrite(cv1, LOW);
      digitalWrite(cv2, LOW);
      delay(liveTime);
      digitalWrite(cv1, LOW);
      digitalWrite(cv2, HIGH);
      delay(killTime);
      digitalWrite(cv1, LOW);
      digitalWrite(cv2, LOW);
      delay(liveTime);
    }
  }
}



//Code to produce the voltage
void killPatternMode() {

  for (i = 0; i < numPatternSteps; i++) {
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
    delay(killTime);
    digitalWrite(cv1, LOW);
    digitalWrite(cv2, LOW);
    delay(liveTime);
  }
}



void blinkLED(int numBlinks, int duration) {
  for (int i = 0; i < numBlinks; i++) {
    digitalWrite(ledOut, HIGH);
    delay(duration);
    digitalWrite(ledOut, LOW);
    delay(duration);
  }
}

void blinkDebugLED(int numBlinks, int duration) {
  for (int i = 0; i < numBlinks; i++) {
    digitalWrite(encSW, HIGH);
    delay(duration);
    digitalWrite(encSW, LOW);
    delay(duration);
  }
}
