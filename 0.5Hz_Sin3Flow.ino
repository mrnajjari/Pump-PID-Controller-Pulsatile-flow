// This is the PID controller program for Arduino micro-controller that controls a pump to produce a periodic signal consist of 3 sin waves as a reference signal. The pump is designed to accept 0-5 V signal for setting the velocity.
// Because the reference signal varies a lot, using gain scheduling is critical. Gains must be tuned for each new setup and applications.
// Copyright (C) <20017>  This code was developed by Mohammad Reza Najjari (PhD candidate) at George Washington University, Department of Mechanical and Aerospace Engineering.
// Email: mnajjari@gwu.edu.
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU Affero General Public License as published
//    by the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU Affero General Public License for more details.

// ____________________________________________________________________________________________________________________________________________________________________________________________________________________

float v, v_prev = 0.0;                                // This is the feedback signal from the flowmeter; It is being read through the A0 pin. Flowmeter provides 0-5 V signal corresponding to 0-5 Lit/min.
float vd, vd_prev = 0.0;                              // This is the Desired (Reference) Signal.
float v_out;                                          // This is the output of controller as voltage sent to the pump.
float ex, ex_prev = 0.0, ei, ei_prev = 0.0 , ed,  dt; // Different terms in PID controller.

//--- PID gains ____\\ This is the initial gain to prevent high overshooting in startup. But they will be defined in the gain scheduling section again.
float kp = 5.0 / 10000.;
float ki = 0.0 / 1000000.;
float kd = 00.0 / 1000.;

int J = 0, C1 = 0;	\\ Integers to control the loop

const int Is_Gain_Schedule = 1;     // if ==0 then NO gain scheduling, if ==1 then gain scheduling is activeated.

// --- Reference signal (Desired signal) parameters:
const int Ref_Sig_T = 2000; 											// This is the period of the Reference signal in mili s
const int num_Step_Ahead = 34; 											// This is the number of points that the PID controller is looking ahead in reference signal to compensate the delay
const float Time_Step = 1.5; 											// This is the number of the elements in the Reference signal vector (This can not be gratter than about 1600 because the memory is not enough)
const int Num_J = int(Ref_Sig_T / Time_Step) + num_Step_Ahead; 			// This is the number of the elements in the Reference signal vector
float GenVd[Num_J];														// Reference signal vector

unsigned long t, t_prev = 0, T1 = 0;    								// t is the actual (real time) of the CPU in mili second or micro second. t_prev is the previous iteration's time. The code converts the "t" to "T1" which is the corresponding element number in reference signal.

unsigned int T = 0;   													// @@@ This is also time like "t" but for Loop speed Check. The serial print for speed check is disabled by default to not slow down the loop.



void setup() {
  pinMode(10, OUTPUT);   // sets the pin as output which sends the PWM type signals to the pump from the PID controller output as an voltage. PWM can either be 0 or 5 v with different duty cycles. So the this output before sending to pump was filtered using an active 2nd order low-pass filter with diode to protect the pump from voltages over 4.2 volts.
  pinMode(11, OUTPUT);   // sets the pin as output. This the reference signal sent to scope for observation. This is also filtered using a simple RC low-pass filter.
  pinMode(50, OUTPUT);   // sets the pin as output. This the Digital TTL trigger sent at T=0 at PIN=50 on Arduino

  //Serial.begin(9600);
  Serial.begin(115200);
  analogWrite(10, 0.0 / 5.0 * 255 );

  //------ Reference signal generation loop. --------------------------------------------------
  for (J = 0; J <= Num_J * Time_Step; J = J + Time_Step) {
    GenVd[int(J / Time_Step)] = 1.75 +  .5 * sin(1 * 2.0 *  PI * J / Ref_Sig_T ) + .5 * sin(2 * 2.0 *  PI * J / Ref_Sig_T ) + .4 * sin(3 * 2.0 *  PI * J / Ref_Sig_T );

  }

  delay(1000);  // Protective delay to prevent pump maxout voltage at start up needs to be modified based on the processing time of "Void Setup"
}

void loop() {

  //t = millis();     // the real time CPU time in mili s
  T = micros();       // @@@Loop speed Check
  t = micros();       //the real time CPU time in micro s. This provides microseconds since the Arduino board began running This number will overflow (go back to zero), after approximately 70 minutes. On 16 MHz Arduino boards (e.g. Duemilanove and Nano), this function has a resolution of four microseconds
  dt = t - t_prev;
  dt = dt / 1000;     // to convert the dt in (micro s) to (mili s) if in above line the "t = micros();" has been activated
  //Serial.println(dt);

  T1 = (unsigned long int)(t / (Time_Step * 1000)) % int(Ref_Sig_T / Time_Step); 		// Very important command line. This is converting the realtime in CPU ("t") to the element index in the Reference signal. Time_Step multiplied to 1000 to convert it to micro second because "t" is in micro s.
  //Serial.println(T1);



  //--------------- PID gain Scheduling. -------------------------
  // Where each new sets of gain starts is very critical.

  if (Is_Gain_Schedule == 1) {

    if (T1 >= 0 && T1 <= 20) {				// To generate a Digital 5 V pulse at Pin 50 for synchronization applications or triggering other devices.
      digitalWrite(50, HIGH);
    }    else {
      digitalWrite(50, LOW);
    }

    if (T1 == 0 && C1 <= 2) {
      ei_prev = 0;    						// resets the integral term error to prevent "windup"
      C1 = C1 + 1;
    }

    if (T1 == 0) {
      kp = 65.0 / 10000;
      ki = 65. / 1000000.0;
      kd = 300.0 / 10000;
    }

    if (T1 >= 0 && T1 <= 160) {   //235
      //kp = kp + 0.09 / 10000;
      ki = ki + 0.005 / 1000000.0;
    }
    else if (T1 > 160 && T1 <= 300) {
      ki = ki - 0.1 / 1000000.0;
    }
    else if (T1 > 760 && T1 <= 1180) {
      //kp = 50.0 / 10000;
      ki = ki - 0.018 / 1000000.0;

    }
    if (T1 > 1180 && T1 <= 1300) {
      //ki = ki + 0.154 / 1000000.0;
      ki = ki + 0.000026000*sq(T1-1180) / 1000000.0;    }
  }

  //-- No gain scheduling
  if (Is_Gain_Schedule == 0) {
    if (T1 == 0 && C1 == 0) {
      ei_prev = 0;    				// resets the integral term error to prevent "windup"
      C1 = 1;

    }

    if (T1 >= 0 && T1 <= 20) {		// To generate a Digital 5 V pulse at Pin 50 for synchronization applications or triggering other devices.
      digitalWrite(50, HIGH);
    }    else {
      digitalWrite(50, LOW);
    }


    kp = 58.0 / 10000.0;                  //70
    ki = 59.0 / 1000000.0;                //45
    kd = 70.0 / 10000.0;                  //40
    //    if (Serial.available()) {
    //      C1 = 0;
    //      kp = Serial.parseFloat() / 10000.0;
    //      Serial.println(kp * 10000.0);
    //      ki = Serial.parseFloat() / 1000000.0;
    //      Serial.println(ki * 1000000.0);
    //      kd = Serial.parseFloat() / 10000.0;
    //      Serial.println(kd * 10000.0);
    //    }
  }

  v = analogRead(A0);                             	// Flowrate feedback input
  analogWrite(11, GenVd[T1] / 5.0 * 255 );        	// Reference signal to Scope for observation. "/ 5.0 * 255" should be used for correct analog values.

  //vd = analogRead(A1);                          	// Desired signal IF IT IS BEING GENERATED FROM OUTSIDE SOURCE LIKE signal generator OR  LABVIEW & DAQ
  vd = GenVd[T1 + num_Step_Ahead] / 5.0 * 1024;   	// Desired signal generated above but due to Analog/digital conversion it must be scaled to 1024 for this Arduino board. --> 5V=1024

  ex = vd  - v;                                 	// Proportional term
  ed = ( (vd - vd_prev) - (v - v_prev)) / dt;   	// Derivative term (1st order forward finite difference)
  ei = ei_prev + ex * dt;                     		// Integral term (rectangle)
  //ei = ei_prev + (ex + ex_prev) / 2.0 * dt;     	// Integral term (Trapezoidal)
  v_out = kp * ex + kd * ed + ki * ei;        		// PID
  //Serial.println(v_out);

  //-- Safety measures for protecting the pump from over/under voltage  --------------
  if (v_out > 4.6) {
    v_out = 4.6;
  }
  if (v_out < 0 ) {
    v_out = 0;
  }
  //------------------------------------------------------------------------------

//v_out = 0.0;
  analogWrite(10, v_out / 5.0 * 255 );
 
 //-------- OUT PUT data to serial print -----------  THIS SLOWS DOWN THE LOOP IF ENABLED SO t1 SHOULD BE MONITORED AND Delay SHOULD BE REDUCED TO ABOUT 20
 
//Serial.print(T1/1332.*2.,3); 		// Serial.print does not go to NEXT LINE but Serial.println goes to NEXT LINE
//Serial.print("\t");
//Serial.print(GenVd[T1], 4);
//Serial.print("\t");
//Serial.print(T1 / 1332.*3.999, 3);
//Serial.print("\t");
//Serial.println(v * 5 / 1024., 4);
//if (T1 == 0 || T1==1) {
  //Serial.println("Begin  t  Ref  t  Flow ");
  //Serial.println("");
//}



  t_prev = t;
  ex_prev = ex;
  ei_prev = ei;
  vd_prev = vd;
  v_prev = v;

  //delay(2);					// This is the Delay in Mili second which is so much for these applications. Instead delayMicroseconds is used.
  delayMicroseconds(750); 		//The minimum for this number is 4 (Because the resolution of 16MHz board is 4 mic s) BUT for stability it is better to be above 1000. OR activate the delay in line above which is in mili seconds for slower applications. It is important that "T" (in mili second) be smaller than "Time_Step" (in mili second).
  //T = micros() - T;       	// @@@Loop speed Check. Shows the Loop running time in "micro s".
 // Serial.println(T, DEC); 	// @@@Loop speed Check

}
