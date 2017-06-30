// This is the PID controller program for Arduino micro-controller that controls a pump to produce the Physiological reference signal. The pump is designed to accept 0-5 V signal for setting the velocity.
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
float L2_Error = 0.0;

//--- PID gains ____\\ This is the initial gain to prevent high overshooting in startup. But they will be defined in the gain scheduling section again.
float kp = 10.0 / 10000.;
float ki = 2.0 / 1000000.;
float kd = 00.0 / 1000.;

int J = 0, C1 = 1;	\\ Integers to control the loop

// ----- Reference signal (Desired signal) parameters: -------------------------------------------------------
const int Ref_Sig_T = 4000; 											// This is the period of the Reference signal in mili s
const int num_Step_Ahead = 14; 											// This is the number of points that the PID controller is looking ahead in reference signal to compensate the delay
const int Time_Step = 3; 												// The Resolution of constructed Reference signial vector in (ms). It is very important that the Loop running speed be fast enough so that dt is smaller than this value. 
const int Num_J = int(Ref_Sig_T / Time_Step) + num_Step_Ahead; 			// This is the number of the elements in the Reference signal vector (This can not be gratter than about 1600 because the memory is not enough)
float GenVd[Num_J];														// Reference signal vector

unsigned long t, t_prev = 0, T1 = 0;    								// t is the actual (real time) of the CPU in mili second or micro second. t_prev is the previous itteration's time. The code converts the "t" to "T1" which is the corresponding element number in reference signal.

unsigned int T = 0;   													// @@@ This is also time like "t" but for Loop speed Check. The serial print for speed check is disabled by default to not slow down the loop.



void setup() {
  pinMode(10, OUTPUT);   		// sets the pin as output which sends the PWM type signals to the pump from the PID controller output as an voltage. PWM can either be 0 or 5 v with different duty cycles. So the this output before sending to pump was filtered using an active 2nd order low-pass filter with diode to protect the pump from voltages over 4.2 volts.
  pinMode(11, OUTPUT);  	 	// sets the pin as output. This the reference signal sent to scope for observation. This is also filtered using a simple RC low-pass filter.
  pinMode(50, OUTPUT);   		// sets the pin as output. This the Digital TTL triger sent at T=0 at PIN=50 on Arduino

  //Serial.begin(9600);
  Serial.begin(115200);
  analogWrite(10, 0.0 / 5.0 * 255 );  // Not writing anything while doing process here in setup

  //------ Reference signal generation loop. A Fourier series was used in matlab to decompose the signal in to Sin & Cos. --------------------------------------------------
  for (J = 0; J <= Num_J * Time_Step; J = J + Time_Step) {
    GenVd[int(J / Time_Step)] = 1.0 * (0.31562913436724243654296628847078 * cos(4.7123889803846898576939650749193 * J / 1000 - 9.4247779607693797153879301498385) - 0.34729779144587291650481120086624 * cos(3.1415926535897932384626433832795 * J / 1000 - 6.283185307179586476925286766559) - 0.1099814231898588040570174939603 * cos(1.5707963267948966192313216916398 * J / 1000 - 3.1415926535897932384626433832795) - 0.092310238864104235401164544327912 * cos(6.283185307179586476925286766559 * J / 1000 - 12.566370614359172953850573533118) - 0.29074963100722561737399018966244 * cos(7.8539816339744830961566084581988 * J / 1000 - 15.707963267948966192313216916398) + 0.14227372323122494668368176462536 * cos(9.4247779607693797153879301498385 * J / 1000 - 18.849555921538759430775860299677) + 0.029957663293548007776845309990676 * cos(10.995574287564276334619251841478 * J / 1000 - 21.991148575128552669238503682957) - 0.037322795400047813207944358282475 * cos(12.566370614359172953850573533118 * J / 1000 - 25.132741228718345907701147066236) + 0.022076717474907948945972435694785 * cos(14.137166941154069573081895224758 * J / 1000 - 28.274333882308139146163790449516) + 0.012087093071536098368157219340446 * cos(15.707963267948966192313216916398 * J / 1000 - 31.415926535897932384626433832795) - 0.012777018279875838879799943015314 * cos(17.278759594743862811544538608037 * J / 1000 - 34.557519189487725623089077216075) + 0.010468148759050143956073064543943 * cos(18.849555921538759430775860299677 * J / 1000 - 37.699111843077518861551720599354) + 0.020489009147817639072686191070716 * cos(20.420352248333656050007181991317 * J / 1000 - 40.840704496667312100014363982634) - 0.0066731056763570323689394037103284 * cos(21.991148575128552669238503682957 * J / 1000 - 43.982297150257105338477007365913) - 0.0079231670471533715421896815200853 * cos(23.561944901923449288469825374596 * J / 1000 - 47.123889803846898576939650749193) + 0.0076854660495851833540426056856631 * cos(25.132741228718345907701147066236 * J / 1000 - 50.265482457436691815402294132472) + 0.0029137833279751982999694615017461 * cos(26.703537555513242526932468757876 * J / 1000 - 53.407075111026485053864937515752) - 0.0031623942176620110401652485876411 * cos(28.274333882308139146163790449516 * J / 1000 - 56.548667764616278292327580899031) - 0.0068889651438828296281524643518424 * cos(29.845130209103035765395112141155 * J / 1000 - 59.690260418206071530790224282311) - 0.0027065592411393505495320610521048 * cos(31.415926535897932384626433832795 * J / 1000 - 62.83185307179586476925286766559) + 0.0027654591437073689809411636275627 * cos(32.986722862692829003857755524435 * J / 1000 - 65.97344572538565800771551104887) - 0.00030378857682657330833292386174094 * cos(34.557519189487725623089077216075 * J / 1000 - 69.115038378975451246178154432149) + 0.0027207811875047975200270222728705 * cos(36.128315516282622242320398907714 * J / 1000 - 72.256631032565244484640797815429) + 0.00050376194667813023204266942300933 * cos(37.699111843077518861551720599354 * J / 1000 - 75.398223686155037723103441198708) - 0.0026664111091364074181131460505867 * cos(39.269908169872415480783042290994 * J / 1000 - 78.539816339744830961566084581988) - 0.0014224023094127622498378249460416 * cos(40.840704496667312100014363982634 * J / 1000 - 81.681408993334624200028727965267) + 0.00051532942915192745357566561636986 * cos(42.411500823462208719245685674273 * J / 1000 - 84.823001646924417438491371348547) + 0.00184387741784130762025262217918 * cos(43.982297150257105338477007365913 * J / 1000 - 87.964594300514210676954014731826) - 0.0001063009081020056426991962639228 * cos(45.553093477052001957708329057553 * J / 1000 - 91.106186954104003915416658115106) - 0.0014857593370028508803631916990184 * cos(47.123889803846898576939650749193 * J / 1000 - 94.247779607693797153879301498385) - 0.40735978165800101891136364429258 * sin(1.5707963267948966192313216916398 * J / 1000 - 3.1415926535897932384626433832795) + 0.36902926697510990416972731509304 * sin(3.1415926535897932384626433832795 * J / 1000 - 6.283185307179586476925286766559) + 0.27650478685578072646222835828667 * sin(4.7123889803846898576939650749193 * J / 1000 - 9.4247779607693797153879301498385) - 0.24010596760411831440684693461662 * sin(6.283185307179586476925286766559 * J / 1000 - 12.566370614359172953850573533118) + 0.22934115622454046001799099485652 * sin(7.8539816339744830961566084581988 * J / 1000 - 15.707963267948966192313216916398) + 0.16314576016869208885395892139059 * sin(9.4247779607693797153879301498385 * J / 1000 - 18.849555921538759430775860299677) - 0.075660085593284007177139471878036 * sin(10.995574287564276334619251841478 * J / 1000 - 21.991148575128552669238503682957) + 0.025080151205476224757218162153549 * sin(12.566370614359172953850573533118 * J / 1000 - 25.132741228718345907701147066236) + 0.029025566898123154879751695034429 * sin(14.137166941154069573081895224758 * J / 1000 - 28.274333882308139146163790449516) - 0.017099148665274298969363186984083 * sin(15.707963267948966192313216916398 * J / 1000 - 31.415926535897932384626433832795) + 0.0068382629448068771652047281861542 * sin(17.278759594743862811544538608037 * J / 1000 - 34.557519189487725623089077216075) + 0.024755682708528722879570693748974 * sin(18.849555921538759430775860299677 * J / 1000 - 37.699111843077518861551720599354) - 0.0026775696315067300246060000290527 * sin(20.420352248333656050007181991317 * J / 1000 - 40.840704496667312100014363982634) - 0.016581165405171804233530608030378 * sin(21.991148575128552669238503682957 * J / 1000 - 43.982297150257105338477007365913) + 0.015873736311036581309030779607383 * sin(23.561944901923449288469825374596 * J / 1000 - 47.123889803846898576939650749193) + 0.0075226318463759928381606734149045 * sin(25.132741228718345907701147066236 * J / 1000 - 50.265482457436691815402294132472) - 0.002888920432183892089000876168825 * sin(26.703537555513242526932468757876 * J / 1000 - 53.407075111026485053864937515752) - 0.0025964056220633387542240377854341 * sin(28.274333882308139146163790449516 * J / 1000 - 56.548667764616278292327580899031) + 0.0017557411274634177576758897743048 * sin(29.845130209103035765395112141155 * J / 1000 - 59.690260418206071530790224282311) + 0.0080436955120667414514956661264478 * sin(31.415926535897932384626433832795 * J / 1000 - 62.83185307179586476925286766559) + 0.0030405048616952375963029542305094 * sin(32.986722862692829003857755524435 * J / 1000 - 65.97344572538565800771551104887) + 0.0015435130995028955528530767438156 * sin(34.557519189487725623089077216075 * J / 1000 - 69.115038378975451246178154432149) + 0.0016212873049575846941261492162312 * sin(36.128315516282622242320398907714 * J / 1000 - 72.256631032565244484640797815429) - 0.0022651699057514053381456875513322 * sin(37.699111843077518861551720599354 * J / 1000 - 75.398223686155037723103441198708) + 0.000066303610137690390034619081216505 * sin(39.269908169872415480783042290994 * J / 1000 - 78.539816339744830961566084581988) + 0.0016990371373291548052347765818126 * sin(40.840704496667312100014363982634 * J / 1000 - 81.681408993334624200028727965267) + 0.0016357166679813214642258412112596 * sin(42.411500823462208719245685674273 * J / 1000 - 84.823001646924417438491371348547) - 0.00071672942111724441665437934645411 * sin(43.982297150257105338477007365913 * J / 1000 - 87.964594300514210676954014731826) - 0.0021529742145817401706431315488999 * sin(45.553093477052001957708329057553 * J / 1000 - 91.106186954104003915416658115106) + 0.00056360059466409285437032972154725 * sin(47.123889803846898576939650749193 * J / 1000 - 94.247779607693797153879301498385) + 0.79318147518131831841259327120497);
    //Serial.println(J);
  }
  ei_prev = 0;
  delay(1000);  // Protective delay to prevent pump maxout voltage at start up needs to be modified based on the processing time of "Void Setup"
}




void loop() {

  //t = millis();     // the real time CPU time in mili s
  T = micros();       // @@@Loop speed Check
  t = micros();       //the real time CPU time in micro s. This provides microseconds since the Arduino board began running This number will overflow (go back to zero), after approximately 70 minutes. On 16 MHz Arduino boards (e.g. Duemilanove and Nano), this function has a resolution of four microseconds
  dt = t - t_prev;
  dt = dt / 1000;     // to convert the dt in (micro s) to (mili s) if in above line the "t = micros();" has been activated
  //Serial.println(dt);

  T1 = (unsigned long int)(t / (Time_Step * 1000)) % int(Ref_Sig_T / Time_Step); 			// Very important command line. This is converting the realtime in CPU ("t") to the element index in the Reference signal. Time_Step multiplied to 1000 to convert it to micro second because "t" is in micro s.
  //Serial.println(T1);


  //--------------- PID gain Scheduling. -------------------------
  // Where each new sets of gain starts/ends (T1) is very critical.

  if (T1 == 0 && C1 <= 3) {
    ei_prev = 0;    						// resets the integral term error to prevent "windup"
    C1 = C1+1;
  }

  if (T1 == 0) {							// To generate a Digital 5 V pulse at Pin 50 for synchronization applications or triggering other devices.
    L2_Error = 0.0;
    digitalWrite(50, HIGH);
  }
  if (T1 == 20) {
    digitalWrite(50, LOW);
  }

  if (T1 == 1) {                           //1
    //ei_prev = 0.1*ei_prev;
    kp = 80.0 / 10000.0;                   //85
    ki = 77. / 1000000.0;                  //80
    kd = 50.0 / 10000.0;                   //50
  }
  else if (T1 == 255) {                    //254
    kp = 75.0 / 10000.0;                   //80
    ki = 60 / 1000000.0;                   //60
    kd = 50.0 / 10000.0;                   //50
  }
  else if (T1 >= 255 && T1 < 320) {        //254< <280
    kp = 45 / 10000.0;                     //61
    ki =  ki - 0.07 / 1000000.0;           //-0.075    kd = 50.0 / 10000.0;
  }

  if (T1 >= 380 && T1 < 1332) {            //380 < <1332
    kp = kp + 0.008 / 10000.0;             //+0.01
    ki = ki + 0.018 / 1000000.0;           //+0.0168
    kd = 50.0 / 10000.0;                   //50
  }



  v = analogRead(A0);                             		// Flowrate feedback input
  analogWrite(11, GenVd[T1] / 5.0 * 255 );        		// Reference signal to Scope for observation. "/ 5.0 * 255" should be used for correct analog values.

  //vd = analogRead(A1);                          		// Desired signal IF IT IS BEING GENERATED FROM OUTSIDE SOURCE LIKE signal generator OR  LABVIEW & DAQ
  vd = GenVd[T1 + num_Step_Ahead] / 5.0 * 1024;   		// Desired signal generated above but due to Analog/digital conversion it must be scaled to 1024 for this Arduino board. --> 5V=1024

  ex = vd  - v;                                 		// Proportional term
  ed = ( (vd - vd_prev) - (v - v_prev)) / dt;   		// Derivative term (1st order forward finite difference)
  //ei = ei_prev + ex * dt;                     		// Integral term (rectangle)
  ei = ei_prev + (ex + ex_prev) / 2.0 * dt;     		// Integral term (Trapezoidal)
  v_out = kp * ex + kd * ed + ki * ei;          		// PID
  //  Serial.println(v_out);

  //-- Safety measures for protecting the pump from over/under voltage  --------------
  if (v_out > 4.7) {
    v_out = 2.;
  }
  if (v_out < 0 ) {
    v_out = 0;
  }
  //------------------------------------------------------------------------------
  //------------------------------------------------------------------------------
//v_out = 0.0;
  analogWrite(10, v_out / 5.0 * 255 );
 
//-------- OUT PUT data to serial print -----------  THIS SLOWS DOWN THE LOOP IF ENABLED SO t1 SHOULD BE MONITORED AND Delay SHOULD BE REDUCED TO ABOUT 20

//  Serial.print(T1 / 1332.*3.999, 3); // Serial.print does not go to NEXT LINE but Serial.println goes to NEXT LINE
//  Serial.print("\t");
//  Serial.print(GenVd[T1], 4);
//  Serial.print("\t");
//  //Serial.print(T1 / 1332.*3.999, 3);
//  //Serial.print("\t");
//  Serial.println(v * 5 / 1024., 4);
//  if (T1 == 0) {
//    Serial.println("Begin  t  Ref  t  Flow ");
//    Serial.println("");
//  }

//  L2_Error = sq(abs(ex) / 1.) + L2_Error;
//  if (T1 == 1332) {
//    L2_Error = sqrt(L2_Error) / 1333;
//    Serial.println(L2_Error);
//    L2_Error = 0.0;
//  }

  t_prev = t;
  ex_prev = ex;
  ei_prev = ei;
  vd_prev = vd;
  v_prev = v;

  //delay(2);					// This is the Delay in Mili second which is so much for these applications. Instead delayMicroseconds is used.
  delayMicroseconds(1500); 		//The minimum for this number is 4 (Because the resolution of 16MHz board is 4 mic s) BUT for stability it is better to be above 1000. OR activate the delay in line above which is in mili seconds for slower applications. It is important that "T" (in mili second) be smaller than "Time_Step" (in mili second).
  //T = micros() - T;       	// @@@Loop speed Check. Shows the Loop running time in "micro s".
  //Serial.println(T, DEC); 	// @@@Loop speed Check

}
