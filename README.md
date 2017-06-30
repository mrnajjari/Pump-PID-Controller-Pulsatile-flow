# Pump Controller program to produce Pulsatile flow rate
Arduino program for pump controller to produce pulsatile flow rate using PID controller. In vitro fluid mechanical experimental studies.
For referencing these program please use the github URL and following paper:

Najjari MR, Plesniak MW (2017) PID controller design to generate pulsatile flow rate for in vitro experimental studies of physiological flows, Experiments in Fluids (submitted)


An Arduino board was used to control a pump to produce variable flow rates.
A proportional-integral-derivative (PID) control algorithm was developed for this purpose.
This system can be implemented in many flow loops due to its simplicity and low cost, and does not require a mathematical model of the system.
The main application is to control the pump for in vitro experimental studies in fluid mechanics research inspired by physiological flow.

Four Arduino scripts are provided to produce 0.5 Hz sine flow, 1 Hz sine flow, 0.5 Hz multi sine flow and 0.25 Hz physiological common carotid artery pulsatile flow rate.
To enhance the accuracy of the controller gain scheduling method was implemented. For each applications gains needs to be tuned.
