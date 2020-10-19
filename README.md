# Silicon-Garden

Welcome! This repo contains several forms of discrete and continuous controllers. Originally this was mostly going to be used for automatically watering a garden but I realized that a simple PID control is more than good enough. Instead I have created a playground for people to apply MPC control to a state space model of their choice, be it continuous or discrete, and see the results! It is a lot of fun, and there are many opportunities for you to mess up or do cool things with it.

To run the MPC, go to task runner. On top of the script there are several parameters that you can play with and adjust, including a section where you can input your own state space model. 

As an example I have a decomposer reactor train with 2 stages, where the monitored variable is the concentration output. See if you can comply with EPA regulations by reducing the output concentration to 10% of the input concentration (that would be input number 2!).
