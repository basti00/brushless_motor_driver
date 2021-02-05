# Brushless motor driver

A STM32 Arduino library to demonstrate space vector modulation (SVM) using three center aligned PWMs and alternating reversing sequencing. For more information on SVM 
see these [slides](http://www.kappaiq.com/download/presentation-material/PDF/05%20Modulation.pdf).

Here you can see the phase switching sequencing for a slowly rotating vector with magnitude 0.8. Notice how it switches between all phases low and all phases high, that's the characteristic of alternating reversing sequencing.

![alt text](https://github.com/basti30/brushless_motor_driver/blob/master/img/svm_08.gif "Logo Title Text 1")

Three modes are available: 
* setting a static vector 
* rotating the vector at constant speed
* advancing the vector a fixed angle in front of the measured rotor angle 

See the provided examples on how to use them.
