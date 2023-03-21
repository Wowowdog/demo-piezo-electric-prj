# piezo electric motor prj 
## introduction
This is a demo script for piezo electric motor control on STM32/ARM M7 cortex. 
## feature
* FIFO ADC ring buffer
* PWM master-slave mode (TIM1 update event triggers PWM1/PWM2 per ctrl output)
* direct memory access b/t ADC & RAM
* high SNR w/ Kalman filter
* discrete controller w/o stability concern
* both open or close model applicable  
* USB receiving callback for ctrl scenario 

## Kalman filter design
![alt text](https://github.com/Wowowdog/demo-piezo-electric-prj/blob/master/png/fil1.png?raw=true)

## controller design
![alt text](https://github.com/Wowowdog/demo-piezo-electric-prj/blob/master/png/ctl1.png?raw=true)

## image snippet
![alt text](https://github.com/Wowowdog/demo-piezo-electric-prj/blob/master/png/mag1.png?raw=true)
