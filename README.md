# piezo electric motor prj 
## introduction
This is a demo script for piezo electric motor control on STM32/ARM M7 cortex. 
## feature
* muti-channel ADC reading
* direct memory access b/tw ADC & RAM
* Kalman filter (optional)
* discrete controller
* both open or close model applicable  
* USB receiving callback for ctrl scenario 
## Kalman filter design
![alt text](https://github.com/Wowowdog/demo-VCM-prj/blob/master/png/fil1.png?raw=true)

## controller design
![alt text](https://github.com/Wowowdog/demo-VCM-prj/blob/master/png/ctl1.png?raw=true)

## image snippet
![alt text](https://github.com/Wowowdog/demo-VCM-prj/blob/master/png/mag1.png?raw=true)

## associated doc.
[filter design note](https://drive.google.com/file/d/1aqMnyfdr6wfS0KhNzTkJJ_yOEIsFtxLj/view?usp=share_link)

[ontroller design note](https://drive.google.com/file/d/1aofWPQ_WVctiZDtwdYiYLdtpWiVnDgS7/view?usp=share_link)
