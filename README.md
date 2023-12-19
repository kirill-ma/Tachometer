# Cheap and fast tachometer project
Based on BluePill STM32F103C6 board, 128x32 OLED and IR sensor module. You could get interconnection schematics from source code. Both of OLED and IR are powered from 3.3V. See more in /pictures/ 

![4](https://github.com/kirill-ma/Tachometer/blob/master/pictures/4.gif)

### Measuring range 6...5000rpm
Just use a piece of white masking tape as a mark on the wheel!

__Don't forget to solder a 22k resistor between pins 5 and 7 of LM393 in order to add some hysteresis!__
<img src="https://github.com/kirill-ma/Tachometer/blob/master/pictures/3.jpg" width="600">

### Code:
I used VScode + PlatformIO + Arduino framework to create it
