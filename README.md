# ATmega8515_EC1204B_Clock

Some weeks ago I get in contact with the circuit board EC1204B.
It's a small pcb with 4 lcd 7-segments and 60 led's arranged in a circuit like a clock.
Original equip with a 'AT89S52' (compatible with 80C51 instruction set and pinout) I want
to reprogram the PCB with my AVR stuff. So I find a (nearly) compatible pinout µC: ATmega8515.

Many thanks to Cristian COPCEA who started a nearly same project in 2015.
He programmed first a 'AT90S8515' and also switched to 'ATmega8515'. 
He described his experiences and the small hardware changes necessary 
for using the ATmega8515 with the EC1204B board in a wide documentation.
This was the base and my entry point. (His lastest version I used to start my project was the 'EC1204B-10.zip')

The two main points for reprogramming the clock was to
- add the possibility to show the number of days to a particular date
  (to show my nephew the number of days until he is 18 :-)

- add a nerf gun detector to stop the alarm clock
  (like seen in this example: https://www.youtube.com/watch?v=-2WAyDod0W4)
  
  I also convert it to a Atmel Studio 7.0 project.
  By the way, I notice that the Hex-file of Studio 7 was smaller then build with WinAVR, etc.
  That was very important for me, cause the current hex-file has nearly a size of 99% of the 
  ATmega8515 flash. If you need more flash size, have a look at the ATmega162. It's a pin compatible
  type which has 16K bytes programmable flash. 
  
  See the documentation folder for the operation manual of the clock
  (github.com/elektro-slobbo/ATmega8515_EC1204B_Clock/tree/master/documentation/)
  
  If you are interessted in the current source code please write email to me 
  and feel free to improve the code and let me know!
  
 Update 2020:
- Time switches after 10 sec now. Other items after a shorter time. 
 
  
  Slobbo  
