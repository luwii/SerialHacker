# SerialHacker
 
 This software will help identify serial communication activity on 
 an unknown circuit board.  Load it onto your Arduino and connect some 
 pins to the unknown board. This will then try every possible pin 
 and baud rate combination to find activity. 
 It can also try to send wakeup characters to stimulate the lines, 
 such as a carriage return or other patterns that you add. 
 Once you've spotted activity, it can help you to determine the baud rate.
 
<B>Usage:</B>
 1. To your project add: main.cpp to the src tree, and utils.c to the include file tree.
 2. Read the text section in Main.cpp
 3. Adjust the User Definition section as required, especially the first setting for the board type.
 4. Load this onto your Arduino. 
 5. Connect the Arduino to a test board that runs on the same voltage to get a feel for how it works. Remember a common Gnd.
 6. Open a Terminal connection to the Arduino at 115400 baud and refer to the User Manual to explain the menu functions.

<B>Notes:</B>

 Testing of this software was performed using a classic Arduino Uno, Pro Mini and a newer Due; against each other and some random boards.
 
 Code compiled in PlatformIO under VisualStudio Code.
 
This project is based on Github repo https://github.com/cyphunk/RS232enum  (v0.3.0 20110924)
A lot was changed to make it more usable, hence a new project was started. 
