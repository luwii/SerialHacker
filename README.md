# SerialHacker
 
 This software will help identify serial communication activity on 
 an unknown circuit board.  Connect your 'Arduino' pins to the unknown board 
 and this sketch will try every possible pin combination and baud rate to 
 determine RX/TX. 
 It can also try to send wakeup characters to stimulate the lines, 
 such as a carriage return or other txpatterns that you add. 
 
<B>Usage:</B>
 1. To your project add: main.cpp to the src tree, and utils.c to the include file tree.
 2. Read the text section in Main.cpp
 3. Adjust the User Definition section as required, especially the first setting for the board type.
 4. Connect a test board that runs on the same voltage to get a feel for how it work.

<B>Notes:</B>

 Testing of this software was performed using a classic Arduino Uno, Pro Mini and a newer Due; against each other and some random boards.
 
 Code compiled in PlatformIO under VisualStudio Code.
 
This project is based on Github repo https://github.com/cyphunk/RS232enum  (v0.3.0 20110924)
A lot was changed, hence a new project was started. 
