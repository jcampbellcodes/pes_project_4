# PES Project 4 Readme
Jack Campbell

## Description
This repo contains custom sources and makefiles for Project 4 as well as adapted and generated code 
from MCUXpresso and the KL25Z SDK.

This project contains three configurations: Test, Debug, and Normal.

Test runs a suite of unit tests covering the state machine code.

Debug and Normal both run the state machines controlling the TMP102, but contain different logging
statement levels.

The state machine uses a strategy pattern to ping-pong its event handler between a table based and
a state-based state machine.

## Observations

I spent the bulk of my time on the front end of this project working on designing the state machine and
logging infrastructure, which ended up being potentially a bad call -- I saved the I2C and TMP102 work 
for the weekend before it was due, and had a lot of trouble debugging that communication.

I also spent most of the weekend learning how to use an oscilloscope and how to pick a header pin configuration
to solder to my board, which was a good experience, but pushed my I2C troubles even closer to the deadline.

In the future I think I will consider my "unbounded work" anything hardware related and try to get it handled first.
Writing and designing pure software is more bounded and can be crammed into the end of a schedule easier than
something I have much less experience with.


I've still attached my logic analyzer diagrams, since I believe the I2C transactions were 
*not* malformed... I point out in the diagrams everywhere where there *should* be a TMP102 ACK, but 
none arrives. However, these graphs show that the master  is doing what it is supposed to be doing. 
According to several students and Shreya, my init code and I2C code is doing the same thing as others who have
a working solution...

I spent a fair amount of time debugging with Shreya (five hours) and tried three different TMP102's and 4 different KL25Z's.
I even tried running Shreya's working I2C code and it wasn't working when run under my setup. So something is wrong in my
setup, probably something simple and dumb, but in the end I decided it was beyond my dedication to get it actually working.


To show off the state machine logic, I added a define in tmp102.c called USE_TMP102. If this is not defined, then I 
have stub functions that return "meaningful" values so I can demonstrate the sequence of the state machine.


## Installation/Execution Notes

These are the steps to build the project in MCUXpresso.

1) Clone the repo
2) In MCUXpresso, click `New > Project`.
3) Select `Makefile project with existing code...`
4) Unselect C++, enter a project name, browse to the directory of the repo, and select `NXP MCU Tools`, then hit next.
5) Now set up the configurations. Right click the project,
6) Hit Properties
7) Uncheck "Generate makefiles"
8) Add "Debug" to the build directory path in the same dialog.
9) Do the same for Normal and Test configurations.

### Running the FB builds

1) Right click on the project name in the file hierarchy, select `Debug as > Debug configurations...`
2) Select `GDB PEMicro Interface Debugging`
3) Hit `New launch configuration`
4) Select a name for the output configuration (you need one for both Release and Debug)
5) Set the `C/C++ Application` field to the binary you want to run, either `Debug/output/kl25z_debug.axf` for Debug or `Release/output/kl25z_run.axf` for Release
6) Hit Apply
7) Hit Debug
8) The program should run in the console below, provided the board is connected successfully.

