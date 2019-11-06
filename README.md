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

## Installation/Execution Notes

These are the steps to build the project in MCUXpresso.

1) Clone the repo
2) In MCUXpresso, click `New > Project`.
3) Select `Makefile project with existing code...`
4) Unselect C++, enter a project name, browse to the directory of the repo, and select `NXP MCU Tools`, then hit next.
5) Now the project is active in the IDE.

### Running the FB builds

1) Right click on the project name in the file hierarchy, select `Debug as > Debug configurations...`
2) Select `GDB PEMicro Interface Debugging`
3) Hit `New launch configuration`
4) Select a name for the output configuration (you need one for both Release and Debug)
5) Set the `C/C++ Application` field to the binary you want to run, either `Debug/output/kl25z_debug.axf` for Debug or `Release/output/kl25z_run.axf` for Release
6) Hit Apply
7) Hit Debug
8) The program should run in the console below, provided the board is connected successfully.

## Observations

I spent the bulk of my time on the front end of this project working on designing the state machine and
logging infrastructure, which ended up being potentially a bad call -- I saved the I2C and TMP102 work 
for the weekend before it was due, and had a lot of trouble debugging that communication.

I also spent most of the weekend learning how to use an oscilloscope and how to pick a header pin configuration
to solder to my board, which was a good experience, but pushed my I2C troubles even closer to the deadline.

In the future I think I will consider my "unbounded work" anything hardware related and try to get it handled first.
Writing and designing pure software is more bounded and can be crammed into the end of a schedule easier than
something I have much less experience with.

