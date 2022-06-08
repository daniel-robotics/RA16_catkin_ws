#### Name of resulting executable
TARGET = RA16


#### List of all .c files. Each file on its own line, separate lines by \ character.
TARGET_SOURCES = ./src/main.c									\
				 ./src/Globals.c								\
				 ./src/Control/Timing.c							\
				 ./src/Control/MotorRegulator.c					\
				 ./src/Control/Targeting.c						\
				 ./src/Control/Kinematics.c						\
				 ./src/Control/Homing.c							\
				 ./src/Sensors/Sensors.c						\
				 ./src/Sensors/PCF8574.c						\
				 ./src/Sensors/EOPD.c							\
				 ./src/Comms/RS485.c							\
				 ./src/Comms/Bluetooth.c						\
				 ./src/Comms/RCXComm.c							\
				 ./src/HumanInterface/Sound.c					\
				 ./src/HumanInterface/LCD.c						\
				 fix16.c										\
				 fix16_str.c									\
				 fix16_sqrt.c									\
				 fix16_trig.c									\
				 fixmatrix.c									\
				 fixarray.c



#### OSEK .oil files, which configure the RTOS environment and tasks
TOPPERS_OSEK_OIL_SOURCE = \
	./project.oil


#### External source code to include. Each path on its own line, separate lines by \ character.
USER_INC_PATH = \
	./include/libfixmatrix				\
	./include/libfixmath
VPATH = $(USER_INC_PATH)


#### External libraries to statically-link. Use the name of the archive file (maybe?) without the .a extension.
#### Should work if the .a file is somewhere in the USER_INC_PATH
# USER_LIB = \
#     libfixmath


#### User-defined compile macros
# USER_DEF = MY_MACRO


#### Defining NO_RUN_ENTER_STOP_EXIT disables initial display and RUN/STOP/ENTER/STOP buttons.
USER_DEF += NO_RUN_ENTER_STOP_EXIT


#### Additional compiler flags; See ecrobot/tool_gcc.mak for more info
# USER_C_OPT = 


#### Bitmap image files to include into the executable
# BMP_SOURCES = \
#  	 ./image.bmp


#### WAV sound files to include into the executable
# WAV_SOURCES = \
#  	 ./soundeffect.wav


#### Specify which executables to build
#### Choose only 1 option or leave commented-out to build all three
# BUILD_MODE = ROM_ONLY
# BUILD_MODE = RAM_ONLY
BUILD_MODE = RXE_ONLY



#### DO NOT MODIFY
O_PATH ?= build
include /usr/local/src/nxtosek/ecrobot/ecrobot.mak
