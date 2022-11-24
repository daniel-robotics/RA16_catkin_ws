# RA16_catkin_ws
ROS Catkin Workspace for the RA-16 Robotic Arm

Project task list:

1. Software infrastructure
   a. Better 
2. NXT param server and PC python client (provided as independent Catkin pkg?)
    a. NXTOSEK task hosts a parameter server over serial. Can interface with USB, RS485, BT, etc.
    b. Params are defined at compile time. Each is given a unique 4-byte name (union of 4 char array and uint32 for easy comparison)
        and a memory address pointing to the global variable holding the parameter's value.
        struct: param
            union: name
                char** ['n', 'a', 'm', 'e']
                uint32_t
            uint8_t type  // enum; FIX16, SIGNED INT, UNSIGNED INT, BOOL, MAT, ETC
            uint8_t size
            uint8_t * addr
            uint8_t flags // bit for readable, bit for writeable, 
    c. Protocol:

        


2. Hitechnic Angle Sensor driver
1
2. Motor controllers:
 b. MATLAB/simulink<->python<->NXT comm stack
 a. Motor testing rig.
 c. 
 - 