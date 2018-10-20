# robo_code

In this package you find all the subpackages which take care of controlling our robot and makes it move. 

# Building the package

The package can be build via: catkin_make

# What is in the package:

Packages which were used on the final version of AROMA:

- Controlling the syringes (aroma_syringe)
- Controllinge the driving of the robot (aroma_drive)
- Controlling the airpump (aroma_airpump)
- Controlling the ArduCam MT9J001 camera (arducam_node)
- Motility image analysis pipeline (aroma_motility)

Furthermore there are also some other packages which were not required on our robot in its final version:
- Controlling the RaspberryPi camera (raspicam_node)
- Controlling stepper motors individually (aroma_stepper)
- An interface for the aroma_stepper package (aroma_interface)

# Scripts

The scripts in here contrain the commands which are needed if the individual programs (nodes) are running on different devices. 

Packages which essentially have to be run on a RaspberryPi, because GPIO Pins are absolutely required:
- aroma_syringe
- aroma_drive
- aroma_airpump

The packages:
- arducam_node
- aroma_motility
Can also be run on another device 

# Further Details

Further Details on how to use the nodes and interface with them can be found in the Readme's in the individual package!
