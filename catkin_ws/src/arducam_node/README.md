# Launching
There are some issues due to the sudo command which is necesarry to read out the usb ports, therefore before you will be able to launch the following commands are necessary:

sudo -s

source /opt/ros/kinetic/setup.bash 

source devel/setup.bash 

To exit this root mode you can simply use "su "YOUR NORMAL USERNAME""

Other variant: make sure to have passwordless sudo and then you can simply source the "root_setup.sh" file,...

Then in the root mode simply type for instance: roslaunch arducam_node arducam_node.launch 

ATTENTION: ONLY USE THIS SUDO MODE FOR LAUNCHING THE STUFF BUT NEVER FOR BUILDING ETC.

# Description
This is a further development of the ArduCAM USB Camera Shield which should yield to a ROS integrated version. Due to time issues, this is done here locally in our aroma repository. 

# Writing and Reading REGISTERS:

In addition I implemented a first way on how the value of some registers can be read out or actually written to:

WRITING (can be achieved by typing this command in another terminal):

rostopic pub /change_reg std_msgs/String "RegisterAdressToBeChanged ValueThatYouWant"

e.g.: rostopic pub /change_reg std_msgs/String "0x3012 0x00FF"

READING (can be achieved via this command:)

rostopic pub /read_reg std_msgs/String "'RegisterToBeReadOut'"

e.g.: rostopic pub /read_reg std_msgs/String "'0x3012'"

# LOGGING

rostopic pub /logging_topic std_msgs/String "true hallo" 

(1st argument tells if we want to start or end,...)

# LAUNCHING OPTIONS

With the MT9J001, we had two applications, on the one hand to provide the image to read out the motility of the bacteria and on the other hand to do the luminescence experiments.

## MOTILITY IMAGING:

In order to launch the camera readout for the motility imaging, simply type:

roslaunch arducam_node arducam_node_old.launch

This will launch the camera and read out the pictures.  

NOTE:

In the launchfile one can specify the camera configuration file which is loaded.

AVAILABLE FILES:

- MT9J001_10MP_opt_m.json: This launchfile causes only the center of the image to be read out. Therefore it was possible to reach a framerate of 27 fps
- MT9J001_10MP.json: This is the original configuration file which causes the whole image ("full field of view") to be read out but therefore only a rate of 7fps can be reached.
- MT9J001_10MP_opt_b.json: Also a smaller field of view is read out in this configuration. However, the smaller field of view is not centrally but in the top left corner of the original "full field of view" image
- MT9J001_10MP_smaller.json: With this configuration file, a field of view in between the original and "opt" configuration is read out. Therefore a framerate of roughly 15fps is achieved and the original image is cropped again in the top left corner with this configuration file

This image stream can then be further analyzed with the help of the aroma_motility package!

## LUMINESCENE IMAGING:

In order to start the luminescence imaging mode with a maximum of exposure time and also adjusted gains one has to type:

roslaunch arducam_node arducam_node_long.launch

When performing the luminescence imaging also a special logging function is available which stores the raw data of 6 consecutive images into a folder. The logging can be started by typing (in another terminal):

rostopic pub /logging_topic std_msgs/String "true hallo" 

(1st argument tells if we want to start or end,...)

Furthermore in the devel_scrips folder a jupyter notebook file is provided which exemplarily shows how the luminescence imaging files can be analysed (lumin_file_readout.ipynb)

# Overview
ArduCAM USB Camera Shield is a general purpose USB camera controller board that can accommodate wide range of CMOS camera module from 0.3MP ~ 14MP.
By using provided SDK library and demo source code, user can design their own applications.
More info can be found from [arducam website](http://www.arducam.com/arducam-usb-camera-shield-released/)

## Now Supported Cameras
-	OV7670		0.3MP
-	OV7675		0.3MP
-	OV7725		0.3MP
-	MT9V034		0.36MP (global shutter)
-	MT9M112		1.3MP	
-	MT9M001		1.3MP (Monochrome/Color)	
-	AR0134		1.2MP (global shutter)
-	OV2640		2MP	
-	OV5642		5MP	
-	OV5640		5MP 
-	MT9P001   5MP
-	MT9N001		9MP
-	MT9J001		10MP (Monochrome)
-	MT9J003   10MP (Color)
-	MT9F002		14MP

## Support OS 
- Windows
- Linux Ubuntu
- Raspbian

## Limitations
The new USB2.0 camera shield now has onboard frame buffer, it won't have the limitation like before.