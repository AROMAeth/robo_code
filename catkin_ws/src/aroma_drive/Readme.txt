This is the node which drives AROMA around! :))

TURNING LEFT == positive angle
TURNING RIGHT == negative angle
IMPORTANT: angle is defined from the front direction!!!

In general the idea is that at first the whole robot turns for an wanted angle and then drives into the direction that it is pointing to!!!

# Code usage

sample usage of this code:
rostopic pub /aroma_drive/control std_msgs/String "backward -10 0"
This lets the robot turn for 10 degrees to the right and 0m of driving overall

OR:
rostopic pub /aroma_drive/control std_msgs/String "forward 0 5"
This lets the robot drive 5 meters forward


# Calibration:

As some people will use other motors or wheels and since all those dynamics will be different we decided to add basically two calibration parameters into our launchfile!!

this is on the one hand the "straight_speed" and the "angle_speed".
Let me give a short explanatory example. If you use the command (rostopic pub /aroma_drive/control std_msgs/String "forward 0 5") and your robot drives 6 instead of 5 meters, then you have to increase the "staight_speed" by a factor of 6/5 = 1.2

The same can be applied for finetuning the angular direction!!!

