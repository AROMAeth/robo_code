In this package you can find the implementation for controlling our syringe system!

MORE DETAILS TO PUT HERE:

# different syringe pump sizes

Since everyone will have other syringe pumps on his or her device or even just use the syringe pump as a standalone solution, there is the parameter step_size. As one can read in the launchfile, in this file there is to specify how many steps are needed by the motor to pump a volume if 1mL.

# controlling several syringes:

if several or mutiple channels have to be controlled, then the name of the respective pump can be changed and therefore it can be controlled individually especially via the launchfile and the parameter name

#  how to control the pump

the pump is again controlled via a standart topic which is called syringe_control

e.g.

rostopic pub /noname/syringe_control std_msgs/String "pull 10 100"
rostopic pub /noname/syringe_control std_msgs/String "push 10 100"
explanation: FIRST: either push or pull, then volume in uL, then speed in uL/min  

# how to specify the pins where they are located

this can also be done in the launchfile via the pins argument

# EXAMPLES:

to see how exemplarily multiple syringes are launched, with different pins, volumes and names can be seen in the aroma_fsm package!!!


