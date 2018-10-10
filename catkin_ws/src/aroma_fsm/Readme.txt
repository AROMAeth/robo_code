Here is the finite state machine that will control and take care of the overall signal flow on our robot aroma.
It will take care that the individual steps run in the right order,....

IMPORTANT:
Inside the launch file of this package, the whole system will be launched.
Therefore checkout the launchfile how to name and specify the name of the syringes, pins, the number of steps needed for 1mL etc.

There are in general two launching options:

When launching via the command:
roslaunch aroma_fsm aroma_fsm.launch automatic:=false
All of the nodes turn on and launch, however, the automatic and autonomous cycle is not launched. Therefore in this configuration the user can define via the terminal what is about to happen. E.g. this launch is needed to drive the syringe pumps into reasonable initial positions

WHEN USING:
roslaunch aroma_fsm aroma_fsm.launch
Then the automatic mode is launched and this equals the mode which is later needed in the field!!!
