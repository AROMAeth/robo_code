This is the package where the handling of our AirPump is happening which is the crucial part for our bubbling system.

# Input paramets

The only crucial input argument is now the one single pin which is responsible for turning the pump on and off.

# Handling of the node

In the next section we will describe how the airpump can be handled!!!

A sample command to bubble in total for 20 seconds with a repeated period length of 5 seconds where 50% of the 5 seconds the pump is on is:
rostopic pub /aroma_airpump/control std_msgs/String "20 5 0.5"

This rostopic pub command can as always be executed in a seperate terminal or also in our Finite State Machine (fsm) which runs AROMA autonomously!

# Other usage

In general, this programme can be also used easily in order to control other electronic components since it simply puts the specified pin to 1, each cycle for a time of: period_length*percentages


