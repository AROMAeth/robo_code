In this package you can find the implementation for controlling our syringe system!

MORE DETAILS TO PUT HERE:

- different syringe pump sizes
- how to pass this information
- how to control the pump
- how to specify the pins where they are located


rostopic pub /PUMP_NAME/syringe_control std_msgs/String "push 1 1" 
explanation: FIRST: either push or pull, then volume in uL, then speed in uL/min