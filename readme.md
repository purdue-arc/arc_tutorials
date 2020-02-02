This package contains example rosnodes and implements custom messages defined in example_msgs

## Nodes
### state_publisher_node
This is a very basic node that exists entirely inside a main function. It
periodically sends out example_msgs/State messages

### clock_face_node
This is a slightly more complex node that exists inside a main function and uses
another class to offload some functionality. It's functionality is pretty
useless, it publishes example_msgs/ClockFace messages that show the current EST
time using the hands on an analog clock

### clock_face_converter_node
This node has a very bare main function, and the functionality exists within a
separate class. It converts the example_msgs/ClockFace messages to a
display_msgs/Marker that can be viewed within Rviz

## Launch files
### state_publisher.launch
Launches state_publisher_node

### clock_face.launch
Launches clock_face_node

### clock_face_converter.launch
Launches clock_face_converter_node

### combined_clock.launch
Launches clock_face_node, clock_face_converter_node, and rviz

## Parameter files
### clock_face_converter_params.yaml
Contains parameters for clock_face_converter_node

## Rviz files
### clock.rviz
Set up to view output of clock_face_converter_node
