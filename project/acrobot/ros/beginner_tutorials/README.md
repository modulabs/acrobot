Basic publish and subscribe example in python, sending and receiving Float64MuliArray data type

This example was modified from standard ROS tutorials.
(<http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(python)>)

It's important to know a data type I want to publish and subscribe.
There are 2 ways.

- Using rosmsg tool
```sh
$ rosmsg show Float64MultiArray
```
Then,
```sh
[std_msgs/Float64MultiArray]:
std_msgs/MultiArrayLayout layout
  std_msgs/MultiArrayDimension[] dim
    string label
    uint32 size
    uint32 stride
  uint32 data_offset
float64[] data
```

- In python: have to open python in the catkin_ws which includes ROS directories.
```sh
>>> from std_msgs.msgs import Float64MultiArray
>>> f = Float64MultiArray()
>>> f
```
```sh
layout: 
  dim: []
  data_offset: 0
data: []
```
```sh
>>> type(f.data)
```
```sh
<type 'list'>
```

From above analysis, we could know that 'list' of float64 can be stored in data.

Next step is just to follow publish and subscribe coding rule.

In talker.py,
```python
from std_msgs.msg import Float64MultiArray

pub = rospy.Publisher('chatter', Float64MultiArray, queue_size=10)
f = Float64MultiArray()
f.data = [0.1, 0,2] # for example
pub.publish(f)
```
In listner.py,
```python
from std_msgs.msg import Float64MultiArray

rospy.Subscriber('chatter', Float64MultiArray, callback)
```
and get float64 data in callback function
```python
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %f, %f', data.data[0], data.data[1])
```
