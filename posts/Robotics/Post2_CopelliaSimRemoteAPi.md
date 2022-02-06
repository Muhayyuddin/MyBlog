# ROS Publisher and Subscriber Nodes

This article will explain ROS publisher and subscriber with the help of basic examples. we will discuss the basic example of publisher and subscriber nodes in C++ and Python. 
## ROS Node

## Publisher and Subscriber Nodes

### C++ example
<table> <tr > <td > 

#### Publisher 
</td> <td > 

#### Subscriber
</td></tr>
<tr > <td > 

```cpp 

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.
  
  advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}

```
</td> <td >

``` cpp
#include "ros/ros.h"
#include "std_msgs/String.h"


void chatterCallback(const std_msgs::String::ConstPtr &msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");

  
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
// 
  ros::spin();

  return 0;
}

### Python example

```
</td> </tr> </table>

<table> <tr > <td > 

#### Publisher
</td> <td > 

#### Subscriber
</td></tr>
<tr > <td > 

```python 

#!/usr/bin/env python
# importing rospy and string from std_msgs
import rospy
from std_msgs.msg import String

#defining a publisher that will publish hello world string along with time. 
def publisher():
    
    #create a publisher with the name simple_publisher that will publish 
    #the messages of type string and buffered 10 messages.
    pub = rospy.Publisher('simple_publisher', String, queue_size=10)
    
    #initialize a topic(rosnode) with the name talker
    rospy.init_node('sender', anonymous=True)
    
    #specifying the frequency of publishing the message.
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        #create a string message to publish
        message_str = "hello world %s" % rospy.get_time()
        #print the message (that will be published). 
        rospy.loginfo(message_str)
        #publish the string message.
        pub.publish(message_str)
        #call ros rate sleep.
        rate.sleep()
#main function
if __name__ == '__main__':
    try:
        #start publisher
        publisher()
    except rospy.ROSInterruptException:
        pass

```
</td> <td >

``` cpp
#!/usr/bin/env python
# importing rospy and string from std_msgs
import rospy
from std_msgs.msg import String
 
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
     
def receiver():
    
    #initialize ros node with the name receiver.
    # The anonymous=True flag means that rospy will choose a unique
    # name for our 'receiver' node so that multiple receiver can
    # run simultaneously.
    rospy.init_node('receiver', anonymous=True)

    #subscribe to sender node.
    rospy.Subscriber("/simple_publisher", String, callback)
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
 
if __name__ == '__main__':
    receiver()

```
</td> </tr> </table>

# Publisher with Rviz

![Alt Text](images/publisherEx2.gif)



```cpp
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <unistd.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "marker_visualizer");
    ros::NodeHandle n;
    std::string path_to_models = "package://publisher_subscriber_rviz/model/sphere.stl";
    ros::Publisher sphere_publisher = n.advertise<visualization_msgs::MarkerArray>("sphere_marker", 1);
    ros::Publisher line_publisher = n.advertise<visualization_msgs::Marker>("line_marker", 1);


    std::vector<double> pose;
    std::vector<double> color;
    std::vector<geometry_msgs::Point> line_points;

    ros::Rate r(10);
    for (uint32_t i = 0; i < 110; ++i)
    {
        geometry_msgs::Point p;
        float x = 2 * sin(i / 100.0f * 2 * M_PI);
        float y = 2 * cos(i / 100.0f * 2 * M_PI);
        p.x = x;
        p.y = y;
        p.z = 1.0;
        line_points.push_back(p);
    }

    while (ros::ok())
    {

        for (uint32_t i = 0; i < 100; ++i)
        {
            visualization_msgs::MarkerArray spheres;
            visualization_msgs::Marker line;

            float x = 2 * sin(i / 100.0f * 2 * M_PI);
            float y = 2 * cos(i / 100.0f * 2 * M_PI);
            pose = {x, y, 1.0, 0.0, 0.0, 0.0, 1.0};
            color = {0.0, 1.0, 0.0, 1.0};
            spheres.markers.push_back(createSphere(path_to_models, 1, "source_frame", pose, color));

            x = 2 * sin((i + 33) / 100.0f * 2 * M_PI);
            y = 2 * cos((i + 33) / 100.0f * 2 * M_PI);
            pose = {x, y, 1.0, 0.0, 0.0, 0.0, 1.0};
            color = {1.0, 0.0, 0.0, 1.0};
            spheres.markers.push_back(createSphere(path_to_models, 2, "source_frame", pose, color));

            x = 2 * sin((i + 66) / 100.0f * 2 * M_PI);
            y = 2 * cos((i + 66) / 100.0f * 2 * M_PI);
            pose = {x, y, 1.0, 0.0, 0.0, 0.0, 1.0};
            color = {0.0, 0.0, 1.0, 1.0};
            spheres.markers.push_back(createSphere(path_to_models, 3, "source_frame", pose, color));

            color = {1.0, 1.0, 1.0, 1.0};
            line = createLine(1, "source_frame", line_points, color);
            line_publisher.publish(line);
            sphere_publisher.publish(spheres);
            r.sleep();
        }
    }
}
```
</td> </tr> </table>

<table> <tr > <td > 

#### Publisher
</td> <td > 

#### Subscriber
</td></tr>
<tr > <td > 

```cpp 

visualization_msgs::Marker createSphere(std::string path_to_model, 
    unsigned int id, std::string frame_id,
    std::vector<double> pose, std::vector<double> color)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "sphere";
    marker.id = id;
    marker.mesh_resource = path_to_model;

    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = pose[0];
    marker.pose.position.y = pose[1];
    marker.pose.position.z = pose[2];
    marker.pose.orientation.x = pose[3];
    marker.pose.orientation.y = pose[4];
    marker.pose.orientation.z = pose[5];
    marker.pose.orientation.w = pose[6];
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = color[3];
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    return marker;
}

```
</td> <td >

``` cpp
visualization_msgs::Marker createLine(unsigned int id, 
std::string frame_id, std::vector<geometry_msgs::Point> points, 
                    std::vector<double> color)
{
    visualization_msgs::Marker line;
    line.header.frame_id = frame_id;
    line.header.stamp = ros::Time::now();
    line.ns = "line";
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.id = id;

    line.type = visualization_msgs::Marker::LINE_STRIP;

    line.scale.x = 0.05;
    line.color.r = color[0];
    line.color.g = color[1];
    line.color.b = color[2];
    line.color.a = color[3];

    for (auto &point : points)
    {
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 1;

        line.points.push_back(p);
    }

    return line;
}
```
</td> </tr> </table>



