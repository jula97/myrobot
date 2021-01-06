/* 
Node to convert the IR range data to cliff data.
IR ranges higher than cliff_mark_threshold are replaced with cliff_mark_distance.
IR ranges lower than cliff_mark_threshold are replaced with a static value.  

Subscribes to topics publishing range data and publishes the cliff data to respective topics.
*/

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int32.h>

ros::Publisher pub_cliff;
ros::Publisher pub_cliff_back;
ros::Publisher pub_cliff_left_1;
ros::Publisher pub_cliff_right_1;
ros::Publisher pub_cliff_left_2;
ros::Publisher pub_cliff_right_2;
double cliff_mark_threshold;
double cliff_mark_distance;

//IR range data to cliff data conversion
float ir_to_cliff(double range){
    if (range>cliff_mark_threshold)
    {
        return cliff_mark_distance;
    }
    else
    {
        return 10;
    }
}

void clbk_ir_front(const sensor_msgs::Range::ConstPtr& msg){
    ROS_INFO("Range received");

    sensor_msgs::Range cliff_range;
 
    cliff_range.header.stamp = ros::Time::now();
    cliff_range.header.frame_id = msg->header.frame_id;
    cliff_range.radiation_type = msg->radiation_type;
    cliff_range.field_of_view = msg->field_of_view;
    cliff_range.min_range = msg->min_range;
    cliff_range.max_range = msg->max_range;
    cliff_range.range = ir_to_cliff(msg->range);

    pub_cliff.publish(cliff_range);
}
void clbk_ir_back(const sensor_msgs::Range::ConstPtr& msg){
    ROS_INFO("Range received");

    sensor_msgs::Range cliff_range;
 
    cliff_range.header.stamp = ros::Time::now();
    cliff_range.header.frame_id = msg->header.frame_id;
    cliff_range.radiation_type = msg->radiation_type;
    cliff_range.field_of_view = msg->field_of_view;
    cliff_range.min_range = msg->min_range;
    cliff_range.max_range = msg->max_range;
    cliff_range.range = ir_to_cliff(msg->range);

    pub_cliff_back.publish(cliff_range);
}
void clbk_ir_left_1(const sensor_msgs::Range::ConstPtr& msg){
    ROS_INFO("Range received");

    sensor_msgs::Range cliff_range;
 
    cliff_range.header.stamp = ros::Time::now();
    cliff_range.header.frame_id = msg->header.frame_id;
    cliff_range.radiation_type = msg->radiation_type;
    cliff_range.field_of_view = msg->field_of_view;
    cliff_range.min_range = msg->min_range;
    cliff_range.max_range = msg->max_range;
    cliff_range.range = ir_to_cliff(msg->range);

    pub_cliff_left_1.publish(cliff_range);
}
void clbk_ir_right_1(const sensor_msgs::Range::ConstPtr& msg){
    ROS_INFO("Range received");

    sensor_msgs::Range cliff_range;
 
    cliff_range.header.stamp = ros::Time::now();
    cliff_range.header.frame_id = msg->header.frame_id;
    cliff_range.radiation_type = msg->radiation_type;
    cliff_range.field_of_view = msg->field_of_view;
    cliff_range.min_range = msg->min_range;
    cliff_range.max_range = msg->max_range;
    cliff_range.range = ir_to_cliff(msg->range);

    pub_cliff_right_1.publish(cliff_range);
}
void clbk_ir_left_2(const sensor_msgs::Range::ConstPtr& msg){
    ROS_INFO("Range received");

    sensor_msgs::Range cliff_range;
 
    cliff_range.header.stamp = ros::Time::now();
    cliff_range.header.frame_id = msg->header.frame_id;
    cliff_range.radiation_type = msg->radiation_type;
    cliff_range.field_of_view = msg->field_of_view;
    cliff_range.min_range = msg->min_range;
    cliff_range.max_range = msg->max_range;
    cliff_range.range = ir_to_cliff(msg->range);

    pub_cliff_left_2.publish(cliff_range);
}
void clbk_ir_right_2(const sensor_msgs::Range::ConstPtr& msg){
    ROS_INFO("Range received");

    sensor_msgs::Range cliff_range;
 
    cliff_range.header.stamp = ros::Time::now();
    cliff_range.header.frame_id = msg->header.frame_id;
    cliff_range.radiation_type = msg->radiation_type;
    cliff_range.field_of_view = msg->field_of_view;
    cliff_range.min_range = msg->min_range;
    cliff_range.max_range = msg->max_range;
    cliff_range.range = ir_to_cliff(msg->range);

    pub_cliff_right_2.publish(cliff_range);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "cliff_data");

    ros::NodeHandle n;
    ros::NodeHandle nh;
    nh.getParam("cliff_mark_threshold", cliff_mark_threshold);    
    nh.getParam("cliff_mark_distance", cliff_mark_distance);

    ros::Subscriber sub_ir =n.subscribe("/sensor/cliff_ir_front", 1, clbk_ir_front);
    ros::Subscriber sub_ir_back =n.subscribe("/sensor/cliff_ir_back", 1, clbk_ir_back);
    ros::Subscriber sub_ir_left_1 =n.subscribe("/sensor/cliff_ir_left_1", 1, clbk_ir_left_1);
    ros::Subscriber sub_ir_right_1 =n.subscribe("/sensor/cliff_ir_right_1", 1, clbk_ir_right_1);
    ros::Subscriber sub_ir_left_2 =n.subscribe("/sensor/cliff_ir_left_2", 1, clbk_ir_left_2);
    ros::Subscriber sub_ir_right_2 =n.subscribe("/sensor/cliff_ir_right_2", 1, clbk_ir_right_2);

    pub_cliff = n.advertise<sensor_msgs::Range>("/sensor/cliff_range_front", 1);
    pub_cliff_back = n.advertise<sensor_msgs::Range>("/sensor/cliff_range_back", 1);
    pub_cliff_left_1 = n.advertise<sensor_msgs::Range>("/sensor/cliff_range_left_1", 1);
    pub_cliff_right_1 = n.advertise<sensor_msgs::Range>("/sensor/cliff_range_right_1", 1);
    pub_cliff_left_2 = n.advertise<sensor_msgs::Range>("/sensor/cliff_range_left_2", 1);
    pub_cliff_right_2 = n.advertise<sensor_msgs::Range>("/sensor/cliff_range_right_2", 1);

    ros::spin();

}
