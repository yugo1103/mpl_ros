/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


// %Tag(fullSource)%
#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>


nav_msgs::Odometry start_pos_;
geometry_msgs::PoseStamped goal_pos_;

class MarkerNode
{
protected:
    MarkerNode(int argc, char **argv, const char *node_name)
    {
        ros::init(argc, argv, node_name);
    }
};

class Marker : MarkerNode
{
public:
    void startFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
    void goalFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
    bool start_serviceCallback(std_srvs::Empty::Request & /*request*/,std_srvs::Empty::Response & /*response*/);
    bool goal_serviceCallback(std_srvs::Empty::Request & /*request*/,std_srvs::Empty::Response & /*response*/);
 
    Marker(int argc, char **argv, const char *node_name) : MarkerNode(argc, argv, node_name)
    {
        start_pos_pub_ = nh.advertise<nav_msgs::Odometry>("start_pose", 10);
        goal_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>("goal_pose", 10);
        ros::ServiceServer start_service = nh.advertiseService("pub_start_pose", &Marker::start_serviceCallback, this);
        ros::ServiceServer goal_service = nh.advertiseService("pub_goal_pose", &Marker::goal_serviceCallback, this);
        

              // create an interactive marker server on the topic namespace simple_marker
        interactive_markers::InteractiveMarkerServer start_server("start_marker");
        interactive_markers::InteractiveMarkerServer goal_server("goal_marker");

        // create an interactive marker for our server
        visualization_msgs::InteractiveMarker int_marker;
        int_marker.header.frame_id = "world";
        int_marker.header.stamp=ros::Time::now();
        int_marker.name = "start_marker";
        int_marker.description = "start_pose";

        // create a grey box marker
        visualization_msgs::Marker box_marker;
        box_marker.type = visualization_msgs::Marker::CUBE;
        box_marker.scale.x = 0.3;
        box_marker.scale.y = 0.3;
        box_marker.scale.z = 0.3;
        box_marker.color.r = 0.0;
        box_marker.color.g = 1.0;
        box_marker.color.b = 0.0;
        box_marker.color.a = 1.0;

        // create a non-interactive control which contains the box
        visualization_msgs::InteractiveMarkerControl box_control;
        box_control.always_visible = true;
        box_control.markers.push_back( box_marker );

        // add the control to the interactive marker
        int_marker.controls.push_back( box_control );

        // create a control which will move the box
        // this control does not contain any markers,
        // which will cause RViz to insert two arrows
        visualization_msgs::InteractiveMarkerControl control;


        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        /***use rotatex
        control.name = "rotate_x";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        */
        control.name = "move_x";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        /***use rotatez
        control.name = "rotate_z";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        */
        control.name = "move_z";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        /***use rotatey
        control.name = "rotate_y";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        */
        control.name = "move_y";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        // add the interactive marker to our collection &
        // tell the server to call processFeedback() when feedback arrives for it
        start_server.insert(int_marker, boost::bind(&Marker::startFeedback, this, _1));




        int_marker.name = "goal_marker";
        int_marker.description = "goal_pose";


        box_marker.color.r = 1.0;
        box_marker.color.g = 0.0;
        box_marker.color.b = 0.0;

        box_control.markers.push_back( box_marker );

        // add the control to the interactive marker
        int_marker.controls.push_back( box_control );


        goal_server.insert(int_marker, boost::bind(&Marker::goalFeedback, this, _1));





        // 'commit' changes and send to all clients
        start_server.applyChanges();
        goal_server.applyChanges();

        // start the ROS main loop
        ros::spin();


    }
private:
    ros::NodeHandle nh;
    ros::Publisher start_pos_pub_;
    ros::Publisher goal_pos_pub_;
};


bool Marker::start_serviceCallback(std_srvs::Empty::Request & /*request*/,std_srvs::Empty::Response & /*response*/){
  start_pos_pub_.publish(start_pos_);
  ROS_INFO_STREAM("Recive Request!");       
  return true;
}

bool Marker::goal_serviceCallback(std_srvs::Empty::Request & /*request*/,std_srvs::Empty::Response & /*response*/){
  goal_pos_pub_.publish(goal_pos_);
  ROS_INFO_STREAM("Recive Request!");       
  return true;
}


void Marker::startFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{  
  start_pos_.header.frame_id = "world";
  start_pos_.pose.pose = feedback->pose; 
}


void Marker::goalFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{  
  goal_pos_.header.frame_id = "world";
  goal_pos_.pose = feedback->pose; 
}



int main(int argc, char** argv)
{
  Marker GoalMarker(argc, argv, "simple_marker");
}
// %Tag(fullSource)%
