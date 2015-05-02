#include <string>
#include <utility> // std::pair

#include <boost/assign/list_of.hpp> // for 'list_of()'
#include <boost/assert.hpp> 

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include "aseba.h"
#include "odometry.h"

const double MAX_SPEED = .16; //m.s^-1 on the wheels for ranger2

static const int BATTERY_MAX_LEVEL = 8400;          //mV
static const int BATTERY_MIN_CHARGED_LEVEL = 8000;  //mV
static const int BATTERY_LOW_THRESHOLD = 7200;      //mV

using namespace std;


template <typename T>
T clamp(const T& n, const T& lower, const T& upper) {
      return std::max(lower, std::min(n, upper));
}

template <typename T>
double getClampRatio(const T& n, const T& lower, const T& upper) {
	  T clampedSpeed = std::max(lower, std::min(n, upper));
      return 1.0*clampedSpeed/n;
}

class Ranger {

public:

    RangerOdometry odom;
    float scaleValue;
    float voltage;

    Ranger(const string& aseba_target):aseba_node(aseba_target.c_str()) {
        // check whether connection was successful
        if (!aseba_node.isValid())
        {
            ROS_ERROR_STREAM("Could not connect to Aseba target " << aseba_target);
            exit(1);
        }
        else {ROS_INFO_STREAM("Connected to Aseba target " << aseba_target);}
    };

    void step() {
        aseba_node.Hub::step(); // check for incoming Aseba events

        scaleValue = aseba_node.scale;
        voltage  = aseba_node.voltage;

        if (aseba_node.is_charging) {
            odom.reset(0.35, 0., 0.);
        }
        else {
            odom.update(aseba_node.l_encoder, aseba_node.r_encoder);
        }
    }

    void set_speed(const geometry_msgs::Twist& msg) {

        pair<double, double> speeds = odom.twist_to_motors(msg.linear.x, msg.angular.z);
        pair<double, double> speedScaled;
        
        speedScaled.first = speeds.first * 100./ MAX_SPEED;
        speedScaled.second = speeds.second * 100./ MAX_SPEED;
        		
		double clampRatioL = getClampRatio<int>(speedScaled.first, -100, 100);
		double clampRatioR = getClampRatio<int>(speedScaled.second, -100, 100);
				
		double clampRatio =  std::min(clampRatioL, clampRatioR);
		
		int lspeed = round(speedScaled.first * clampRatio);
		int rspeed = round(speedScaled.second * clampRatio);
		
        aseba_node.setSpeed(lspeed, -rspeed);

    }

private:
    RangerAsebaBridge aseba_node;

};

int main(int argc, char** argv){
    ros::init(argc, argv, "ranger");

    ros::NodeHandle nh("~");
    string aseba_target = "";
    nh.getParam("aseba_target", aseba_target);

    Ranger ranger(aseba_target);

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Publisher scale_pub = n.advertise<std_msgs::Float64>("ranger_scale", 50);

    // TODO: future work - make /diagnostics (diagnostic_msgs/DiagnosticArray) topic instead
    // publishers for robot battery status - voltage and level
    ros::Publisher voltage_pub =    n.advertise<std_msgs::Float64>("voltage", 50);
    ros::Publisher bat_level_pub =  n.advertise<std_msgs::Float64>("battery_level", 50);

    tf::TransformBroadcaster odom_broadcaster;

    ros::Subscriber cmd_vel_sub = n.subscribe<const geometry_msgs::Twist&>("cmd_vel", 1, &Ranger::set_speed, &ranger);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(11.0); // Aseba low-level publishes encoders at ~10Hz

    double x, y, th, dx, dr;

    // scale and voltage values will be accumulated and published only at rate/5 ~2Hz
    int window_size = 5;
    int counter = 0;
    float voltage_mean = 0;
    float weight_mean = 0;

    while(n.ok()){

        ros::spinOnce();               // check for incoming messages
        ranger.step();

        current_time = ros::Time::now();

        x = ranger.odom.get_x();
        y = ranger.odom.get_y();
        th = ranger.odom.get_th();
        dx = ranger.odom.get_dx();
        dr = ranger.odom.get_dr();

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = dx;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = dr;
        
        //set the covariance
        odom.pose.covariance =  boost::assign::list_of(1e-3) (0) (0)  (0)  (0)  (0)
													   (0) (1e-3)  (0)  (0)  (0)  (0)
													   (0)   (0)  (1e6) (0)  (0)  (0)
													   (0)   (0)   (0) (1e6) (0)  (0)
													   (0)   (0)   (0)  (0) (1e6) (0)
													   (0)   (0)   (0)  (0)  (0)  (1e3) ;

        //publish the odom message
        odom_pub.publish(odom);

        //increase counter and take mod window size
        counter++;
        counter = counter % window_size;
        // accumulate weight and voltages
        weight_mean += ranger.scaleValue/window_size;
        voltage_mean += ranger.voltage/window_size;

        //prepare messages and publish if accumulated
        if (counter==0) {
            std_msgs::Float64 weight;
            std_msgs::Float64 voltage;
            std_msgs::Float64 bat_level;
            // set msg data
            weight.data = weight_mean;
            voltage.data = voltage_mean;

            bat_level.data = float(voltage.data - BATTERY_LOW_THRESHOLD) / (BATTERY_MAX_LEVEL - BATTERY_LOW_THRESHOLD);
            bat_level.data = (bat_level.data>1)?1:bat_level.data;

            //publish the scale (weight) message
            scale_pub.publish(weight);
            //publish battery voltage message
            voltage_pub.publish(voltage);


            bat_level_pub.publish(bat_level);

            // reset means
            voltage_mean = 0;
            weight_mean = 0;

        }

        last_time = current_time;
        r.sleep();
    }
}

