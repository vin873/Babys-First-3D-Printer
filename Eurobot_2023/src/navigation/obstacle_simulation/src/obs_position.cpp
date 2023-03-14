#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "obstacle_simulation/obs_position.h"



double generateGaussianNoise(double mu, double sigma)
{
    const double epsilon = std::numeric_limits<double>::min();
    const double two_pi = 2.0*3.14159265358979323846;

    static double z0, z1;
    static bool generate;
    generate = !generate;

    if (!generate)
       return z1 * sigma + mu;

    double u1, u2;
    do
    {
      u1 = rand() * (1.0 / RAND_MAX);
      u2 = rand() * (1.0 / RAND_MAX);
    }
    while ( u1 <= epsilon );

    z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
    z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
    return z0 * sigma + mu;
}

void obstaclePub(ros::Publisher pub, std::string frame, std::vector<std::vector<double>> obstacle_pos){
    int obstacle_num = obstacle_pos.size();
    for(int i = 0; i < obstacle_num; i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = frame; 
        pose.pose.position.x = obstacle_pos[i][0];
        pose.pose.position.y = obstacle_pos[i][1];
        pub.publish(pose);
    }
}

void obstaclePubArray(ros::Publisher pub, std::string frame, std::vector<std::vector<double>> obstacle_pos){
    int obstacle_num = obstacle_pos.size();
    geometry_msgs::PoseArray poses;
    poses.header.frame_id = frame;
    poses.header.stamp = ros::Time::now();

    for(int i = 0; i < obstacle_num; i++)
    {     
        geometry_msgs::Pose p;
        p.position.x = obstacle_pos[i][0];
        p.position.y = obstacle_pos[i][1];
        poses.poses.push_back(p);
    }
    pub.publish(poses);
}

/**
 *  @brief: pubish all types of obstacles 
 *  @param: std::vector<std::vector<double>> obstacles : all the obstacles information from YAML file
 */ 
void obstaclePub(ros::Publisher pub, std::vector<Obstacle> obstacles)
{
    for(int i=0; i<obstacles.size(); i++)
    {
        geometry_msgs::PoseArray poses;
        poses.header.frame_id = obstacles[0].from;
        poses.header.stamp = ros::Time::now();
        switch(obstacles[i].motion_type){
            case Motion_type::STATIC:
                for(int j = 0; j<obstacles[0].position.size(); j++)
                {
                    geometry_msgs::Pose p;
                    p.position.x = generateGaussianNoise(obstacles[i].position[j][0], obstacles[i].stdev);
                    p.position.y = generateGaussianNoise(obstacles[i].position[j][1], obstacles[i].stdev);
                    poses.poses.push_back(p);
                }
                pub.publish(poses);
                break;
            case Motion_type::RECIPROCATION:
                break;
        }
    }
}


/** @brief Parse a vector of vector of floats from a string.
 * @param input
 * @param error_return
 * Syntax is [[1.0, 2.0], [3.3, 4.4, 5.5], ...] */
std::vector<std::vector<double>> parseVVF(const std::string & input, std::string & error_return)
{
    std::vector<std::vector<double>> result;

    std::stringstream input_ss(input);
    int depth = 0;
    std::vector<double> current_vector;
    while (!!input_ss && !input_ss.eof()) {
        switch (input_ss.peek()) {
            case EOF:
                break;
            case '[':
                depth++;
                if (depth > 2) {
                    error_return = "Array depth greater than 2";
                    return result;
                }
                input_ss.get();
                current_vector.clear();
                break;
            case ']':
                depth--;
                if (depth < 0) {
                    error_return = "More close ] than open [";
                    return result;
                }
                input_ss.get();
                if (depth == 1) {
                    result.push_back(current_vector);
                }
                break;
            case ',':
            case ' ':
            case '\t':
                input_ss.get();
                break;
                default:  // All other characters should be part of the numbers.
                if (depth != 2) {
                    std::stringstream err_ss;
                    err_ss << "Numbers at depth other than 2. Char was '" << char(input_ss.peek()) << "'.";
                    error_return = err_ss.str();
                    return result;
                }
                float value;
                input_ss >> value;
                if (!!input_ss) {
                    current_vector.push_back(value);
                }
                break;
        }
    }
    if (depth != 0) {
        error_return = "Unterminated vector string.";
    } else {
        error_return = "";
    }
    return result;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "obstacle_position");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Publisher obs_pub_PoseStamped = nh.advertise<geometry_msgs::PoseStamped>("obstacle_position",1000);
    ros::Publisher obs_pub_PoseArray = nh.advertise<geometry_msgs::PoseArray>("obstacle_position_array",1000);
    // ros::Publisher obs_pub_all_PoseArray = nh.advertise<geometry_msgs::PoseArray>("obstacle_all_position_array1",1000);


    // read the YAML file
    int update_frequency;
    private_nh.param("update_frequency", update_frequency, 10);
    ros::Rate loop_rate(update_frequency);

    std::vector<Obstacle> obstacles;
    if (private_nh.hasParam("obstacles"))
    {
        XmlRpc::XmlRpcValue my_list;
        private_nh.getParam("obstacles", my_list);
        for (int32_t i = 0; i < my_list.size(); ++i)
        {
            Obstacle obs;
            obs.from = static_cast<std::string>(my_list[i]["type"]);
            ROS_INFO("Obstacle Simulation: Using Sources \"%s\"", obs.from.c_str());

            std::string type = static_cast<std::string>(my_list[i]["motion_type"]);
            if(type == "static")
                obs.motion_type = Motion_type::STATIC;
            else if(type == "reciprocation")
                obs.motion_type = Motion_type::RECIPROCATION;

            std::string pos, error_return;
            pos = static_cast<std::string>(my_list[i]["position"]);
            obs.position = parseVVF(pos, error_return);
            obs.stdev = static_cast<double>(my_list[i]["stdev"]);
            if(error_return!= "")
                ROS_ERROR("obstacle_simulation: error_return %s", error_return.c_str());
            else
                obstacles.push_back(obs);

        }
    }

    double obstacle_num = 0;

    while(ros::ok())
    {
        // obstaclePub(obs_pub_PoseStamped, frame, obstacle_pos);
        // obstaclePubArray(obs_pub_PoseArray, frame, obstacle_pos);
        obstaclePub(obs_pub_PoseArray, obstacles);
        ros::spinOnce();
        loop_rate.sleep();
    }

}