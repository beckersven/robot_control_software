#include "../include/laser_scanner.h"
#include "geometry_msgs/TransformStamped.h"
#include <iostream>
#include <geometry_msgs/Point32.h>
#include <cmath>
namespace coppeliasim_interface{
    LaserScanner::LaserScanner(){
        client_id = -1;
        distances.resize(0);
        resolution = 0;
        angle = 0;
        
    }
    LaserScanner::~LaserScanner(){
        if(client_id != -1){
            simxFinish(client_id);
            ROS_DEBUG_STREAM("Disconnected from CoppeliaSim");
        }
        delete measurements;
    }
    bool LaserScanner::init(ros::NodeHandle& nh_){
        nh = nh_;
        simxInt coppelia_port, connection_timeout_ms;
        double noise_std_deviation;
        if(!nh.getParam("/coppelia_config/PORT_laser_scanner", coppelia_port) 
            || !nh.getParam("/coppelia_config/connection_timeout_ms", connection_timeout_ms)
            || !nh.getParam("/coppelia_config/laser_scanner_focus_frame_id", scanner_focus_frame_id) 
            || !nh.getParam("world_frame_id", world_frame_id)
            || !nh.getParam("resolution", resolution)
            || !nh.getParam("angle", angle)
            || !nh.getParam("noise_std_deviation", noise_std_deviation)
            || !nh.getParam("noise_intensity", noise_intensity)
            ){
            ROS_ERROR_STREAM("Could not resolve parameter " << nh.resolveName("/coppelia_config/PORT_laser_scanner") << 
                " or " <<  nh.resolveName("/coppelia_config/connection_timeout_ms") << 
                " or " <<  nh.resolveName("/coppelia_config/laser_scanner_focus_frame_id") << 
                " or " <<  nh.resolveName("world_frame_id") <<
                " or " <<  nh.resolveName("resolution") << 
                " or " <<  nh.resolveName("angle") << ". Laser scanner won't work."
                " or " <<  nh.resolveName("angle") << 
                " or " <<  nh.resolveName("noise_std_deviation") << 
                " or " <<  nh.resolveName("noise_intensity") << ". Laser scanner won't work."
            );
            return false;
        }
        measurements = new simxFloat[2 * resolution];
        measurements = new simxFloat[resolution];
        noise_distribution = std::normal_distribution<double>(0.0, noise_std_deviation);
        client_id = simxStart("127.0.0.1", coppelia_port, true, true, connection_timeout_ms, 5);
        if(client_id == -1){
            ROS_ERROR_STREAM("Could not connect to CoppeliaSim at 127.0.0.1:" << coppelia_port);
            return false;
        }
        if(simxGetObjectHandle(client_id, scanner_focus_frame_id.c_str(), &scanner_focus_handle, simx_opmode_blocking) != simx_return_ok){
            ROS_ERROR_STREAM("Can not resolve " << scanner_focus_frame_id << " in CoppeliaSim");
            return false;
        }
        simxFloat dummy[4];
        simxGetObjectPosition(client_id, scanner_focus_handle, -1, dummy, simx_opmode_streaming);
        simxGetObjectQuaternion(client_id, scanner_focus_handle, -1, dummy, simx_opmode_streaming);
        measurement_pub = nh.advertise<sensor_msgs::PointCloud>("/measured_pointcloud", 1, false);
        sense_srv = nh.advertiseService("/laser_scanner_sense", &LaserScanner::sense, this);
        ROS_DEBUG_STREAM("Laser-Scanner is ready");
        return true;
    }
    bool LaserScanner::sense(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res){
        simxFloat position[3], quaternion[4];
        if(!simxGetObjectPosition(client_id, scanner_focus_handle, -1, position, simx_opmode_buffer) == simx_return_ok
            || !simxGetObjectQuaternion(client_id, scanner_focus_handle, -1, quaternion, simx_opmode_buffer) == simx_return_ok){
                res.success = true;
                ROS_WARN_STREAM_THROTTLE(1.0, "Could not obtain scanner_focus-pose. This message must disappear shortly after start.");
                return true;
            }
        geometry_msgs::TransformStamped tf;
        tf.header.frame_id = world_frame_id;
        tf.header.stamp = ros::Time::now();
        tf.child_frame_id = scanner_focus_frame_id;
        tf.transform.translation.x = position[0];
        tf.transform.translation.y = position[1];
        tf.transform.translation.z = position[2];
        tf.transform.rotation.x = quaternion[0];
        tf.transform.rotation.y = quaternion[1];
        tf.transform.rotation.z = quaternion[2];
        tf.transform.rotation.w = quaternion[3];
        tf_broad.sendTransform(tf);
        simxInt integer_params[1] = { resolution };
        simxFloat float_params[1] = { angle };
        simxInt double_resolution = 2 * resolution;
        int result = simxCallScriptFunction(client_id, "laser_scanner", 1, "sense", 1, integer_params, 1, float_params, 0, NULL, 0, NULL, NULL, NULL, &resolution, &measurements, NULL, NULL, NULL, NULL, simx_opmode_blocking);
        if(!result == simx_return_ok) std::cout << "A" << result << std::endl;
        sensor_msgs::PointCloud measured_cloud;
        measured_cloud.header.stamp = ros::Time::now();
        measured_cloud.header.frame_id = scanner_focus_frame_id;
        std::vector<geometry_msgs::Point32> measured_points(0);
        double angle_current = -1 * angle * M_PI / 360;
        for(int i = 0; i < resolution; angle_current += angle / (resolution - 1) * M_PI / 180, i++){
            if(measurements[i] == -1) continue;
            geometry_msgs::Point32 to_add;
            measurements[i] += noise_distribution(noise_generator) * noise_intensity;
            to_add.x = std::cos(angle_current) * measurements[i];
            to_add.y = std::sin(angle_current) * measurements[i];
            to_add.z = 0;
            measured_cloud.points.push_back(to_add);
        }
        measurement_pub.publish(measured_cloud);
        res.success = true;
        return true;
    }
}