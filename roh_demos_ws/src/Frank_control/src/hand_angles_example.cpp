#include <ros/ros.h>
#include "Frank_control/GetHandAnglesBlock.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hand_angles_example");
    ros::NodeHandle nh;
    
    // 创建GetHandAnglesBlock实例
    sage_block_utility_hand::GetHandAnglesBlock hand_angles_block;
    
    // 初始化block
    if (!hand_angles_block.onInit()) {
        ROS_ERROR("手部角度获取block初始化失败");
        return -1;
    }
    
    // 构建布局
    hand_angles_block.layoutBuild();
    
    ROS_INFO("手部角度获取block已启动，等待服务调用...");
    
    // 主循环
    ros::Rate rate(10); // 10Hz
    while (ros::ok()) {
        // 执行block
        sage_block::BlockState state = hand_angles_block.executeOnce();
        
        if (state == sage_block::BlockState::SUCCEEDED) {
            ROS_INFO("手部角度获取成功");
        } else if (state == sage_block::BlockState::FAILED) {
            ROS_WARN("手部角度获取失败");
        }
        
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
} 