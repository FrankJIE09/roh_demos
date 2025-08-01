#pragma once
#include <sage_block/BlockBase.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "Frank_control/GetHandAngles.h" 

using namespace sage_block;
namespace sage_block_utility_hand{
class GetHandAnglesBlock: public sage_block::BlockBase{

    ros::ServiceClient client;

    public:
    bool onInit() override{
        ros::NodeHandle n;
        client = n.serviceClient<Frank_control::GetHandAngles>("get_hand_angles");
        return true;
    }

    void layoutBuild() override{
        //设置blocklyMessage
        block_json->setMessage("手部角度获取：检测状态 %1 拇指弯曲 %2 食指弯曲 %3 中指弯曲 %4 无名指弯曲 %5 小指弯曲 %6 拇指旋转 %7");
        //设置输出格式
        block_json->setInputsInline(false);
        //设置颜色
        block_json->setColour(120); // 使用绿色表示手部相关
    }
    
    PortsList providedArgs() override{
        return {
            InputValue("detected", "bool"),
            InputValue("thumb_bend", "num"),
            InputValue("index_bend", "num"),
            InputValue("middle_bend", "num"),
            InputValue("ring_bend", "num"),
            InputValue("pinky_bend", "num"),
            InputValue("thumb_rot", "num")
        };
    }

    BlockState executeOnce() override{
        // 创建服务请求
        Frank_control::GetHandAngles hand_angles;
        
        // 调用服务
        if(!client.call(hand_angles))
        {
            ROS_ERROR("手部角度获取服务调用失败");
            return BlockState::FAILED;
        }
        
        // 成功后将反馈结果返回
        writeArg<bool>("detected", hand_angles.response.detected);
        writeArg<float>("thumb_bend", hand_angles.response.thumb_bend);
        writeArg<float>("index_bend", hand_angles.response.index_bend);
        writeArg<float>("middle_bend", hand_angles.response.middle_bend);
        writeArg<float>("ring_bend", hand_angles.response.ring_bend);
        writeArg<float>("pinky_bend", hand_angles.response.pinky_bend);
        writeArg<float>("thumb_rot", hand_angles.response.thumb_rot);
        
        return BlockState::SUCCEEDED;
    }
};

} 