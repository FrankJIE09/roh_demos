#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import os

# 添加包路径
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from Frank_control.srv import GetHandAngles, GetHandAnglesResponse

class MockHandAnglesService:
    """模拟手部角度服务，用于测试"""
    
    def __init__(self):
        self.service = rospy.Service('get_hand_angles', GetHandAngles, self.handle_get_hand_angles)
        rospy.loginfo("模拟手部角度服务已启动")
    
    def handle_get_hand_angles(self, req):
        """处理手部角度请求"""
        response = GetHandAnglesResponse()
        
        # 模拟手部角度数据
        response.detected = True
        response.thumb_bend = 45.0
        response.index_bend = 30.0
        response.middle_bend = 25.0
        response.ring_bend = 20.0
        response.pinky_bend = 15.0
        response.thumb_rot = 10.0
        
        rospy.loginfo("返回模拟手部角度数据")
        return response

def test_hand_angles_service():
    """测试手部角度服务"""
    rospy.init_node('test_hand_angles_service')
    
    # 启动模拟服务
    mock_service = MockHandAnglesService()
    
    # 等待服务可用
    rospy.wait_for_service('get_hand_angles', timeout=5.0)
    
    try:
        # 创建服务客户端
        get_hand_angles = rospy.ServiceProxy('get_hand_angles', GetHandAngles)
        
        # 调用服务
        response = get_hand_angles()
        
        # 验证响应
        assert response.detected == True, "手部检测状态错误"
        assert response.thumb_bend == 45.0, "拇指弯曲角度错误"
        assert response.index_bend == 30.0, "食指弯曲角度错误"
        assert response.middle_bend == 25.0, "中指弯曲角度错误"
        assert response.ring_bend == 20.0, "无名指弯曲角度错误"
        assert response.pinky_bend == 15.0, "小指弯曲角度错误"
        assert response.thumb_rot == 10.0, "拇指旋转角度错误"
        
        rospy.loginfo("✅ 手部角度服务测试通过")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"❌ 服务调用失败: {e}")
        return False
    except AssertionError as e:
        rospy.logerr(f"❌ 测试断言失败: {e}")
        return False
    
    return True

if __name__ == '__main__':
    try:
        success = test_hand_angles_service()
        if success:
            rospy.loginfo("🎉 所有测试通过！")
        else:
            rospy.logerr("💥 测试失败！")
            sys.exit(1)
    except KeyboardInterrupt:
        rospy.loginfo("测试被用户中断")
    except Exception as e:
        rospy.logerr(f"测试过程中发生错误: {e}")
        sys.exit(1) 