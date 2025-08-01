# GetHandAnglesBlock 低代码开发方式

## 概述

`GetHandAnglesBlock` 是一个基于 sage_block 框架的低代码开发组件，用于获取手部角度数据。它封装了 `GetHandAngles.srv` 服务，提供了简单易用的接口。

## 文件结构

```
Frank_control/
├── include/Frank_control/
│   └── GetHandAnglesBlock.h          # 头文件
├── src/
│   ├── GetHandAnglesBlock.cpp        # 实现文件
│   └── hand_angles_example.cpp       # 使用示例
├── srv/
│   └── GetHandAngles.srv             # 服务定义
└── README_GetHandAnglesBlock.md      # 说明文档
```

## 功能特性

- **低代码开发**：基于 sage_block 框架，简化开发流程
- **服务封装**：自动处理 ROS 服务调用
- **多参数输出**：支持获取所有手部角度参数
- **错误处理**：内置错误检测和状态反馈

## 输出参数

| 参数名 | 类型 | 描述 |
|--------|------|------|
| detected | bool | 手部检测状态 |
| thumb_bend | float32 | 拇指弯曲角度 |
| index_bend | float32 | 食指弯曲角度 |
| middle_bend | float32 | 中指弯曲角度 |
| ring_bend | float32 | 无名指弯曲角度 |
| pinky_bend | float32 | 小指弯曲角度 |
| thumb_rot | float32 | 拇指旋转角度 |

## 使用方法

### 1. 基本使用

```cpp
#include "Frank_control/GetHandAnglesBlock.h"

// 创建实例
sage_block_utility_hand::GetHandAnglesBlock hand_angles_block;

// 初始化
if (hand_angles_block.onInit()) {
    // 构建布局
    hand_angles_block.layoutBuild();
    
    // 执行获取
    sage_block::BlockState state = hand_angles_block.executeOnce();
    
    if (state == sage_block::BlockState::SUCCEEDED) {
        // 获取成功，可以读取输出参数
        bool detected;
        float thumb_bend, index_bend, middle_bend, ring_bend, pinky_bend, thumb_rot;
        
        // 读取参数（需要实现相应的读取方法）
        // hand_angles_block.readArg<bool>("detected", detected);
        // hand_angles_block.readArg<float>("thumb_bend", thumb_bend);
        // ... 其他参数
    }
}
```

### 2. 在 sage_block 框架中使用

在 Blockly 界面中，该组件会显示为：
```
手部角度获取：检测状态 [detected] 拇指弯曲 [thumb_bend] 食指弯曲 [index_bend] 
中指弯曲 [middle_bend] 无名指弯曲 [ring_bend] 小指弯曲 [pinky_bend] 拇指旋转 [thumb_rot]
```

## 编译配置

确保在 `CMakeLists.txt` 中包含以下配置：

```cmake
# 添加可执行文件
add_executable(hand_angles_example src/hand_angles_example.cpp)

# 添加依赖
add_dependencies(hand_angles_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 链接库
target_link_libraries(hand_angles_example ${catkin_LIBRARIES})
```

## 服务要求

该组件需要 `get_hand_angles` 服务可用。确保相关的手部检测服务正在运行：

```bash
# 启动手部检测服务（具体命令根据实际服务调整）
rosrun your_hand_detection_package hand_detection_node
```

## 注意事项

1. **服务可用性**：使用前确保 `get_hand_angles` 服务正在运行
2. **错误处理**：组件会自动处理服务调用失败的情况
3. **性能考虑**：建议控制调用频率，避免过度频繁的服务请求
4. **依赖关系**：需要 sage_block 框架支持

## 扩展开发

如需扩展功能，可以：

1. 修改 `layoutBuild()` 方法来自定义 Blockly 界面
2. 在 `executeOnce()` 中添加额外的处理逻辑
3. 扩展 `providedArgs()` 来添加更多输入/输出参数

## 故障排除

### 常见问题

1. **服务调用失败**
   - 检查 `get_hand_angles` 服务是否正在运行
   - 使用 `rosservice list` 确认服务名称

2. **编译错误**
   - 确保所有依赖包已正确安装
   - 检查 CMakeLists.txt 配置

3. **运行时错误**
   - 检查 ROS 环境变量设置
   - 确认 sage_block 框架正确安装 