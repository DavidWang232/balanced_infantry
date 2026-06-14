# JLUI 绘图移植指南

## 核心问题

原 `jlui_ui_task` 在 while 循环中每次调用 `JLUI_CreateString()` 创建新元素，导致：

- 内存堆积（重复创建未释放）
- UI元素不断增加
- 任务可能崩溃或卡顿

## 正确的实现模式

### 参考工程：H7_below_485_new_3.24_lms 的做法

#### 初始化阶段（for 循环之前）

```c
// 【关键】所有UI元素在此初始化，只创建一次！
Uiid HP_Dec = JLUI_CreateString(4, UiMagenta, 0, 750, 700, 40, "Please Spin");
osDelay(3);  // 给JLUI处理时间

Uiid VisionFound = JLUI_CreateCircle(5, UiTeam, 0, 960, 538, 200);
osDelay(3);

Uiid headstock = JLUI_CreateArc(5, UiMagenta, 0, 960, 538, 280, 280, 0, 0);
osDelay(3);

Uiid VisionMode = JLUI_CreateInt(5, UiBlack, 0, 310, 600, 20, vision_mode);
osDelay(3);
```

#### 循环阶段（for 循环内）

```c
for (;;) {
    // 【关键】循环中只做更新，不再创建元素！
    
    // 10Hz 心跳
    if (tick_count % 10 == 0) {
        JLUI_10HzTick();
    }
    
    // 更新可见性
    if (condition) {
        JLUI_SetVisible(HP_Dec, 1);
    } else {
        JLUI_SetVisible(HP_Dec, 0);
    }
    
    // 更新数值
    JLUI_SetInt(VisionMode, current_vision_mode);
    
    // 更新位置/角度等
    JLUI_SetStartAngle(headstock, angle_start);
    JLUI_SetEndAngle(headstock, angle_end);
    
    osDelay(10);
}
```

## 完整的 jlui_ui_task 模板

```c
void jlui_ui_task(void const * argument)
{
    // ========== 初始化 S ==========
    
    // 必须首先设置发送者/接收者ID
    JLUI_SetSenderReceiverId(1, 0x0101);
    
    // 创建互斥锁
    void * jlui_mutex = xSemaphoreCreateMutex();
    if (jlui_mutex == NULL) {
        jlui_mutex = (void *)1;
    }
    JLUI_SetMutexObject(jlui_mutex);
    
    // 【关键】一次性创建所有UI元素，保存ID供后续更新使用
    Uiid HP_Dec = JLUI_CreateString(4, UiMagenta, 0, 750, 700, 40, "Please Spin");
    osDelay(3);
    
    Uiid VisionFound = JLUI_CreateCircle(5, UiTeam, 0, 960, 538, 200);
    osDelay(3);
    
    Uiid headstock = JLUI_CreateArc(5, UiMagenta, 0, 960, 538, 280, 280, 0, 0);
    osDelay(3);
    
    Uiid VisionMode = JLUI_CreateInt(5, UiBlack, 0, 310, 600, 20, 0);
    osDelay(3);
    
    // ========== 初始化 E ==========
    
    // ========== 循环 S ==========
    uint32_t tick_counter = 0;
    uint8_t ui_time = 100;
    
    for (;;) {
        tick_counter++;
        
        // 100ms 调用一次 JLUI_10HzTick
        if (tick_counter >= 10) {
            tick_counter = 0;
            JLUI_10HzTick();
        }
        
        // 动态调整延迟（可选）
        osDelay(ui_time);
        if (ui_time > 35) {
            ui_time -= 2;
        }
        
        // --------- 在此处更新各个UI元素 ---------
        
        // 1. 根据条件显示/隐藏
        if (condition_for_spin) {
            JLUI_SetVisible(HP_Dec, 1);
        } else {
            JLUI_SetVisible(HP_Dec, 0);
        }
        
        // 2. 更新数值显示
        JLUI_SetInt(VisionMode, get_current_vision_mode());
        
        // 3. 更新位置/旋转角度（对于容器坐标，需要在循环中更新）
        float angle_start = chassis_angle - 15;
        float angle_end = chassis_angle + 15;
        JLUI_SetStartAngle(headstock, angle_start);
        JLUI_SetEndAngle(headstock, angle_end);
        
        // 4. 条件显示
        if (vision_found_flag) {
            JLUI_SetVisible(VisionFound, 1);
        } else {
            JLUI_SetVisible(VisionFound, 0);
        }
    }
    // ========== 循环 E ==========
}
```

## 关键要点

| 特性 | 说明 |
|------|------|
| **JLUI_SetSenderReceiverId()** | 【必须首先调用】在任何其他JLUI API之前调用 |
| **互斥锁** | 创建后立即传给JLUI库，参考工程禁用了实际的锁机制 |
| **创建元素** | 只在初始化阶段调用`JLUI_Create*()`，保存返回的ID |
| **更新元素** | 循环中使用`JLUI_Set*()`函数更新，如`JLUI_SetInt()`、`JLUI_SetVisible()`等 |
| **JLUI_10HzTick()** | 必须定期调用（每100ms一次），用于刷新显示 |
| **osDelay()** | 不可在中断中调用这些函数，必须在任务中调用 |

## 常用的更新函数

```c
// 显示/隐藏
JLUI_SetVisible(uiid, visible);

// 更新位置
JLUI_MoveTo(uiid, x, y);

// 更新数值（整数）
JLUI_SetInt(uiid, value);

// 更新浮点数值
JLUI_SetFloat(uiid, value);

// 更新字符串
JLUI_SetString(uiid, str);

// 更新颜色
JLUI_SetColor(uiid, color);

// 更新圆弧起始角度和结束角度
JLUI_SetStartAngle(uiid, angle);
JLUI_SetEndAngle(uiid, angle);

// 删除单个元素
JLUI_Delete(uiid);

// 删除所有元素（仅在需要重置时使用，参考工程中在UI_RES==1时使用）
JLUI_DeleteAll();
```

## 调试技巧

1. **检查ID保存** - 确保 `JLUI_Create*()` 返回的ID被正确保存
2. **检查循环结构** - 确保有 `for(;;)` 或 `while(1)` 的无限循环
3. **检查osDelay()** - 不要忘记循环中的 `osDelay()`，否则会占用过多CPU
4. **逐步创建UI** - 初期可以创建较少的UI元素测试，后续逐步增加
5. **查看串口输出** - 通过UART验证JLUI数据是否正确发送到裁判系统

## 从 referee_usart_task.c 迁移步骤

如果你之前在 referee_usart_task.c 中绘图：

1. **复制UI创建代码** - 把原来分散在各处的 `JLUI_Create*()` 调用集中到 `jlui_ui_task` 的初始化部分
2. **保存所有ID** - 改为在 `jlui_ui_task` 中声明全局或局部变量保存所有ID
3. **移动更新逻辑** - 把更新UI的逻辑移到 `jlui_ui_task` 的循环部分
4. **测试** - 编译烧录验证UI正常显示且不会卡顿

## 参考代码位置

- **参考工程示例**: `H7_below_485_new_3.24_lms/H7_below_485/freeRTOS/UI/source/jlui_port_freertos.c`
  - 展示了完整的UI任务实现
  - 包括初始化、循环更新、条件显示等
- **当前项目**: `StandardRobotpp-master/application/referee/referee_usart_task.c` (已修复)
