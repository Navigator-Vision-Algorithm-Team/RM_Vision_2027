# Game Protocol Definition

## 1. GAME_STATUS (0x0001)

- **发送频率**：1 Hz
- **数据结构名称**：`game_status_t`

### 字段定义

| 字段名            | 类型     | 位宽   | 描述                   |
| ----------------- | -------- | ------ | ---------------------- |
| game_type         | uint8_t  | 4 bit  | 比赛类型               |
| game_progress     | uint8_t  | 4 bit  | 比赛阶段               |
| stage_remain_time | uint16_t | 16 bit | 当前阶段剩余时间（秒） |
| SyncTimeStamp     | uint64_t | 64 bit | 同步时间戳             |

### 结构体定义

```c
typedef struct {
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} __packed game_status_t;
```

------

## 2. GAME_ROBOT_STATUS (0x0201)

- **发送频率**：10 Hz
- **数据结构名称**：`game_robot_status_t`

### 字段定义

| 字段名                     | 类型     | 位宽   | 描述                 |
| -------------------------- | -------- | ------ | -------------------- |
| robot_id                   | uint8_t  | 8 bit  | 机器人ID             |
| robot_level                | uint8_t  | 8 bit  | 机器人等级           |
| remain_HP                  | uint16_t | 16 bit | 当前血量             |
| max_HP                     | uint16_t | 16 bit | 最大血量             |
| shooter_cooling_rate       | uint16_t | 16 bit | 射击冷却速率         |
| shooter_heat_limit         | uint16_t | 16 bit | 射击热量上限         |
| chassis_power_limit        | uint16_t | 16 bit | 底盘功率限制         |
| mains_power_gimbal_output  | uint8_t  | 1 bit  | 云台电源输出状态     |
| mains_power_chassis_output | uint8_t  | 1 bit  | 底盘电源输出状态     |
| mains_power_shooter_output | uint8_t  | 1 bit  | 射击机构电源输出状态 |

### 结构体定义

```c
typedef struct {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;

    uint16_t shooter_cooling_rate;
    uint16_t shooter_heat_limit;

    uint16_t chassis_power_limit;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} __packed game_robot_status_t;
```

------

## 📝 备注

- 所有结构体均使用 `__packed`，**无填充字节**
- 位域（bit-field）用于压缩数据传输体积