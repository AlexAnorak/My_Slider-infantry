## UP=>DOWN

### high freq send

ID:150

| offset | length | value          | description                                                           |
| ------ | ------ | -------------- | --------------------------------------------------------------------- |
| 0      | 1      | deforceFlag    | -                                                                     |
| 1      | 2      | rcSwitch1      | 拨杆 1                                                                |
| 3      | 2      | rcSwitch2      | 拨杆 2                                                                |
| 5      | 1      | keyCtrl        | -                                                                     |
| 6      | 1      | resetFlag      | 重启标志位                                                            |
| 7      | 1      | selfCheck      | 自检标志位                                                            |
| 8      | 2      | chassisMode    | 0 正常，1 陀螺，2 摇摆，3 不跟随云台，4 变速陀螺，5 飞坡，6 自定义，7 |
| 10     | 13     | speedWS        | 前后速度                                                              |
| 23     | 13     | speedAD        | 左右速度                                                              |
| 36     | 11     | speedYaw       | 旋转速度                                                              |
| 47     | 13     | zeroYaw        | 零点（度）                                                            |
| 50     | 1      | tryUsingBackup | 使用备用功率                                                          |

### low freq send

ID:170

| offset | length | value                | description            |
| ------ | ------ | -------------------- | ---------------------- |
| 0      | 6      | frictionSpdA         | -                      |
| 6      | 6      | frictionSpdB         | -                      |
| 12     | 1      | visionOffline        | -                      |
| 13     | 1      | frictionOffline      | -                      |
| 14     | 1      | forceOpenLoop        | -                      |
| 15     | 1      | unLimitedFired       | -                      |
| 16     | 1      | ammoCoverOpen        | -                      |
| 17     | 1      | blockError           | -                      |
| 18     | 1      | visionLock           | -                      |
| 19     | 1      | visionBeat           | -                      |
| 20     | 2      | gimbalMode           | -                      |
| 22     | 9      | localHeat            | -                      |
| 31     | 1      | heatChoose           | 0 为摩擦轮，1 为拨弹轮 |
| 32     | 1      | sprocketMotorOffline | 拨弹轮离线             |
| 33     | 2      | VisionMode           | -                      |

## DOWN=>UP

### high freq send

ID:160

| offset | length | value       | description        |
| ------ | ------ | ----------- | ------------------ |
| 0      | 13     | speedWS     | 前后速度           |
| 13     | 1      | jgmtOffline | 裁判系统是否离线   |
| 14     | 9      | bulletSpeed | 实际射速 int(\*10) |
| 23     | 16     | CoolingHeat | 当前热量           |
| 39     | 16     | shootNum    | 当前发射子弹数     |
| 55     |

### low freq send

ID:140

| offset | length | value        | description                                                                   |
| ------ | ------ | ------------ | ----------------------------------------------------------------------------- |
| 0      | 2      | robotLevel   | 等级                                                                          |
| 2      | 4      | gameProgress | 比赛进程                                                                      |
| 6      | 3      | hurtType     | 0 装甲伤害<br>1 模块掉血<br>2 超射速<br>3 超热量<br>4 超功率<br>5 撞击 |
| 9      | 2      | enemyColor   | 0 无<br>1 红<br>2 蓝？                                                        |
| 11     | 5      | SpeedLimit   | 射速                                                                          |
| 16     | 16     | CoolingRate  | 冷却速率                                                                      |
| 32     | 16     | CoolingLimit | 热量上限                                                                      |
| 48     | 16     | maxHp        | 血量上限                                                                      |
| 64     |

## POWER

<table>
    <tr>
        <th>var name</th>
        <th>Description</th>
    <tr>
    <tr>
        <th colspan="2" :--:>id: 0x110</th>
    <tr>
    <tr>
        <td>chassis_power</td>
        <td>底盘功率</td>
    <tr>
    <tr>
        <td>chassis_power_buffer</td>
        <td>底盘缓冲能量</td>
    <tr>    
    <tr>
        <td>max_chassis_power</td>
        <td>最大功率</td>
    <tr>
    <tr>
        <th colspan="2" :--:>id: 0x120</th>
    <tr>
    <tr>
        <td>PowerPath_Switch</td>
        <td>是否直通</td>
    <tr>
    <tr>
        <td>Check_Mode</td>
        <td>检录模式</td>
    <tr>    
    <tr>
        <td>ULTS_Mode</td>
        <td>特殊模式</td>
    <tr>
        <th colspan="2" :--:>id: 0x130</th>
    <tr>
    <tr>
        <td>capacitance_percentage</td>
        <td>电容能量百分比</td>
    <tr>
    <tr>
        <td>maxCurrent</td>
        <td>最大电流</td>
    <tr>    
</table>
