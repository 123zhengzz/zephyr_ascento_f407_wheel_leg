# 参数持久化存储 — 初学者指南

> 适用：Ascento 轮腿机器人，STM32F407IGH6 主控
> 更新：2026-05-05

---

## 1. 这是什么？

机器人有很多可调参数（平衡角度、PID 增益、电流限制等）。以前改参数需要修改代码、重新编译、重新烧录，很麻烦。

现在可以通过串口**实时修改参数**，并且**保存到芯片内部 Flash**，下次开机自动加载，不需要重新编译。

---

## 2. 快速上手

### 2.1 连接串口

```bash
# 方法一：使用项目脚本
./scripts/serial.sh

# 方法二：手动连接
picocom -b 115200 /dev/ttyACM0
```

### 2.2 查看当前参数

```text
uart:~$ robot param

theta_eq       = 0.3670 rad (21.0 deg)
k_pitch        = -8.0019*L^2 + 2.9106*L + -5.2007
k_pitch_rate   = -19.1727*L^2 + 7.0983*L + -1.4842
k_position     = -0.6325
k_velocity     = -6.1109*L^2 + 2.3991*L + -1.1471
k_yaw_rate     = 0.0000
stiction_ma    = 2800 mA
stiction_start = 0.10 deg
stiction_full  = 1.50 deg
current_limit  = 1500 mA
current_scale  = 1.00
fault_deg      = 45.0 deg
recover_deg    = 15.0 deg
```

### 2.3 修改参数

```text
# 修改平衡角度为 0.35 rad（约 20 度）
uart:~$ robot param theta_eq 0.35
theta_eq = 0.35

# 修改 stiction 电流为 0（禁用）
uart:~$ robot param stiction_ma 0
stiction_ma = 0
```

参数修改后**立即生效**，但断电会丢失。

### 2.4 保存参数（永久）

```text
uart:~$ robot param save
ascento: params saved to flash (184 bytes)
params saved to flash
```

保存后，参数写入芯片内部 Flash，**断电不丢失**，下次开机自动加载。

### 2.5 恢复默认值

```text
uart:~$ robot param reset
params reset to defaults
theta_eq       = 0.3670 rad (21.0 deg)
...
```

恢复为代码中 `#define` 定义的默认值，并清除 Flash 中的保存数据。

---

## 3. 完整命令列表

| 命令 | 作用 |
|------|------|
| `robot param` | 查看所有当前参数 |
| `robot param list` | 列出可调参数名及说明 |
| `robot param <name> <value>` | 修改单个参数（立即生效，断电丢失） |
| `robot param save` | 保存到 Flash（永久，断电保留） |
| `robot param reset` | 恢复代码默认值，清除 Flash 存储 |

---

## 4. 所有可调参数

### 4.1 平衡参考角

| 参数名 | 说明 | 默认值 |
|--------|------|--------|
| `theta_eq` | 平衡参考角 (rad)，前倾为正 | 0.367 (21°) |

**含义**：机器人静止时的自然前倾角度。如果机器人往前倒，增大此值；往后倒，减小此值。

### 4.2 LQR 增益（按腿长 L 调度）

增益公式：`K(L) = A*L² + B*L + C`

| 参数名 | 对应增益 | 默认值 |
|--------|---------|--------|
| `k_pitch_a/b/c` | 倾角回复力矩 | -8.0 / 2.9 / -5.2 |
| `k_pitch_rate_a/b/c` | 角速度阻尼 | -19.2 / 7.1 / -1.5 |
| `k_position` | 位置漂移回复 | -0.63 |
| `k_velocity_a/b/c` | 速度阻尼 | -6.1 / 2.4 / -1.1 |
| `k_yaw_rate` | 偏航角速率增益 | 0.0 |

**调法**：
- 机器人振荡（左右晃）→ 减小 `k_pitch_c` 的绝对值
- 机器人无力倒下 → 增大 `k_pitch_c` 的绝对值
- 振荡剧烈 → 增大 `k_pitch_rate_c` 的绝对值（更多阻尼）

### 4.3 Stiction 补偿

| 参数名 | 说明 | 默认值 |
|--------|------|--------|
| `stiction_ma` | 补偿电流幅值 (mA) | 2800 |
| `stiction_start` | 起始角度 (deg) | 0.10 |
| `stiction_full` | 满幅角度 (deg) | 1.50 |

**含义**：电机控制器有最小电流死区，stiction 补偿在小角度时叠加额外电流来克服死区。

**注意**：如果机器人响应过激，先尝试 `robot param stiction_ma 0` 禁用 stiction。

### 4.4 电流限制

| 参数名 | 说明 | 默认值 |
|--------|------|--------|
| `current_limit` | 轮子电流上限 (mA) | 1500 |
| `current_scale` | 电流缩放系数 | 1.0 |

### 4.5 故障保护

| 参数名 | 说明 | 默认值 |
|--------|------|--------|
| `fault_deg` | 触发保护的角度阈值 (deg) | 45.0 |
| `recover_deg` | 恢复允许的角度范围 (deg) | 15.0 |

**含义**：倾角超过 `fault_deg` 时切断电机，防止翻转损坏。调试时可临时增大：

```text
uart:~$ robot param fault_deg 90
```

---

## 5. 调参实战流程

```text
# 第一步：查看当前参数
robot param

# 第二步：禁用 stiction（排除干扰）
robot param stiction_ma 0

# 第三步：放宽故障保护
robot param fault_deg 90

# 第四步：调整平衡角度（扶住机器人看 pitch 值）
robot param theta_eq 0.35

# 第五步：松手测试，观察行为
# - 往前倒 → 增大 theta_eq
# - 往后倒 → 减小 theta_eq
# - 振荡   → 减小 k_pitch_c 绝对值，增大 k_pitch_rate_c 绝对值

# 第六步：调好后保存
robot param save

# 第七步：断电重启，验证参数是否保留
# 开机后执行 robot param 查看
```

---

## 6. 常见问题

### Q: 保存后断电，参数还在吗？

**A**: 在。参数保存在芯片内部 Flash，断电永久保留。

### Q: 重新烧录固件后，参数还在吗？

**A**: 在。参数分区（0x0E0000）和固件分区（0x000000-0x029800）是独立的，烧录固件不会擦除参数。

### Q: 什么时候需要 `robot param reset`？

**A**: 当你调乱了参数想恢复出厂默认值时。执行后 Flash 中的保存数据被清除，恢复为代码中的 `#define` 值。

### Q: `robot param save` 和直接改代码有什么区别？

| 方式 | 优点 | 缺点 |
|------|------|------|
| `robot param save` | 不需要重新编译，实时调试 | 只在这一台机器上生效 |
| 改 `app_config.h` | 所有新烧录的机器都用新默认值 | 需要重新编译烧录 |

**推荐流程**：先用 `robot param` 串口调好参数 → `robot param save` 保存 → 确认效果好后 → 改 `app_config.h` 写入代码。

### Q: 参数保存失败怎么办？

**A**: 正常情况下不会失败。如果看到 `save failed` 错误码，请检查：
- 错误码 -2 (ENOENT)：Flash 分区未定义（检查 DTS）
- 错误码 -5 (EIO)：Flash 写入错误（硬件问题）

---

## 7. 技术细节（进阶）

### 存储格式

```
偏移 0x00: magic     (4 字节) = 0x41534350 ("ASC P")
偏移 0x04: version   (4 字节) = 1
偏移 0x08: data_size (4 字节) = sizeof(ascento_balance_params_t)
偏移 0x0C: data      (N 字节) = 参数结构体原始数据
偏移 0x0C+N: CRC32   (4 字节) = 前面所有数据的 CRC32 校验
```

### Flash 分区

```
STM32F407IGH6 Flash (1MB = 0x100000)

0x00000 ┌─────────────────────┐
        │  固件 (约 164KB)     │  Sector 0-5 (16KB×4 + 64KB + 128KB)
0x29800 ├─────────────────────┤
        │  未使用              │
0x40000 ├─────────────────────┤
        │  ...                │  Sector 6-10 (128KB × 5)
0xE0000 ├─────────────────────┤
        │  参数存储 (128KB)    │  Sector 11 ← robot param save 写这里
0xFFFFF └─────────────────────┘
```

### Kconfig 依赖

```ini
# prj.conf 中的相关配置
CONFIG_FLASH=y              # Flash 驱动
CONFIG_FLASH_PAGE_LAYOUT=y  # Flash 页布局（flash_area_get_sectors 需要）
CONFIG_FLASH_MAP=y          # Flash Map API（flash_area_open/read/write/erase）
CONFIG_CRC=y                # CRC32 校验库
```

### 为什么不用 NVS？

Zephyr 的 NVS（Non-Volatile Storage）库的 `sector_size` 字段是 `uint16_t`（最大 65535）。STM32F407 的 128KB 扇区 = 131072 字节，超过限制，NVS 初始化会返回 -EDOM 错误。因此改用直接 Flash 读写方式。
