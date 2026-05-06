# 参数持久化存储实现报告

> 开发者视角：修改了什么、怎么改的、用了哪些 Zephyr 知识
> 更新：2026-05-05

---

## 1. 目标

在串口 shell 中实现 `robot param` 系列命令，支持：
- 运行时修改 LQR 平衡控制参数（立即生效）
- 保存到 Flash（断电、重新烧录固件后仍然保留）
- 恢复默认值

---

## 2. 修改文件清单

| 文件 | 改动类型 | 说明 |
|------|---------|------|
| `src/ascento_balance.h` | 扩展结构体 | params 结构体增加 19 个运行时可调字段 |
| `src/ascento_balance.c` | 重写存储逻辑 | 从 NVS 改为直接 Flash 读写 |
| `src/main.c` | 调用方式修改 | 使用 getter 函数获取参数快照 |
| `src/shell_commands.c` | 新增命令 | 添加 `robot param` shell 命令 |
| `boards/.../dji_f407igh6_c.dts` | 新增分区 | 定义 128KB Flash 存储分区 |
| `prj.conf` | 新增配置 | 启用 Flash、Flash Map、CRC |
| `docs/PERSISTENT_PARAMS_GUIDE.md` | 新增文档 | 用户使用指南 |
| `docs/PARAM_TUNING_GUIDE.md` | 更新文档 | 补充 fault_deg/recover_deg 参数 |

---

## 3. 实现步骤

### 3.1 扩展参数结构体（ascento_balance.h）

原来所有 LQR 增益、stiction、电流限制都是 `#define` 编译时常量。改为在 `ascento_balance_params_t` 结构体中增加运行时可调字段：

```c
typedef struct {
    /* ... 原有物理参数（不变） ... */

    /* Runtime-tunable overrides（新增） */
    float theta_eq_rad;
    float gain_c0_a, gain_c0_b, gain_c0_c;  // K_pitch 多项式
    float gain_c1_a, gain_c1_b, gain_c1_c;  // K_pitch_rate 多项式
    float gain_c2;                           // K_position
    float gain_c3_a, gain_c3_b, gain_c3_c;  // K_velocity 多项式
    float stiction_current_ma;
    float stiction_start_deg;
    float stiction_full_deg;
    int16_t current_limit_ma;
    float current_scale;
    float fault_deg;
    float recover_deg;
} ascento_balance_params_t;
```

新增 API：

```c
void ascento_balance_get_params(ascento_balance_params_t *params);  // 线程安全读取
void ascento_balance_set_params(const ascento_balance_params_t *params);  // 线程安全写入
int ascento_balance_save_params(void);   // 保存到 Flash
int ascento_balance_reset_params(void);  // 恢复默认，擦除 Flash
int ascento_balance_settings_init(void); // 启动时从 Flash 加载
```

### 3.2 实现线程安全的参数访问（ascento_balance.c）

将全局参数从 `const` 改为 `static` 可变变量，用 `k_mutex` 保护：

```c
static struct k_mutex params_lock;
static ascento_balance_params_t ascento_balance_runtime_params = {
    .theta_eq_rad = APP_ASCENTO_THETA_EQ_STAND_RAD,
    .gain_c0_a = APP_ASCENTO_GAIN_C0_A,
    // ... 从 #define 初始化
};

void ascento_balance_get_params(ascento_balance_params_t *params)
{
    k_mutex_lock(&params_lock, K_FOREVER);
    *params = ascento_balance_runtime_params;
    k_mutex_unlock(&params_lock);
}
```

控制循环读参数时用 getter 获取快照，避免读到写了一半的数据。

### 3.3 替换 #define 为结构体字段（ascento_balance.c）

在 `ascento_balance_update()` 和 `compute_gains()` 中，把所有 `APP_ASCENTO_*` 宏替换为 `params->*` 字段访问：

```c
// 之前
float err = pitch_rad - APP_ASCENTO_THETA_EQ_STAND_RAD;

// 之后
float err = pitch_rad - params->theta_eq_rad;
```

涉及的替换：theta_eq、gain_c0~c3 多项式系数、stiction 三参数、current_limit、current_scale、fault_deg、recover_deg。

### 3.4 定义 Flash 分区（dji_f407igh6_c.dts）

在设备树中定义存储分区，位于 STM32F407 最后 128KB 扇区：

```dts
&flash0 {
    partitions {
        compatible = "fixed-partitions";
        #address-cells = <1>;
        #size-cells = <1>;
        storage_partition: partition@e0000 {
            label = "storage";
            reg = <0x0e0000 0x20000>;
        };
    };
};
```

地址 0x0E0000 = STM32F407 Sector 11（128KB），固件只用到 ~0x29800，不冲突。

### 3.5 实现 Flash 读写（ascento_balance.c）

**最初尝试 NVS**：使用 Zephyr Settings + NVS 后端。失败，`settings_subsys_init()` 返回 -EDOM (-33)。

**原因**：NVS 的 `sector_size` 字段是 `uint16_t`（最大 65535），STM32F407 的 128KB 扇区 = 131072，超限。

**最终方案**：直接使用 Flash Map API 读写，自定义存储格式：

```c
struct params_flash_header {
    uint32_t magic;      // 0x41534350 ("ASC P")
    uint32_t version;    // 1
    uint32_t data_size;  // sizeof(ascento_balance_params_t)
};
// 后面紧跟 params 数据和 CRC32 校验
```

核心函数：

```c
int ascento_balance_save_params(void)
{
    // 1. flash_area_open() 打开分区
    // 2. 构造 header + params 数据 + CRC32
    // 3. flash_area_erase() 擦除整个扇区
    // 4. flash_area_write() 写入数据
    // 5. flash_area_close() 关闭
}

int ascento_balance_settings_init(void)
{
    // 1. flash_area_open() 打开分区
    // 2. flash_area_read() 读取 header
    // 3. 校验 magic 和 version
    // 4. 校验 CRC32
    // 5. memcpy 到 runtime_params
    // 6. flash_area_close() 关闭
}
```

### 3.6 实现 Shell 命令（shell_commands.c）

添加 `robot param` 命令，支持子命令：

```c
// 参数表：名称 → 结构体偏移量
static const struct {
    const char *name;
    size_t offset;
    const char *desc;
    enum { PF_FLOAT, PF_INT16 } type;
} param_table[] = {
    { "theta_eq",    offsetof(ascento_balance_params_t, theta_eq_rad),    "平衡参考角 (rad)", PF_FLOAT },
    { "k_pitch_a",   offsetof(ascento_balance_params_t, gain_c0_a),      "K_pitch L^2 系数", PF_FLOAT },
    // ... 共 19 个参数
};
```

用 `offsetof()` 实现通用的参数读写，避免为每个参数写单独的 if-else。

### 3.7 修改 main.c 调用方式

```c
// 之前：直接用 const 指针
const ascento_balance_params_t *ap = &ascento_balance_default_params;

// 之后：用 getter 获取快照
ascento_balance_params_t ap_copy;
ascento_balance_get_params(&ap_copy);
const ascento_balance_params_t *ap = &ap_copy;
```

启动时调用 `ascento_balance_settings_init()` 从 Flash 加载参数。

### 3.8 启用 Kconfig（prj.conf）

```ini
CONFIG_FLASH=y              # STM32 Flash 驱动
CONFIG_FLASH_PAGE_LAYOUT=y  # flash_area_get_sectors() 需要
CONFIG_FLASH_MAP=y          # flash_area_open/read/write/erase API
CONFIG_CRC=y                # crc32_ieee() 校验函数
```

移除了 NVS 相关配置：`CONFIG_NVS`、`CONFIG_SETTINGS`、`CONFIG_SETTINGS_NVS`。

---

## 4. 用到的 Zephyr 内核知识

### 4.1 互斥锁（k_mutex）

**用途**：保护共享参数，防止控制循环和 shell 命令同时读写。

```c
static struct k_mutex params_lock;

k_mutex_lock(&params_lock, K_FOREVER);   // 获取锁，永远等待
// ... 读写参数 ...
k_mutex_unlock(&params_lock);            // 释放锁
```

**为什么用 mutex 而不是 irq_lock**：mutex 支持睡眠等待（K_FOREVER），不会阻塞中断；irq_lock 会关闭中断，影响实时性。

### 4.2 Flash Map API

Zephyr 的跨平台 Flash 抽象层，隐藏了不同芯片的 Flash 驱动差异：

```c
#include <zephyr/storage/flash_map.h>

const struct flash_area *fa;
flash_area_open(FIXED_PARTITION_ID(storage_partition), &fa);  // 打开分区
flash_area_read(fa, offset, buf, len);     // 读取
flash_area_write(fa, offset, buf, len);    // 写入（必须先擦除）
flash_area_erase(fa, offset, size);        // 擦除（按扇区对齐）
flash_area_close(fa);                      // 关闭
```

**关键约束**：Flash 写入前必须擦除，擦除粒度是整个扇区（128KB），所以每次 save 都擦除整个分区再重写。

### 4.3 设备树（Devicetree）

用 DTS 描述硬件资源，编译时生成 C 头文件：

```dts
storage_partition: partition@e0000 {
    label = "storage";
    reg = <0x0e0000 0x20000>;
};
```

代码中通过宏引用：

```c
FIXED_PARTITION_ID(storage_partition)  // → 编译时解析为整数 ID
```

**好处**：换硬件（如换芯片或改分区地址）只需改 DTS，不用改 C 代码。

### 4.4 CRC32 校验

```c
#include <zephyr/sys/crc.h>

uint32_t crc = crc32_ieee(data, len);  // IEEE 802.3 CRC32
```

**用途**：检测 Flash 数据是否损坏（掉电写入中断、Flash 老化等）。启动时校验 CRC，不匹配则忽略 Flash 数据，使用默认参数。

### 4.5 Shell 子命令系统

Zephyr 的 shell 模块支持多级命令：

```c
SHELL_CMD_ARG(param, NULL,
    "robot param [list | save | reset | <name> <value>]",
    cmd_robot_param, 1, 2),
```

- `SHELL_CMD_ARG`：注册带参数的命令
- 第 5 个参数 `1`：最少参数数（不含命令名）
- 第 6 个参数 `2`：可选参数数

### 4.6 offsetof 宏

```c
#include <stddef.h>

offsetof(ascento_balance_params_t, theta_eq_rad)  // → 字段在结构体中的字节偏移
```

**用途**：实现通用参数表，用偏量量直接读写结构体字段，避免为 19 个参数各写一套代码。

```c
uint8_t *base = (uint8_t *)&params;
*(float *)(base + param_table[i].offset) = fval;  // 通用写入
```

---

## 5. 调试过程中遇到的问题

### 5.1 NVS 返回 -EDOM（根因）

**现象**：`robot param save` 返回 -2 (ENOENT)。

**排查过程**：
1. 添加 printk 调试 → 发现 `settings_subsys_init` 返回 -33
2. 查 Zephyr 源码 → -33 = -EDOM
3. 定位到 `settings_nvs.c`：`if (nvs_sector_size > UINT16_MAX) return -EDOM;`
4. 计算：128KB = 131072 > 65535 (UINT16_MAX)

**解决**：绕过 NVS，直接使用 Flash Map API。

### 5.2 Boot 日志丢失

**现象**：`ascento_balance_settings_init()` 中的 printk 在启动时不可见。

**原因**：函数在 UART 驱动初始化之前调用，printk 输出到尚未就绪的串口。

**验证**：通过 `robot param` 命令确认参数已从 Flash 加载（值与默认值不同）。

### 5.3 烧录后参数保留

**验证**：`flash write_image erase` 只擦除固件占用的扇区（Sector 0-5），不影响 Sector 11 的参数分区。

---

## 6. 最终效果

```text
# 串口修改参数
uart:~$ robot param theta_eq 0.35
theta_eq = 0.35

# 保存到 Flash
uart:~$ robot param save
params saved to flash (184 bytes)

# 断电重启后，参数自动加载
uart:~$ robot param
theta_eq       = 0.3500 rad (20.1 deg)   ← 保留了！
...

# 重新烧录固件后，参数仍然保留
uart:~$ robot param
theta_eq       = 0.3500 rad (20.1 deg)   ← 依然保留！
```
