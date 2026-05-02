# 串口终端使用说明

## 权限问题

串口设备（如 `/dev/ttyUSB0`）属于 `dialout` 组，当前用户需要加入该组：

```bash
sudo usermod -a -G dialout $USER
```

然后**注销重新登录**使其生效。如果不想重登录，可以临时用：

```bash
newgrp dialout
```

## 启动串口

```bash
./scripts/serial.sh
```

默认自动查找 `/dev/serial/by-id/*` 下的设备，波特率 115200。也可手动指定：

```bash
./scripts/serial.sh /dev/ttyUSB0 115200
```

脚本已配置 `--echo` 本地回显，输入时会显示在终端中。

## picocom 基本使用

- 串口已连接后，**直接打字即可发送**，每个字符实时发送
- 按下**回车**发送换行符
- `Ctrl+A` `Ctrl+H` — 查看所有快捷键
- `Ctrl+A` `Ctrl+E` — 切换本地回显开关
- `Ctrl+A` `Ctrl+C` — 切换换行符格式
- `Ctrl+A` `Ctrl+X` — 退出 picocom
