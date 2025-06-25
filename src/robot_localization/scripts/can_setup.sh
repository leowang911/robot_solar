#!/bin/bash
#第一步:赋予可执行权限
# 启用 CAN 接口
#sudo ip link set can0 down
#sleep 2
# 设置 CAN 波特率为 1000kbps
#sudo ip link set can0 type can bitrate 1000000
# sudo ip link set can0 up

# 启动 VCAN 接口
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan

sudo ip link set vcan0 up
cansend vcan0 123#1122334455667788


echo "CAN interface configured and started"

# 1)在 在 OPi OSArch 系统中 40pin 中的 CAN 默认都是关闭的    需要手动打开才能
# 使用。
# 在/boot/extlinux/extlinux.conf 中加入下面红色字体部分的配置，然后重启 OPi OS
# Arch 系统就可以打开 CAN0 和 和 CAN1 。
# [orangepi@orangepi ~]$ sudo vim /boot/extlinux/extlinux.conf
# 435
# LINUX /Image
# FDT /dtbs/rockchip/rk3588-orangepi-5-max.dtb

# FDTOVERLAYS /dtbs/rockchip/overlay/rk3588-can0-m0.dtbo /dtbs/rockchip/overlay/rk3588-can1-m1.dtbo
# 上面红色字体配置需要写在一行，不同的配置之间需要用空格隔开。

# 2) 进入 OPi OS Arch 系统后，使用 sudo ifconfig -a 命令如果能看到 CAN 的设备节
# 点，就说明 CAN 已正确打开了
# [orangepi@orangepi ~]$ sudo pacman -Syy net-tools
# [orangepi@orangepi ~]$ sudo ifconfig -a

# bash
# sudo nano /etc/systemd/system/can_setup.service
# 添加以下内容：

# [Unit]
# Description=Set up CAN interface
# After=network.target

# [Service]
# ExecStart=/hoome/orangepi/robot_solar/src/robot_localization/scripts//can_setup.sh
# Restart=on-failure

# [Install]
# WantedBy=multi-user.target