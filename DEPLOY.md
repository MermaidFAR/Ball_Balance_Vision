# RDK X5 新板部署说明

## 包含内容

- 完整 C++ 源码和 `CMakeLists.txt`
- V4L2 相机采集、X5 MJPEG 硬件解码和 BPU 检测
- 固定线性 α-β 位置/速度观测器
- `500 Hz` UART 位置、速度和控制字节输出
- 低负载网页预览、目标控制、录像与回放
- systemd 服务与相机热插拔模板 `deploy/ti-vision.service`、`deploy/ti-vision.path`
- 已验证的 `640x96 NV12` BPU 模型
- 旧板当前运行的 RDK X5 预编译二进制 `prebuilt/ti_vision`

## 模型文件

交付包已经包含：

```text
models/compiled/ball_yolo26n_640x96_nv12.bin
```

模型 SHA-256：

```text
D72BEB8098BFBF1FEA5078BA71027E7083395C1B64B0907BAA217B1D51CC93A2
```

启动前必须将它安装到：

```text
/root/TI/models/compiled/ball_yolo26n_640x96_nv12.bin
```

该模型输入必须是 `640x96 NV12`。模型不匹配时程序会拒绝启动。

## 上传目录

将本目录完整上传为新板的：

```text
/root/TI/vision
```

不要只复制 `src/main.cpp` 等几个覆盖文件，否则会缺少相机、硬解、BPU、网页和回放模块。

上传后安装模型：

```bash
mkdir -p /root/TI/models/compiled
cp /root/TI/vision/models/compiled/ball_yolo26n_640x96_nv12.bin \
  /root/TI/models/compiled/ball_yolo26n_640x96_nv12.bin
```

## 新板构建

在 RDK X5 上执行：

```bash
cd /root/TI/vision
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
```

系统镜像需要提供 Horizon RDK 的 `dnn`、`multimedia` 和 `hbmem` 开发库。构建成功后应生成：

```text
/root/TI/vision/build/ti_vision
```

也可直接使用旧板已验证的预编译二进制：

```bash
mkdir -p /root/TI/vision/build
install -m 755 /root/TI/vision/prebuilt/ti_vision \
  /root/TI/vision/build/ti_vision
```

预编译二进制 SHA-256：

```text
20267B163858BCD4465C12C5306C37DB3032AF68A6A23BB0D7D2DC755084BEA6
```

## 手动启动验证

```bash
/root/TI/vision/build/ti_vision \
  --uart /dev/ttyS1 \
  --target-control 0
```

默认参数：

- 相机：`/dev/video0`
- UART：`/dev/ttyS1`，`115200 8N1`
- UART 发布频率：`500 Hz`
- 网页端口：`8080`
- 预览与录像：`8 FPS`
- 录像目录：`/root/TI/recordings`
- 三点实机标定：`zero_px=333.0831`、`cm_per_px=0.0220166164`

浏览器访问 `http://<RDK-IP>:8080/`。页面可查看识别画面、目标控制、位置和速度数据，并可录制与回放。

## UART 接收端兼容性

本包发送 11 字节小端帧：

```text
A5 09 + float32 position_cm + float32 speed_cm_s + int8 target_control
```

部署前确认 H7 接收端也使用这一版协议。若 H7 仍是旧的 `A5 05` 位置-only 协议，不能直接混用。

## 安装开机服务

手动启动验证通过后执行：

```bash
cp /root/TI/vision/deploy/ti-vision.service /etc/systemd/system/ti-vision.service
cp /root/TI/vision/deploy/ti-vision.path /etc/systemd/system/ti-vision.path
systemctl daemon-reload
systemctl disable --now ti-vision.service
systemctl enable --now ti-vision.path
systemctl status ti-vision.service --no-pager
```

`ti-vision.path` 会等待 `/dev/video0` 出现后自动启动视觉。这样开机未插相机时不会反复重启刷日志，相机随后接入也会自动拉起服务。

查看日志：

```bash
journalctl -u ti-vision.service -f
```

停止并禁止开机启动：

```bash
systemctl disable --now ti-vision.path
systemctl stop ti-vision.service
```

## 低性能影响设计

- 网页只复用推理链路已有的 `640x96` Y 平面，不额外解码或推理
- 默认每秒只复制 `8` 帧用于预览和录像
- HTTP 线程使用低调度优先级
- 录像异步写盘，队列拥塞时丢录像帧，不阻塞识别线程
- 可用 `--web-port 0` 完全关闭网页和回放，做纯视觉性能对照
