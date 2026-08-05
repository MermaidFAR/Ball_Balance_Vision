# RDK X5 滚球视觉与网页控制端

原生 C++ 热路径：V4L2 MMAP 固定采集 `640x480 MJPEG@120 FPS`，使用 `5 ms` 手动曝光防止摄像头动态降帧，X5 硬件 MJPEG 解码为 NV12，截取中央全宽 `640x96` ROI，BPU 执行 YOLO26n，最高置信度单球结果按 latest-value 方式发布。程序不切换分辨率；丢球时保留最后一次左右方向，以 `2 cm/s` 从最后位置连续变化到对应的 `+10 cm` 或 `-10 cm`。

位置和速度由固定线性 α-β 观测器同时给出，时间常数为 `40 ms`，没有死区、增益切换、尖峰裁剪或自适应非线性。UART 默认按 `500 Hz` 发布，并在两次视觉更新之间执行最长 `12 ms` 的 `position += speed × age` 线性外推，以兼顾低延迟和输出连续性。

程序内置轻量网页控制台，默认监听 `8080` 端口。网页可以切换目标位置、查看带检测框的实时画面、手动发送测试位置、立即停发，并完整记录与回放每次测试。

## 目录

- `include/ti_vision/`：模块接口与数据类型
- `src/`：V4L2、硬解、BPU、UART、网页与记录模块
- `config/`：不含凭据的运行参数模板
- `models/compiled/`：已验证的 `640x96 NV12` BPU 模型
- `prebuilt/`：旧板当前运行的已验证 RDK X5 二进制
- `deploy/`：systemd 服务模板
- `BENCHMARK.md`：模型与完整链路实测记录
- `DEPLOY.md`：新板完整部署与启动说明

## RDK 构建

```bash
cd /root/TI/vision
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
```

## 启动视觉、UART 与网页

```bash
/root/TI/vision/build/ti_vision \
  --uart /dev/ttyS1 \
  --target-control 0
```

浏览器访问：

```text
http://<RDK-IP>:8080/
```

UART 使用 11 字节小端协议：

```text
A5 09 + float32 position_cm + float32 speed_cm_s + int8 target_control
```

- `A5`：帧头
- `09`：负载长度
- `position_cm`：观测位置，单位 `cm`
- `speed_cm_s`：观测速度，单位 `cm/s`
- `target_control`：网页或命令行设置的控制字节

目标控制值定义：

- `0`：固定中心 `0 cm`
- `1`：执行第三题 `0 → +5 → -5 cm` 斜坡
- 其他 `-12..+12` 整数：固定目标位置，单位 `cm`

视觉主循环本身停止更新超过 `80 ms` 时 UART 停止发送。发送线程使用绝对周期，不做跨周期追帧突发。可用 `--lost-ramp-cm-s X` 修改丢球后的变化速度，可用 `--send-rate X` 修改 UART 发布频率。

## 低负载预览与完整回放

网页实时预览和测试记录默认使用推理输入中已经存在的 `640x96` Y 平面，每秒只复制 8 帧；不增加 JPEG 编码、第二次解码或第二次推理。HTTP 线程使用 `SCHED_IDLE`，记录写盘由独立线程完成，队列拥塞时优先丢记录帧而不阻塞识别。

点击“开始完整记录”后，记录保存在 `/root/TI/recordings/*.tivr`。每帧包含灰度画面、检测框、识别位置、实际发送位置、目标控制值和时间戳；网页可选择任意历史测试完整播放。单次记录最长 5 分钟，默认数据率约 `0.49 MB/s`，30 秒测试约 `15 MB`。

可调整参数：

```text
--web-port 8080
--preview-fps 8
--recordings-dir /root/TI/recordings
```

`--web-port 0` 可完全关闭网页与记录模块。只测原始视觉性能时可运行：

```bash
/root/TI/vision/build/ti_vision --duration 10 --web-port 0
```

完整的新板部署步骤、模型文件说明与 systemd 安装方法见 `DEPLOY.md`。
