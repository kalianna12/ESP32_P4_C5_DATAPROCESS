# P4 SSVEP TCP Simulator

这个小工程用电脑模拟 EEG/SSVEP 数据，通过 TCP 发送给数据处理 P4。

## 它能做什么

- 按你现在 P4 固件兼容的 **二进制 37 字节帧** 发送假数据
- 也能发送 `csv` 文本行，验证 P4 的文本解析分支
- 可固定发 `8/10/12/14Hz` 之一
- 可做 `sweep` 轮询测试，自动每几秒切一次频率

## 目录

- `sender.py`：主发送脚本

## 测试前准备

1. 让 `电脑` 和 `数据处理 P4` 都连到同一个手机热点
2. 给 P4 上电，打开串口监视器
3. 在 P4 控制台先连网

推荐顺序：

```text
wifiinit
scan
<输入热点序号>
<输入热点密码>
status
```

看到 `Got IP: yes` 就表示 P4 已经联网。

## 如何知道 P4 的 IP

在串口里执行：

```text
status
```

串口里会打印当前 SSID、RSSI，以及拿到的 IP。

## 最简单的测试命令

先发 10Hz 二进制数据：

```powershell
python .\sender.py --host 你的P4IP --format binary --mode fixed --freq 10
```

例如：

```powershell
python .\sender.py --host 172.20.10.8 --format binary --mode fixed --freq 10
```

## 轮询测试

每 4 秒切一次 `8,10,12,14Hz`：

```powershell
python .\sender.py --host 你的P4IP --format binary --mode sweep --target-freqs 8,10,12,14 --switch-seconds 4
```

## 发 CSV 文本

```powershell
python .\sender.py --host 你的P4IP --format csv --mode fixed --freq 12
```

## 常用参数

- `--host`：P4 的 IP，必填
- `--port`：默认 `1234`
- `--format`：`binary` 或 `csv`
- `--mode`：`fixed` 或 `sweep`
- `--freq`：固定频率模式下的目标频率
- `--sample-rate`：默认 `250Hz`
- `--amplitude`：默认 `1200`
- `--noise`：默认 `80`
- `--duration`：测试时长，`0` 表示一直发

## 当前 P4 串口上会看到什么

### 连接时

P4 会打印类似：

```text
I (...) dataprocess: TCP server listening on port 1234
I (...) dataprocess: Client connected: 172.20.10.xx:xxxxx
I (...) dataprocess: Detected binary ADS1299 frame stream
```

### 检测成功时

P4 会打印类似：

```text
I (...) dataprocess: CCA detected 10 Hz idx=1 conf=0.82 [0.21 0.82 0.15 0.09]
```

这里含义是：

- `10 Hz`：当前判到了 10Hz
- `idx=1`：映射到显示侧枚举 1
- `conf=0.82`：当前置信度
- 方括号里 4 个值：分别对应 `8/10/12/14Hz` 的相关系数

## 你现在怎么调

### 在电脑侧调

直接改发送参数：

- 想测 8Hz：`--freq 8`
- 想测 10Hz：`--freq 10`
- 想测 12Hz：`--freq 12`
- 想测 14Hz：`--freq 14`
- 想更稳定：减小 `--noise`
- 想更容易识别：增大 `--amplitude`
- 想连续切频率：用 `--mode sweep`

### 在 P4 侧看效果

目前这版固件里，最直接的反馈就是串口日志：

- 是否连上 TCP
- 是否识别成 binary/csv
- CCA 检测到了哪个频率

## 推荐第一轮联调方式

先这样跑：

```powershell
python .\sender.py --host 你的P4IP --format binary --mode fixed --freq 10 --noise 40 --amplitude 1500
```

这组参数更容易先看到稳定识别。

## 如果没看到检测日志

优先检查这几项：

1. P4 串口 `status` 里是否已经 `Got IP: yes`
2. 电脑和 P4 是否真的在同一个手机热点网段
3. `sender.py` 里的 `--host` 是否填成了 P4 的 IP
4. 是否看到了 `Client connected`
5. 是否看到了 `Detected binary ADS1299 frame stream`
6. 当前发的数据频率是否是 `8/10/12/14Hz` 之一
7. 采样率是否仍然按 `250Hz` 发送

## 说明

这个模拟器更适合做“链路联调”和“算法通路验证”。它不是医学级 EEG 仿真器，但足够用来验证：

- hosted Wi-Fi 是否工作
- TCP 接收是否正常
- P4 数据格式识别是否正常
- CCA 检测是否触发
- SPI 结果发送是否进入执行路径
