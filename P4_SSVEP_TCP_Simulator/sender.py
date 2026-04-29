import argparse
import math
import random
import socket
import struct
import time

HEADER0 = 0xAA
HEADER1 = 0x55
CHANNELS = 8
FRAME_STRUCT = struct.Struct("<BBH8fB")
DEFAULT_TARGET_FREQS = [8.0, 10.0, 12.0, 14.0]


def calc_crc(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
    return crc


def build_binary_frame(seq: int, values: list[float]) -> bytes:
    payload = struct.pack("<BBH8f", HEADER0, HEADER1, seq & 0xFFFF, *values)
    crc = calc_crc(payload)
    return payload + bytes([crc])


def build_csv_line(values: list[float]) -> bytes:
    return (",".join(f"{v:.6f}" for v in values) + "\n").encode("utf-8")


# 【修改点 1】：重新设计的生成函数，支持混入所有目标频率
def generate_channel_values(
    sample_index: int,
    sample_rate: float,
    main_freq: float,
    all_freqs: list[float],  # 新增：把需要混入的干扰频率列表传进来
    amplitude: float,
    noise_amplitude: float,
    dc_offset: float,
    harmonics: int,
) -> list[float]:
    t = sample_index / sample_rate
    values = []
    
    for ch in range(CHANNELS):
        phase = ch * 0.08
        scale = 1.0 - ch * 0.04
        signal = 0.0
        
        # 将传入的所有频率成分混合在一起
        for f in all_freqs:
            # 区分主频和干扰频：主频幅值权重1.0，其他干扰频率权重0.3
            weight = 1.0 if f == main_freq else 0.3
            
            for harmonic in range(1, harmonics + 1):
                harmonic_scale = 1.0 / harmonic
                # 把不同频率和高次谐波的信号叠加到 signal 变量中
                signal += weight * harmonic_scale * math.sin(2.0 * math.pi * f * harmonic * t + phase)

        noise = random.uniform(-noise_amplitude, noise_amplitude)
        values.append(dc_offset + amplitude * scale * signal + noise)
        
    return values


def parse_frequency_list(text: str) -> list[float]:
    return [float(item.strip()) for item in text.split(",") if item.strip()]


def main() -> None:
    parser = argparse.ArgumentParser(description="ESP32-P4 SSVEP TCP simulator")
    parser.add_argument("--host", required=True, help="P4 IP address")
    parser.add_argument("--port", type=int, default=1234, help="P4 TCP port")
    parser.add_argument("--format", choices=["binary", "csv"], default="binary", help="Output stream format")
    parser.add_argument("--mode", choices=["fixed", "sweep"], default="fixed", help="Fixed target or cyclic sweep")
    parser.add_argument("--freq", type=float, default=10.0, help="Fixed target frequency in Hz")
    parser.add_argument("--target-freqs", default="8,10,12,14", help="Sweep frequency list")
    parser.add_argument("--switch-seconds", type=float, default=4.0, help="Sweep dwell time per target")
    parser.add_argument("--sample-rate", type=float, default=250.0, help="Samples per second to send")
    parser.add_argument("--duration", type=float, default=0.0, help="Run duration in seconds; 0 means forever")
    parser.add_argument("--amplitude", type=float, default=1200.0, help="Main sine amplitude")
    parser.add_argument("--noise", type=float, default=80.0, help="Noise amplitude")
    parser.add_argument("--dc-offset", type=float, default=0.0, help="Signal DC offset")
    parser.add_argument("--harmonics", type=int, default=2, help="Number of harmonics to mix into fake EEG")
    args = parser.parse_args()

    sample_interval = 1.0 / args.sample_rate
    sweep_freqs = parse_frequency_list(args.target_freqs)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((args.host, args.port))
    print(f"Connected to {args.host}:{args.port}")
    print(
        f"format={args.format} mode={args.mode} sample_rate={args.sample_rate}Hz "
        f"amplitude={args.amplitude} noise={args.noise}"
    )

    seq = 0
    sample_index = 0
    start_time = time.perf_counter()
    next_send_time = start_time
    last_status_time = start_time
    sent_samples = 0

    try:
        while True:
            now = time.perf_counter()
            elapsed = now - start_time
            if args.duration > 0.0 and elapsed >= args.duration:
                break

            if args.mode == "fixed":
                active_freq = args.freq
            else:
                slot = int(elapsed / args.switch_seconds) % len(sweep_freqs)
                active_freq = sweep_freqs[slot]

            # 【修改点 2】：调用函数时，增加了 all_freqs 参数
            values = generate_channel_values(
                sample_index=sample_index,
                sample_rate=args.sample_rate,
                main_freq=active_freq,
                all_freqs=sweep_freqs,     # 把命令行解析出来的 [8.0, 10.0, 12.0, 14.0] 传进去作为混频源
                amplitude=args.amplitude,
                noise_amplitude=args.noise,
                dc_offset=args.dc_offset,
                harmonics=max(1, args.harmonics),
            )

            if args.format == "binary":
                packet = build_binary_frame(seq, values)
                seq = (seq + 1) & 0xFFFF
            else:
                packet = build_csv_line(values)

            sock.sendall(packet)
            sample_index += 1
            sent_samples += 1

            if now - last_status_time >= 1.0:
                print(
                    f"elapsed={elapsed:6.2f}s active_freq={active_freq:4.1f}Hz "
                    f"sent_samples={sent_samples}"
                )
                last_status_time = now

            next_send_time += sample_interval
            sleep_time = next_send_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_send_time = time.perf_counter()
    except KeyboardInterrupt:
        print("Stopped by user")
    finally:
        sock.close()
        print("Socket closed")


if __name__ == "__main__":
    main()