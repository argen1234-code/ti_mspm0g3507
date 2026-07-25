"""
串口自动识别器 -- Serial Finder
================================
自动扫描所有 COM 口, 通过 MSPM0_ALIVE 握手信号找到对应的串口,
然后自动启动监视器。

用法:
  python serial_find.py            # 扫描并连接
  python serial_find.py --scan     # 仅扫描, 不连接
  python serial_find.py COM3       # 跳过扫描, 直接连接

依赖: pyserial
"""

import sys
import os
import time
import threading
import serial
import serial.tools.list_ports

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_FILE = os.path.join(SCRIPT_DIR, "serial_log.txt")
SCAN_TIMEOUT = 1.5   # 每个端口监听秒数
BAUD_RATE = 115200
HANDSHAKE_MARKER = "MSPM0"


def list_all_ports():
    """列出所有可用串口"""
    ports = serial.tools.list_ports.comports()
    return [p.device for p in ports]


def scan_port(port: str, timeout: float = SCAN_TIMEOUT) -> tuple[bool, str]:
    """
    打开端口, 监听一段时间, 检查是否收到握手信号。
    返回 (found, data_preview)
    """
    data = ""
    found = False
    try:
        ser = serial.Serial(port, baudrate=BAUD_RATE, timeout=0.1)
        ser.reset_input_buffer()
        deadline = time.time() + timeout
        while time.time() < deadline:
            if ser.in_waiting:
                chunk = ser.read(ser.in_waiting)
                try:
                    text = chunk.decode("utf-8", errors="replace")
                except Exception:
                    text = ""
                data += text
                if HANDSHAKE_MARKER in data:
                    found = True
                    break
            time.sleep(0.05)
        ser.close()
    except serial.SerialException as e:
        return False, f"(无法打开: {e})"
    except Exception as e:
        return False, f"(错误: {e})"

    preview = data[:120].replace("\r\n", " ").replace("\n", " ").replace("\r", " ")
    return found, preview


def find_mspm0_port() -> str | None:
    """
    扫描所有 COM 口, 返回匹配 MSPM0 的端口名。
    如果找到, 打印匹配结果。
    """
    ports = list_all_ports()

    if not ports:
        print("[!] 没有发现任何 COM 口")
        print("    请检查 USB 转串口模块是否已插入")
        return None

    print(f"扫描 {len(ports)} 个串口 (每个监听 {SCAN_TIMEOUT} 秒)...\n")
    print(f"{'端口':<10} {'状态':<12} 接收数据预览")
    print("-" * 70)

    found_port = None

    for port in ports:
        print(f"{port:<10} 监听中...   ", end="", flush=True)
        ok, preview = scan_port(port)
        if ok:
            status = ">>> 找到!"
            found_port = port
        else:
            status = "无应答"
        print(f"\r{port:<10} {status:<12} {preview[:50]}")

    print("-" * 70)

    if found_port:
        print(f"\n[OK] MSPM0 在 {found_port}")
    else:
        print(f"\n[!] 未找到 MSPM0 设备")
        print("    请确认:")
        print("    1. MSPM0 已上电")
        print("    2. USB 转串口 RX 接 MSPM0 PA.10, GND 接 GND")
        print("    3. 上电后 30 秒内运行本脚本")

    return found_port


def connect_and_monitor(port: str):
    """调用 serial_monitor.py 连接并显示数据"""
    monitor_script = os.path.join(SCRIPT_DIR, "serial_monitor.py")
    if os.path.exists(monitor_script):
        print(f"\n启动监视器: {port}\n")
        os.system(f'python "{monitor_script}" {port} {BAUD_RATE}')
    else:
        # 内置简易监视器
        print(f"\n连接 {port} @ {BAUD_RATE} (Ctrl+C 停止)\n")
        try:
            ser = serial.Serial(port, baudrate=BAUD_RATE, timeout=0.1)
            with open(LOG_FILE, "a", encoding="utf-8") as log:
                log.write(f"\n--- {time.strftime('%Y-%m-%d %H:%M:%S')} "
                          f"连接 {port} ---\n")
                while True:
                    if ser.in_waiting:
                        raw = ser.read(ser.in_waiting)
                        text = raw.decode("utf-8", errors="replace")
                        sys.stdout.write(text)
                        sys.stdout.flush()
                        log.write(text)
                        log.flush()
                    time.sleep(0.01)
        except KeyboardInterrupt:
            print("\n[STOP] 已断开")
        finally:
            if 'ser' in locals() and ser.is_open:
                ser.close()


def main():
    if len(sys.argv) >= 2:
        arg = sys.argv[1].upper()
        if arg == "--SCAN":
            find_mspm0_port()
            return
        # 直接指定 COM 口
        port = arg if arg.startswith("COM") else f"COM{arg}"
        connect_and_monitor(port)
        return

    # 默认: 扫描并连接
    port = find_mspm0_port()
    if port:
        input("\n按 Enter 开始监听, Ctrl+C 退出...")
        connect_and_monitor(port)


if __name__ == "__main__":
    main()
