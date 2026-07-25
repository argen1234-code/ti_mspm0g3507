"""
串口数据监视器 -- Serial Monitor
==================================
功能:
  1. 列出所有可用串口
  2. 连接指定串口并持续接收数据
  3. 所有数据实时显示 + 自动写入日志文件
  4. Claude 可通过读取日志文件获取接收数据

用法:
  python serial_monitor.py                  # 列出串口，交互选择
  python serial_monitor.py COM3             # 直接连接 COM3
  python serial_monitor.py COM3 115200      # 指定波特率

日志文件: serial_log.txt (自动创建于脚本所在目录)
"""

import sys
import os
import time
import threading
from datetime import datetime

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[ERROR] 需要安装 pyserial: pip install pyserial")
    sys.exit(1)

# ---- 默认参数 ----
DEFAULT_BAUD = 115200
DEFAULT_TIMEOUT = 0.1  # 秒 (非阻塞读取)
LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "serial_log.txt")

# ---- 全局状态 ----
g_ser: serial.Serial | None = None
g_running = True
g_byte_count = 0
g_lock = threading.Lock()


def list_ports():
    """列出可用串口"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("[INFO] 没有发现可用串口")
        return []
    print("\n可用的串口:")
    print("-" * 50)
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  —  {p.description}")
    print("-" * 50)
    return list(ports)


def connect_port(port: str, baud: int) -> serial.Serial | None:
    """打开串口"""
    try:
        ser = serial.Serial(port, baudrate=baud, timeout=DEFAULT_TIMEOUT)
        print(f"[OK] 已连接 {port} @ {baud} baud")
        return ser
    except Exception as e:
        print(f"[ERROR] 无法打开 {port}: {e}")
        return None


def reader_thread(ser: serial.Serial, log_path: str):
    """后台线程: 持续读取串口数据并写入日志"""
    global g_running, g_byte_count

    print(f"[LOG] 日志文件: {log_path}")
    print("[RUN] 开始接收数据... (按 Ctrl+C 停止)\n")

    with open(log_path, "a", encoding="utf-8") as f:
        f.write(f"\n=== {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} "
                f"打开 {ser.port} @ {ser.baudrate} ===\n")
        f.flush()

        while g_running:
            try:
                if ser.is_open and ser.in_waiting > 0:
                    raw = ser.read(ser.in_waiting)
                    if raw:
                        with g_lock:
                            g_byte_count += len(raw)

                        # 解码 (尝试 UTF-8, 失败则显示 hex)
                        try:
                            text = raw.decode("utf-8", errors="replace")
                        except Exception:
                            text = raw.hex(" ")

                        # 终端显示
                        sys.stdout.write(text)
                        sys.stdout.flush()

                        # 写入日志
                        f.write(text)
                        f.flush()
                else:
                    time.sleep(0.01)
            except serial.SerialException as e:
                print(f"\n[ERROR] 串口异常: {e}")
                break
            except Exception as e:
                print(f"\n[ERROR] 读取异常: {e}")
                time.sleep(0.1)


def main():
    global g_running

    script_dir = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(script_dir, "serial_log.txt")

    # ---- 解析命令行参数 ----
    if len(sys.argv) >= 2:
        port = sys.argv[1].upper()
        if not port.startswith("COM"):
            port = f"COM{port}"
        baud = int(sys.argv[2]) if len(sys.argv) >= 3 else DEFAULT_BAUD
    else:
        ports = list_ports()
        if not ports:
            return
        try:
            idx = int(input("\n选择串口编号 (0~N): ").strip())
            port = ports[idx].device
        except (ValueError, IndexError):
            print("[ERROR] 无效选择")
            return
        baud_input = input(f"波特率 (默认 {DEFAULT_BAUD}): ").strip()
        baud = int(baud_input) if baud_input else DEFAULT_BAUD

    # ---- 连接 ----
    ser = connect_port(port, baud)
    if ser is None:
        return

    # ---- 清空旧日志 ----
    with open(log_path, "w", encoding="utf-8") as f:
        f.write(f"# Serial Monitor Log\n"
                f"# Port: {port}  Baud: {baud}\n"
                f"# Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
                f"# {'='*60}\n\n")

    # ---- 启动读线程 ----
    t = threading.Thread(target=reader_thread, args=(ser, log_path), daemon=True)
    t.start()

    try:
        while t.is_alive():
            t.join(1)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"\n\n[STOP] 共接收 {g_byte_count} 字节")
        with open(log_path, "a", encoding="utf-8") as f:
            f.write(f"\n=== {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} "
                    f"关闭, 共 {g_byte_count} 字节 ===\n")
        g_running = False
        if ser and ser.is_open:
            ser.close()
            print(f"[OK] 串口已关闭")


if __name__ == "__main__":
    main()
