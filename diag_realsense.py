"""
Диагностика RealSense на macOS: собирает всё сразу, чтобы не гонять
по одной команде. Запускать из того терминала, где есть доступ к камере.
"""
import subprocess
import sys


def sh(cmd):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True,
                              text=True, timeout=20).stdout.strip()
    except Exception as e:
        return f"(не выполнилось: {e})"


print("=" * 60)
print("1. ВИДИТ ЛИ СИСТЕМА КАМЕРУ")
print("=" * 60)
usb = sh("system_profiler SPUSBDataType | grep -A 8 -i realsense")
print(usb or "камера в USB не найдена!")

print()
print("=" * 60)
print("2. ДОСТУП К КАМЕРЕ (AVFoundation)")
print("=" * 60)
try:
    import cv2
    cap = cv2.VideoCapture(0)
    print("AVFoundation отдаёт камеру:", cap.isOpened())
    cap.release()
except Exception as e:
    print("ошибка:", e)

print()
print("=" * 60)
print("3. КТО ЕЩЁ ДЕРЖИТ КАМЕРУ ПРЯМО СЕЙЧАС")
print("=" * 60)
# Приложение, захватившее камеру, не даст libusb забрать интерфейс.
busy = sh("lsof 2>/dev/null | grep -iE 'AppleCamera|VDCAssistant|appleh13camerad' "
          "| awk '{print $1}' | sort -u")
print(busy or "(явных захватчиков не видно)")
print("Процессы, типично занимающие камеру:")
print(sh("pgrep -l 'zoom|Teams|Photo Booth|FaceTime|obs|Skype|Webex|Camera' "
         "| head") or "(таких нет)")

print()
print("=" * 60)
print("4. ЧТО ГОВОРИТ САМ SDK")
print("=" * 60)
try:
    import pyrealsense2 as rs
except ImportError as e:
    print("pyrealsense2 не импортируется:", e)
    sys.exit(1)

rs.log_to_file(rs.log_severity.debug, "rs_debug.log")
try:
    devs = list(rs.context().query_devices())
    print("устройств найдено:", len(devs))
    for d in devs:
        print("  имя:", d.get_info(rs.camera_info.name))
        print("  прошивка:", d.get_info(rs.camera_info.firmware_version))
        print("  USB:", d.get_info(rs.camera_info.usb_type_descriptor))
        print()
        print("  доступные режимы:")
        for s in d.sensors:
            nm = s.get_info(rs.camera_info.name)
            seen = set()
            for pr in s.profiles:
                try:
                    v = pr.as_video_stream_profile()
                    key = (str(pr.stream_type()), v.width(), v.height(), pr.fps())
                except Exception:
                    continue
                if key not in seen:
                    seen.add(key)
            for k in sorted(seen)[:14]:
                print(f"    [{nm}] {k[0]:22s} {k[1]}x{k[2]} @ {k[3]}")
except Exception as e:
    print("ОШИБКА при опросе устройств:", type(e).__name__, e)

print()
print("=" * 60)
print("5. КЛЮЧЕВЫЕ СТРОКИ ИЗ ЛОГА SDK")
print("=" * 60)
try:
    with open("rs_debug.log", encoding="utf-8", errors="replace") as f:
        lines = [l.rstrip() for l in f
                 if any(w in l for w in ("ERROR", "WARN", "claim", "power",
                                         "ACCESS", "permission"))]
    for l in lines[:25]:
        print(" ", l)
    if not lines:
        print("  (ошибок в логе нет — хороший знак)")
except FileNotFoundError:
    print("  лог не создался")

print()
print("Полный лог: rs_debug.log")
