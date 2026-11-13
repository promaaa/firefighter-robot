"""설치 상태와 연결된 장치(ODrive, CAN 어댑터)의 Windows 드라이버를 확인한다. 장치에 아무것도 쓰지 않는다.

    python 00_env_check.py
"""
import importlib
import json
import re
import subprocess
import sys

DEVICES = {
    "1209:0D32": ("ODrive / XDrive", "Interface 2(MI_02)가 libusb0 이어야 odrivetool 0.5.1이 연결됨 (Zadig: libusb-win32)"),
    "0483:DF11": ("STM32 DFU 부트로더", "펌웨어 업로드 모드. 드라이버를 바꾸지 마세요"),
    "1D50:606F": ("candleLight (gs_usb)", "WinUSB 이어야 함. --interface gs_usb"),
    "16D0:117E": ("CANable slcan", "COM 포트(usbser). --interface slcan --channel COMx"),
    "0C72:000C": ("PEAK PCAN-USB", "PEAK 드라이버 필요. --interface pcan"),
}

PS = r"""
[Console]::OutputEncoding = [Text.Encoding]::UTF8
$ids = 'VID_1209&PID_0D32|VID_0483&PID_DF11|VID_1D50&PID_606F|VID_16D0&PID_117E|VID_0C72&PID_000C'
$out = @(Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue | Where-Object { $_.InstanceId -match $ids } | ForEach-Object {
  $svc = (Get-PnpDeviceProperty -InstanceId $_.InstanceId -KeyName DEVPKEY_Device_Service -ErrorAction SilentlyContinue).Data
  [pscustomobject]@{ Name = $_.FriendlyName; Id = $_.InstanceId; Status = "$($_.Status)"; Service = $svc }
})
ConvertTo-Json -InputObject $out -Compress
"""


def version(mod):
    try:
        m = importlib.import_module(mod)
        return getattr(m, "__version__", "설치됨")
    except Exception as e:
        return f"없음 ({type(e).__name__})"


def main():
    print("== Python ==")
    print(f"  {sys.version.split()[0]}  {sys.executable}")
    if "odrv051" not in sys.executable:
        print("  !! odrv051 가상환경이 아닙니다. %USERPROFILE%\\odrv051\\Scripts\\activate 후 다시 실행하세요.")

    print("== 패키지 ==")
    for mod in ("odrive", "can", "usb", "serial", "gs_usb", "libusb_package", "numpy", "matplotlib"):
        print(f"  {mod:15s} {version(mod)}")

    print("== pyusb 백엔드 ==")
    try:
        import usb.backend.libusb0 as b0
        ok0 = b0.get_backend() is not None
        print(f"  libusb0 (ODrive용, Zadig libusb-win32가 설치): {'있음' if ok0 else '없음 -> ODrive에 Zadig 먼저'}")
    except Exception as e:
        print(f"  libusb0 확인 실패: {e}")
    try:
        import libusb_package
        print(f"  libusb-1.0 (candleLight용, 스크립트가 직접 로드): {libusb_package.get_library_path()}")
    except Exception as e:
        print(f"  libusb-1.0 확인 실패: {e}")

    print("== 연결된 장치 (Windows) ==")
    try:
        r = subprocess.run(["powershell", "-NoProfile", "-Command", PS], capture_output=True, timeout=60)
        devs = json.loads(r.stdout.decode("utf-8", "replace").strip() or "[]")
    except Exception as e:
        devs = []
        print(f"  장치 조회 실패: {e}")
    if not devs:
        print("  해당 장치 없음 (DC 전원과 USB 케이블을 확인하세요. 충전 전용 케이블은 안 됩니다).")
    for d in devs:
        m = re.search(r"VID_([0-9A-F]{4})&PID_([0-9A-F]{4})", d["Id"], re.I)
        key = f"{m.group(1)}:{m.group(2)}".upper() if m else "?"
        name, hint = DEVICES.get(key, ("?", ""))
        print(f"  [{name}] {d['Name']}")
        print(f"      {d['Id']}  상태={d['Status']}  드라이버={d['Service']}")
        if key == "1209:0D32":
            if "MI_02" in d["Id"]:
                ok = (d["Service"] or "").lower() == "libusb0"
                print("      -> OK: odrivetool 0.5.1이 연결할 수 있음" if ok else
                      "      !! 이 인터페이스에 Zadig로 libusb-win32를 설치해야 합니다.")
        elif hint:
            print(f"      -> {hint}")

    print("== COM 포트 ==")
    try:
        from serial.tools import list_ports
        for p in list_ports.comports():
            vidpid = f"{p.vid:04X}:{p.pid:04X}" if p.vid is not None else "----:----"
            print(f"  {p.device:8s} {vidpid}  {p.description}")
    except Exception as e:
        print(f"  조회 실패: {e}")


if __name__ == "__main__":
    main()
