#!/usr/bin/env bash
# Set up a fresh Radxa ROCK 3C (Radxa OS, Debian 12 bookworm) as the robot computer.
# Safe to run again: each step checks first and changes only what is different (idempotent).
#
#   bash robot/setup/install.sh --dry-run        print every step and file, change nothing
#   bash robot/setup/install.sh                  do it (asks for the sudo password)
#   bash robot/setup/install.sh --list-usb       print the USB ids and serial numbers of the connected devices
#   bash robot/setup/install.sh --render DIR     only write the udev and systemd files into DIR (for checks)
#   bash robot/setup/install.sh --keep-desktop   do not switch the boot to text mode
#
# Values come from robot/config.toml: CAN serial numbers and bitrate, USB ids, IMU UART and overlay.
# Sources for the ROCK 3C facts: robot/README.md, section "ROCK 3C facts".
set -euo pipefail

REPO=$(cd "$(dirname "$0")/../.." && pwd)
DRY=0 KEEP_DESKTOP=0 RENDER="" REBOOT=0
USER_NAME=${SUDO_USER:-$USER}
USER_HOME=$(getent passwd "$USER_NAME" | cut -d: -f6)

while [[ $# -gt 0 ]]; do
  case $1 in
    --dry-run) DRY=1 ;;
    --keep-desktop) KEEP_DESKTOP=1 ;;
    --render) RENDER=$2; shift ;;
    --list-usb) LIST=1 ;;
    -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
    *) echo "unknown option $1"; exit 2 ;;
  esac
  shift
done

step() { echo; echo "== $*"; }
run() {  # run a command, or print it in dry-run mode
  if [[ $DRY == 1 ]]; then echo "  would run: $*"; else echo "  run: $*"; "$@"; fi
}
put() {  # put DEST MODE < content. Writes only when the content is different. Returns 0 when it wrote.
  local dest=$1 mode=$2 tmp
  tmp=$(mktemp)
  cat > "$tmp"
  if [[ -n $RENDER ]]; then
    install -D -m "$mode" "$tmp" "$RENDER/$(basename "$dest")"; rm -f "$tmp"; echo "  rendered $RENDER/$(basename "$dest")"; return 0
  fi
  if [[ -f $dest ]] && cmp -s "$tmp" "$dest"; then echo "  unchanged: $dest"; rm -f "$tmp"; return 1; fi
  if [[ $DRY == 1 ]]; then
    echo "  would write $dest:"; sed 's/^/    | /' "$tmp"; rm -f "$tmp"; return 0
  fi
  sudo install -D -m "$mode" "$tmp" "$dest"; rm -f "$tmp"; echo "  wrote $dest"; return 0
}

PY=$REPO/.venv/bin/python
[[ -x $PY ]] || PY=python3  # before uv sync: Debian 12 python3 (3.11) has tomllib
cfg() { (cd "$REPO" && "$PY" robot/robot_config.py "$1"); }

if [[ ${LIST:-0} == 1 ]]; then
  echo "USB devices (vendor:product, serial, name):"
  for d in /sys/bus/usb/devices/*; do
    [[ -f $d/idVendor ]] || continue
    printf "  %s:%s  serial=%-26s %s %s\n" "$(cat "$d/idVendor")" "$(cat "$d/idProduct")" "$(cat "$d/serial" 2>/dev/null || echo -)" \
      "$(cat "$d/manufacturer" 2>/dev/null || true)" "$(cat "$d/product" 2>/dev/null || true)"
  done
  echo "CAN interfaces (the serial goes into config.toml [can] left_serial or right_serial):"
  for n in /sys/class/net/*; do
    [[ $(cat "$n/type") == 280 ]] || continue
    echo "  $(basename "$n"): serial $(cat "$n/device/../serial" 2>/dev/null || echo unknown)"
  done
  echo "Video devices:"; ls -l /dev/v4l/by-id/ 2>/dev/null | sed 's/^/  /' || echo "  none"
  echo "Serial devices:"; ls -l /dev/serial/by-id/ 2>/dev/null | sed 's/^/  /' || echo "  none"
  exit 0
fi

# ---- values from config.toml
CAN_USB=$(cfg can.usb) LEFT=$(cfg can.left_serial) RIGHT=$(cfg can.right_serial)
read -r CAN_L CAN_R < <(cfg can.channels | tr -d '[]",' | xargs)
BITRATE=$(cfg can.bitrate) TXQ=$(cfg can.txqueuelen) RESTART=$(cfg can.restart_ms)
SERVO_USB=$(cfg actuator.servo_usb) CAM_USB=$(cfg camera.usb) CAM_DEV=$(cfg camera.device)
IMU_UART=$(cfg imu.uart) IMU_OVERLAY=$(cfg imu.overlay)

if [[ -z $RENDER ]]; then
  step "0. Check the computer"
  arch=$(uname -m); os=$(. /etc/os-release && echo "$PRETTY_NAME")
  echo "  $os, $arch, user $USER_NAME, repository $REPO"
  [[ $arch == aarch64 ]] || echo "  WARNING: this is not the ROCK 3C (aarch64). Use --dry-run here."
  command -v apt-get >/dev/null || { echo "  WARNING: no apt-get: only --dry-run makes sense here."; [[ $DRY == 1 ]] || exit 1; }

  step "1. System packages"
  PKGS=(git curl build-essential ffmpeg v4l-utils can-utils gpiod picocom)
  missing=()
  for p in "${PKGS[@]}"; do dpkg -s "$p" >/dev/null 2>&1 || missing+=("$p"); done
  if [[ ${#missing[@]} -gt 0 ]]; then
    run sudo apt-get update
    run sudo apt-get install -y "${missing[@]}"
  else
    echo "  all installed: ${PKGS[*]}"
  fi

  step "2. User groups (serial ports, camera, GPIO)"
  getent group gpio >/dev/null || run sudo groupadd --system gpio
  for g in dialout video plugdev gpio; do
    if id -nG "$USER_NAME" | tr ' ' '\n' | grep -qx "$g"; then echo "  $USER_NAME is in $g"; else run sudo usermod -aG "$g" "$USER_NAME"; fi
  done

  step "3. uv and the Python environment (with the board extra: gpiod)"
  UV=$USER_HOME/.local/bin/uv
  if command -v uv >/dev/null; then UV=$(command -v uv); echo "  uv found: $UV"
  elif [[ ! -x $UV ]]; then
    if [[ $DRY == 1 ]]; then echo "  would run: curl -LsSf https://astral.sh/uv/install.sh | sh"
    else curl -LsSf https://astral.sh/uv/install.sh | sh; fi
  fi
  (cd "$REPO" && run "$UV" sync --extra board)
  PY=$REPO/.venv/bin/python; [[ -x $PY ]] || PY=python3
fi

step "4. udev rules: stable device names"
RULES="# Written by robot/setup/install.sh from robot/config.toml. Edit config.toml and run the script again.
# GPIO character devices for the gpio group (pump pin, e-stop input)
SUBSYSTEM==\"gpio\", KERNEL==\"gpiochip*\", GROUP=\"gpio\", MODE=\"0660\"
# Waveshare Bus Servo Adapter (A), CH343: /dev/servo
SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"${SERVO_USB%%:*}\", ATTRS{idProduct}==\"${SERVO_USB##*:}\", SYMLINK+=\"servo\", GROUP=\"dialout\", MODE=\"0660\"
# IMU on the header UART: /dev/imu
SUBSYSTEM==\"tty\", KERNEL==\"$IMU_UART\", SYMLINK+=\"imu\", GROUP=\"dialout\", MODE=\"0660\"
"
if [[ -n $CAM_USB ]]; then
  RULES+="# Arducam B0201, capture node only (index 0): $CAM_DEV
SUBSYSTEM==\"video4linux\", ATTRS{idVendor}==\"${CAM_USB%%:*}\", ATTRS{idProduct}==\"${CAM_USB##*:}\", ATTR{index}==\"0\", SYMLINK+=\"${CAM_DEV#/dev/}\", GROUP=\"video\", MODE=\"0660\"
"
else
  echo "  NOTE: config.toml [camera] usb is empty: no $CAM_DEV symlink. Read the id with --list-usb."
fi
if [[ -n $LEFT && -n $RIGHT ]]; then
  for side in "$LEFT $CAN_L" "$RIGHT $CAN_R"; do
    read -r serial name <<< "$side"
    RULES+="# CANable 2.0 Pro (candleLight, gs_usb) with serial $serial: $name, set up by robot-can@$name.service
SUBSYSTEM==\"net\", ACTION==\"add\", ATTRS{idVendor}==\"${CAN_USB%%:*}\", ATTRS{idProduct}==\"${CAN_USB##*:}\", ATTRS{serial}==\"$serial\", NAME=\"$name\", TAG+=\"systemd\", ENV{SYSTEMD_WANTS}+=\"robot-can@$name.service\"
"
  done
else
  RULES+="# CANable adapters with no serial in config.toml: the kernel names them can0, can1. Each one is still set up.
SUBSYSTEM==\"net\", ACTION==\"add\", ATTRS{idVendor}==\"${CAN_USB%%:*}\", ATTRS{idProduct}==\"${CAN_USB##*:}\", TAG+=\"systemd\", ENV{SYSTEMD_WANTS}+=\"robot-can@%k.service\"
"
  echo "  NOTE: config.toml [can] left_serial or right_serial is empty. Read them with --list-usb, then run this again."
fi
UDEV_CHANGED=0
put /etc/udev/rules.d/70-robot.rules 644 <<< "$RULES" && UDEV_CHANGED=1

step "5. CAN buses at boot: $BITRATE bit/s, txqueuelen $TXQ, restart $RESTART ms after bus-off"
put /etc/systemd/system/robot-can@.service 644 <<EOF || true
# Written by robot/setup/install.sh. udev starts one instance for each CANable (70-robot.rules).
[Unit]
Description=Robot CAN bus %i: bitrate, queue length, up
BindsTo=sys-subsystem-net-devices-%i.device
After=sys-subsystem-net-devices-%i.device

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStartPre=-ip link set %i down
ExecStart=ip link set %i type can bitrate $BITRATE restart-ms $RESTART
ExecStart=ip link set %i txqueuelen $TXQ
ExecStart=ip link set %i up
ExecStop=ip link set %i down
EOF
put /etc/modules-load.d/robot-can.conf 644 <<< "gs_usb" || true

step "6. Robot software service (installed, NOT enabled: no autostart)"
put /etc/systemd/system/robot@.service 644 <<EOF || true
# Written by robot/setup/install.sh. Instances: robot@bench (no legs), robot@full (the whole robot).
# Start: sudo systemctl start robot@bench. Autostart at boot: sudo systemctl enable robot@bench.
# The legs wait for "ready": run .venv/bin/python robot/launch.py --send ready when the robot stands.
[Unit]
Description=Firefighter robot software, robot/launch.py --%i
After=network.target
Wants=network.target

[Service]
User=$USER_NAME
WorkingDirectory=$REPO
ExecStart=$REPO/.venv/bin/python robot/launch.py --%i
KillSignal=SIGINT
TimeoutStopSec=20
Restart=no

[Install]
WantedBy=multi-user.target
EOF

step "7. CPU governor: performance (Radxa default: ondemand), so the 25 Hz policy loop does not wait for a frequency step"
put /etc/tmpfiles.d/scaling_governor.conf 644 <<'EOF' || true
# Written by robot/setup/install.sh. Same name as the Radxa file in /usr/lib/tmpfiles.d, so this one replaces it.
w- /sys/devices/system/cpu/cpufreq/policy*/scaling_governor - - - - performance
EOF

[[ -n $RENDER ]] && exit 0

if [[ $UDEV_CHANGED == 1 ]]; then
  run sudo udevadm control --reload
  run sudo udevadm trigger
fi
run sudo systemctl daemon-reload
run sudo systemd-tmpfiles --create /etc/tmpfiles.d/scaling_governor.conf

step "8. IMU UART overlay: $IMU_OVERLAY (/dev/$IMU_UART)"
if [[ -f /boot/dtbo/$IMU_OVERLAY.dtbo ]]; then
  echo "  enabled: /boot/dtbo/$IMU_OVERLAY.dtbo"
elif command -v rsetup >/dev/null || [[ $DRY == 1 ]]; then
  run sudo rsetup enable_overlays "$IMU_OVERLAY.dtbo"
  REBOOT=1
else
  echo "  WARNING: no rsetup. Enable $IMU_OVERLAY by hand (robot/README.md)."
fi
if [[ $IMU_UART == ttyS2 ]] && grep -q 'console=ttyFIQ0' /etc/kernel/cmdline 2>/dev/null; then
  echo "  pins 8 and 10 (UART2) carry the Linux console: remove it from /etc/kernel/cmdline"
  run sudo sed -i -E 's/ ?console=ttyFIQ0[^ ]*//g; s/ ?console=ttyS2[^ ]*//g' /etc/kernel/cmdline
  run sudo u-boot-update
  REBOOT=1
fi

step "9. Swap: zram is on by default. Add a 1 GiB swap file when the RAM is below 3 GB."
mem_kb=$(awk '/MemTotal/ {print $2}' /proc/meminfo)
if [[ $mem_kb -lt 3000000 && ! -f /swapfile ]]; then
  run sudo fallocate -l 1G /swapfile
  run sudo chmod 600 /swapfile
  run sudo mkswap /swapfile
  run sudo swapon -p -2 /swapfile
  grep -q '^/swapfile' /etc/fstab || run sudo sh -c 'echo "/swapfile none swap sw,pri=-2 0 0" >> /etc/fstab'
else
  echo "  no change: RAM $((mem_kb / 1024)) MiB, /swapfile $([[ -f /swapfile ]] && echo exists || echo 'not needed')"
fi

step "10. Boot to text mode (the KDE desktop takes RAM and CPU). Undo: sudo systemctl set-default graphical.target"
if [[ $KEEP_DESKTOP == 1 ]]; then
  echo "  --keep-desktop: no change"
elif [[ $(systemctl get-default) == graphical.target ]]; then
  run sudo systemctl set-default multi-user.target
  REBOOT=1
else
  echo "  already $(systemctl get-default)"
fi

step "Done"
[[ $REBOOT == 1 ]] && echo "  Reboot now: sudo reboot" || echo "  No reboot needed. Log out and in again when step 2 added a group."
