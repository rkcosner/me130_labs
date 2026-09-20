#!/usr/bin/env bash
#
# ME130 Raspberry Pi Golden-Image Setup
# Target: Raspberry Pi 4 Model B, Ubuntu 24.04
#
# Run ONCE while creating the golden image:
#   chmod +x setup.sh
#   sudo ./setup.sh
#
# This installs/configures the common environment only. It intentionally does
# NOT clone course repositories, authenticate GitHub, or install solutions.
#
set -Eeuo pipefail

readonly CONFIG_FILE="/boot/firmware/config.txt"
readonly MODULES_FILE="/etc/modules-load.d/me130.conf"
readonly BEGIN="# >>> ME130 MANAGED HARDWARE CONFIG >>>"
readonly END="# <<< ME130 MANAGED HARDWARE CONFIG <<<"

log(){ printf '\n[ME130] %s\n' "$*"; }
warn(){ printf '[WARN] %s\n' "$*" >&2; }
die(){ printf '[FAIL] %s\n' "$*" >&2; exit 1; }

[[ $EUID -eq 0 ]] || die "Run with sudo: sudo ./setup.sh"
[[ -f "$CONFIG_FILE" ]] || die "Missing $CONFIG_FILE"

source /etc/os-release
[[ "${ID:-}" == "ubuntu" ]] || die "This script expects Ubuntu."
[[ "${VERSION_ID:-}" == "24.04" ]] || warn "Verified target is Ubuntu 24.04; detected ${PRETTY_NAME:-unknown}."

if [[ -r /proc/device-tree/model ]]; then
    MODEL="$(tr -d '\0' </proc/device-tree/model)"
    log "Hardware: $MODEL"
    [[ "$MODEL" == *"Raspberry Pi 4"* ]] || warn "ME130 was developed for Raspberry Pi 4."
fi

log "Installing required packages..."
apt-get update
apt-get install -y \
    build-essential cmake git \
    gpiod libgpiod-dev \
    i2c-tools \
    libeigen3-dev \
    python3 python3-pip python3-numpy python3-pandas python3-matplotlib \
    openssh-server avahi-daemon

systemctl enable ssh
systemctl enable avahi-daemon

# ---------------------------------------------------------------------------
# ROS 2 Jazzy
# ---------------------------------------------------------------------------
log "Configuring locale (ROS 2 requires a UTF-8 locale)..."
apt-get install -y locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

log "Enabling the universe component..."
apt-get install -y software-properties-common curl
add-apt-repository -y universe

# ros-jazzy-ros-base depends on liblz4-dev / libzstd-dev at versions that live
# in the -updates pocket. Some Pi images ship with only the release pocket
# enabled in the primary archive stanza, and the install then fails to resolve
# them. Ensure -updates is listed before we try.
readonly UBUNTU_SOURCES="/etc/apt/sources.list.d/ubuntu.sources"
if [[ -f "$UBUNTU_SOURCES" ]]; then
    cp -a "$UBUNTU_SOURCES" "${UBUNTU_SOURCES}.me130-backup-$(date +%Y%m%d-%H%M%S)"
    python3 - "$UBUNTU_SOURCES" "${VERSION_CODENAME}" <<'PY'
from pathlib import Path
import sys
path, codename = Path(sys.argv[1]), sys.argv[2]
want = codename + "-updates"
lines = path.read_text().splitlines()
for i, line in enumerate(lines):
    if not line.lower().startswith("suites:"):
        continue
    suites = line.split(":", 1)[1].split()
    # Leave the security stanza alone; only the primary archive needs -updates.
    if any(s.endswith("-security") for s in suites):
        continue
    if want not in suites:
        lines[i] = "Suites: " + " ".join(suites + [want])
        path.write_text("\n".join(lines) + "\n")
        print("added " + want)
    else:
        print(want + " already enabled")
    break
PY
else
    warn "No $UBUNTU_SOURCES; skipping the -updates check."
fi

apt-get update

log "Installing the ROS 2 apt source..."
ROS_APT_SOURCE_VERSION="$(curl -sfL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | awk -F'"' '{print $4}')"
[[ -n "$ROS_APT_SOURCE_VERSION" ]] || die "Could not resolve the latest ros-apt-source release (no network, or GitHub API rate limit)."
log "  ros-apt-source $ROS_APT_SOURCE_VERSION"
curl -fL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
dpkg -i /tmp/ros2-apt-source.deb
rm -f /tmp/ros2-apt-source.deb
apt-get update

log "Installing ROS 2 Jazzy (ros-base) and build tools..."
apt-get install -y \
    ros-jazzy-ros-base \
    ros-dev-tools \
    python3-colcon-common-extensions \
    liblz4-dev libzstd-dev

# Make ros2 available in every login shell on the image.
cat >/etc/profile.d/me130-ros2.sh <<'EOF'
# ME130: put the ROS 2 Jazzy environment on every login shell.
# Guarded on bash: setup.bash is not POSIX sh.
if [ -n "${BASH_VERSION:-}" ] && [ -f /opt/ros/jazzy/setup.bash ]; then
    . /opt/ros/jazzy/setup.bash
fi
EOF
chmod 0644 /etc/profile.d/me130-ros2.sh

log "Configuring I2C and both hardware PWM channels..."
PWM_OVERLAY="/boot/firmware/overlays/pwm-2chan.dtbo"
[[ -f "$PWM_OVERLAY" ]] || die "Missing PWM overlay: $PWM_OVERLAY"

BACKUP="${CONFIG_FILE}.me130-backup-$(date +%Y%m%d-%H%M%S)"
cp -a "$CONFIG_FILE" "$BACKUP"

# Remove a previous ME130-managed block.
python3 - "$CONFIG_FILE" "$BEGIN" "$END" <<'PY'
from pathlib import Path
import sys
p=Path(sys.argv[1]); begin=sys.argv[2]; end=sys.argv[3]
out=[]; inside=False
for line in p.read_text().splitlines():
    if line.strip()==begin: inside=True; continue
    if line.strip()==end: inside=False; continue
    if not inside: out.append(line)
p.write_text("\n".join(out).rstrip()+"\n")
PY

# Disable active duplicate/conflicting declarations; the managed block below is
# the single authoritative declaration.
python3 - "$CONFIG_FILE" <<'PY'
from pathlib import Path
import re, sys
p=Path(sys.argv[1])
patterns=[
    re.compile(r'^\s*dtparam\s*=\s*i2c_arm\s*=\s*(?:on|off)\s*$',re.I),
    re.compile(r'^\s*dtoverlay\s*=\s*pwm-2chan(?:,.*)?\s*$',re.I),
]
out=[]
for line in p.read_text().splitlines():
    if not line.lstrip().startswith("#") and any(x.match(line) for x in patterns):
        out.append("# ME130: superseded by managed block: "+line)
    else: out.append(line)
p.write_text("\n".join(out).rstrip()+"\n")
PY

cat >>"$CONFIG_FILE" <<EOF

$BEGIN
# I2C1: GPIO2=SDA, GPIO3=SCL. MPU6050=0x68; AS5600 normally=0x36.
dtparam=i2c_arm=on

# Hardware PWM used by ME130:
# GPIO18 -> PWM0, GPIO19 -> PWM1
dtoverlay=pwm-2chan,pin=18,func=2,pin2=19,func2=2
$END
EOF

cat >"$MODULES_FILE" <<'EOF'
# ME130 user-space I2C interface
i2c-dev
EOF
modprobe i2c-dev || warn "i2c-dev will be loaded at next boot."

# ---------------------------------------------------------------------------
# Hardware access for the lab user
# ---------------------------------------------------------------------------
# Without this the ROS nodes die on startup with "Could not open /dev/gpiochip0"
# or ".../pwm/pwmchip0/export". Running `ros2 launch` under sudo is NOT a fix:
# sudo drops the ROS environment, so the launch file and workspace overlay are
# not found. Grant the login user access instead.
if [[ -x "$(dirname "$0")/me130_permissions.sh" ]]; then
    log "Applying hardware permissions..."
    "$(dirname "$0")/me130_permissions.sh"
else
    warn "me130_permissions.sh not found next to this script; run it separately."
fi

log "Verifying installed software..."
for cmd in g++ cmake git gpiodetect i2cdetect python3 colcon; do
    command -v "$cmd" >/dev/null || die "$cmd is missing."
done

[[ -f /opt/ros/jazzy/setup.bash ]] || die "ROS 2 Jazzy is missing (/opt/ros/jazzy/setup.bash)."
# A fresh bash: this script runs under `set -u`, which ROS setup scripts trip.
bash -c 'set -e; . /opt/ros/jazzy/setup.bash; ros2 pkg list >/dev/null' \
    || die "ROS 2 installed but the ros2 CLI is not functional."
log "ROS 2 Jazzy OK: $(bash -c '. /opt/ros/jazzy/setup.bash; ros2 pkg list | wc -l') packages"

python3 - <<'PY'
import numpy, pandas, matplotlib
print("numpy", numpy.__version__)
print("pandas", pandas.__version__)
print("matplotlib", matplotlib.__version__)
PY

grep -Fqx "dtparam=i2c_arm=on" "$CONFIG_FILE" || die "I2C boot configuration failed."
grep -Fqx "dtoverlay=pwm-2chan,pin=18,func=2,pin2=19,func2=2" "$CONFIG_FILE" || die "PWM boot configuration failed."

cat <<'EOF'

ME130 golden-image base setup is complete.

NEXT:
  1. sudo reboot
  2. Verify:
       gpiodetect
       ls -l /dev/i2c-1
       sudo i2cdetect -y 1
       ls -l /sys/class/pwm/
       ls -l /sys/class/pwm/pwmchip0/
       ros2 --help          # in a NEW shell, so /etc/profile.d is picked up
  3. Temporarily clone/build the PUBLIC student repositories for verification.
  4. Delete those verification clones.
  5. Capture the golden image.

WIFI:
  A Wi-Fi profile configured on this golden Pi will normally be included in a
  full SD-card image. Only bake in a course/shared network profile that is
  approved for distribution. Never bake personal/instructor credentials into
  the image.

IMPORTANT:
  Do not put instructor solutions, private Git repositories, GitHub credentials,
  personal SSH keys, or student work on the golden image.

After flashing the golden image onto each physical lab Pi, run:
  sudo ./provision.sh <number>

Example:
  sudo ./provision.sh 7
which provisions hostname me130-pi-07 and regenerates SSH host keys.
EOF
