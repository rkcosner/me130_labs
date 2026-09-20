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

log "Configuring I2C and both hardware PWM channels..."
PWM_OVERLAY="/boot/firmware/overlays/pwm-2chan.dtbo"
[[ -f "$PWM_OVERLAY" ]] || die "Missing PWM overlay: $PWM_OVERLAY"

BACKUP="${CONFIG_FILE}.me130-backup-$(date +%Y%m%d-%H%M%S)"
cp -a "$CONFIG_FILE" "$BACKUP"

cat >>"$CONFIG_FILE" <<EOF

$BEGIN

# Hardware PWM used by ME130:
# GPIO18 -> PWM0, GPIO19 -> PWM1
dtoverlay=pwm-2chan,pin=18,func=2,pin2=19,func2=2
$END
EOF

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
