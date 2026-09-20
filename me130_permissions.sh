#!/usr/bin/env bash
#
# ME130 hardware permissions.
#
# The lab nodes need /dev/gpiochip0, /sys/class/pwm and /dev/i2c-1. By default
# those are root-only, so the nodes die on startup with "Could not open ...".
#
# Running `ros2 launch` under sudo is the wrong fix: sudo does not carry the
# ROS environment, so the launch file and the workspace overlay are not found.
# Grant the login user access instead, which is what this script does.
#
#   sudo ./me130_permissions.sh
#   # then LOG OUT and back in for the new groups to apply
#
set -Eeuo pipefail

log(){ printf '\n[ME130] %s\n' "$*"; }
die(){ printf '[FAIL] %s\n' "$*" >&2; exit 1; }

[[ $EUID -eq 0 ]] || die "Run with sudo: sudo ./me130_permissions.sh"

TARGET_USER="${SUDO_USER:-}"
if [[ -z "$TARGET_USER" || "$TARGET_USER" == "root" ]]; then
    TARGET_USER="$(awk -F: '$3>=1000 && $3<65534 {print $1; exit}' /etc/passwd)"
fi
[[ -n "$TARGET_USER" ]] || die "Could not determine which user to grant access to."
log "Granting hardware access to: $TARGET_USER"

groupadd -f gpio
groupadd -f i2c

# dialout owns /dev/gpiochip* on this image; gpio/i2c are created above and
# used by the udev rules below.
usermod -aG gpio,i2c,dialout "$TARGET_USER"

log "Installing udev rules..."
cat >/etc/udev/rules.d/99-me130.rules <<'RULES'
# ME130 lab hardware access.

# GPIO character device (libgpiod: encoder, DIR, PS).
KERNEL=="gpiochip[0-9]*", GROUP="gpio", MODE="0660"

# I2C (MPU6050).
KERNEL=="i2c-[0-9]*", GROUP="i2c", MODE="0660"

# Hardware PWM. sysfs creates pwmN/ subtrees on export, after this rule has
# already run, so re-apply the ownership across the whole tree on any pwm
# event rather than matching a single node.
SUBSYSTEM=="pwm*", ACTION=="add", PROGRAM="/bin/sh -c '\
    chgrp -R gpio /sys/class/pwm /sys/devices/platform/*/*.pwm/pwm/pwmchip* 2>/dev/null || true; \
    chmod -R g+rw /sys/class/pwm /sys/devices/platform/*/*.pwm/pwm/pwmchip* 2>/dev/null || true'"
RULES

# A udev rule alone cannot win this race: writing pwmchipN/export CREATES the
# pwmN/ subtree, and the node writes pwmN/period microseconds later -- before
# udev has processed the new device. So export the channels at boot and fix
# the permissions once, deterministically.
log "Installing the PWM export service..."
cat >/usr/local/sbin/me130-pwm-setup <<'PWMEOF'
#!/bin/bash
# Export the ME130 hardware PWM channels and hand them to the gpio group.
# Run at boot by me130-pwm.service, so the lab nodes find them ready.
CHIP=/sys/class/pwm/pwmchip0
[[ -d "$CHIP" ]] || { echo "no $CHIP (is dtoverlay=pwm-2chan set?)" >&2; exit 0; }

for ch in 0 1; do
    [[ -d "$CHIP/pwm$ch" ]] || echo "$ch" > "$CHIP/export" 2>/dev/null || true
done
# Let sysfs finish creating the attribute files before touching them.
sleep 0.5

fix() {
    [[ -e "$1" ]] || return 0
    chgrp -R gpio "$1" 2>/dev/null || true
    chmod -R g+rw "$1" 2>/dev/null || true
}
fix /sys/class/pwm
for d in /sys/devices/platform/*/*.pwm/pwm/pwmchip*; do fix "$d"; done
PWMEOF
chmod 0755 /usr/local/sbin/me130-pwm-setup

cat >/etc/systemd/system/me130-pwm.service <<'SVCEOF'
[Unit]
Description=ME130: export hardware PWM channels and grant gpio group access
After=sysinit.target local-fs.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/local/sbin/me130-pwm-setup

[Install]
WantedBy=multi-user.target
SVCEOF

systemctl daemon-reload
systemctl enable me130-pwm.service
systemctl start me130-pwm.service || warn "me130-pwm.service did not start cleanly."

udevadm control --reload-rules
udevadm trigger --subsystem-match=gpio --subsystem-match=pwm --subsystem-match=i2c-dev || true

# The pwmchip already exists from boot, so the rule above has not fired for it.
# Fix it now so a reboot is not required.
if [[ -d /sys/class/pwm ]]; then
    chgrp -R gpio /sys/class/pwm 2>/dev/null || true
    chmod -R g+rw /sys/class/pwm 2>/dev/null || true
    for chip in /sys/devices/platform/*/*.pwm/pwm/pwmchip*; do
        [[ -e "$chip" ]] || continue
        chgrp -R gpio "$chip" 2>/dev/null || true
        chmod -R g+rw "$chip" 2>/dev/null || true
    done
fi
chgrp gpio /dev/gpiochip* 2>/dev/null || true
chmod g+rw /dev/gpiochip* 2>/dev/null || true

# Catch any pwmN/ subtree that already exists from an earlier run.
for d in /sys/class/pwm/pwmchip*/pwm[0-9]*; do
    [[ -e "$d" ]] || continue
    chgrp -R gpio "$d" 2>/dev/null || true
    chmod -R g+rw "$d" 2>/dev/null || true
done

cat <<EOS

Done. $TARGET_USER is now in: gpio, i2c, dialout

>>> LOG OUT AND BACK IN (or reboot) for the group change to take effect. <<<

The PWM channels are exported at boot by me130-pwm.service, so pwm0/ exists
and is group-writable before any node starts. Check it with:
    systemctl status me130-pwm.service
    ls -l /sys/class/pwm/pwmchip0/pwm0/

Verify afterwards, with no sudo:
    id -nG | tr ' ' '\n' | grep -E 'gpio|i2c|dialout'
    gpiodetect
    ls -l /dev/gpiochip0 /sys/class/pwm/pwmchip0/export
    ros2 launch me130_pendulum motor_characterization.launch.py

EOS
