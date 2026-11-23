#!/usr/bin/env bash
set -euo pipefail

# Cartographer requires scan timestamps to advance continuously.  On an offline
# Raspberry Pi, ROS may start before NTP has synchronized and the wall clock can
# then jump by several minutes during an active trajectory.  Wait briefly for an
# initial synchronization and freeze NTP while the navigation service is alive.

UAV_SYNC_WAIT_SECONDS="${UAV_TIME_SYNC_WAIT_SEC:-20}"
if ! [[ "${UAV_SYNC_WAIT_SECONDS}" =~ ^[0-9]+$ ]]; then
  echo "Invalid UAV_TIME_SYNC_WAIT_SEC=${UAV_SYNC_WAIT_SECONDS}; expected a non-negative integer." >&2
  exit 2
fi

UAV_NTP_ENABLED="$(timedatectl show -p NTP --value 2>/dev/null || true)"
UAV_CLOCK_SYNCHRONIZED="$(timedatectl show -p NTPSynchronized --value 2>/dev/null || true)"

if [[ "${UAV_NTP_ENABLED}" == "yes" && "${UAV_CLOCK_SYNCHRONIZED}" != "yes" ]]; then
  for ((UAV_WAITED_SECONDS = 0;
        UAV_WAITED_SECONDS < UAV_SYNC_WAIT_SECONDS;
        UAV_WAITED_SECONDS++)); do
    UAV_CLOCK_SYNCHRONIZED="$(
      timedatectl show -p NTPSynchronized --value 2>/dev/null || true
    )"
    if [[ "${UAV_CLOCK_SYNCHRONIZED}" == "yes" ]]; then
      break
    fi
    sleep 1
  done
fi

if [[ "${UAV_CLOCK_SYNCHRONIZED}" == "yes" ]]; then
  echo "UAV clock guard: initial NTP synchronization is complete."
else
  echo "UAV clock guard: NTP was not synchronized within ${UAV_SYNC_WAIT_SECONDS}s; " \
       "continuing with the current clock to keep boot usable offline." >&2
fi

# The service unit runs this script with systemd's '+' prefix, so it has the
# privilege required by timedatectl even though the main ROS process runs as ljz.
timedatectl set-ntp false

if systemctl is-active --quiet systemd-timesyncd.service; then
  echo "UAV clock guard: failed to stop systemd-timesyncd." >&2
  exit 1
fi
