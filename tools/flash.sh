#!/bin/bash
# Flashes build/bead_init.bin from WSL: asks the running app to jump to the
# Daisy bootloader ('b' over USB serial), attaches the bootloader to WSL with
# usbipd-win, runs `make program-dfu` and waits for the app to come back.
#
# USBIPD, USBIPD_BUSID, WSL_DISTRO and COM_PORT come from the Makefile.
#
# The attach always names WSL_DISTRO: without it usbipd uses the default
# distribution, and when that one shuts down again its systemd-binfmt clears
# the binfmt entries all WSL distributions share, which breaks running .exe
# files (this script included) until WSLInterop is registered again.

cd "$(dirname "$0")/.." || exit 1
tools=tools

bootloader_attached() {
  dfu-util -l 2>/dev/null | grep -q "@Flash /0x90000000"
}

if ! bootloader_attached; then
  "$tools/serial.sh" send b >/dev/null 2>&1 || \
      echo "Could not reach the app on $COM_PORT. Put the module in the" \
           "bootloader by hand: press RESET, then BOOT while the LED pulses."
  echo "Waiting for the Daisy bootloader on busid $USBIPD_BUSID..."
  end=$((SECONDS + 120))
  until bootloader_attached; do
    if [ $SECONDS -ge $end ]; then
      echo "No bootloader after 2 minutes. Is it bound in usbipd" \
           "(usbipd bind --busid $USBIPD_BUSID, as administrator)?"
      exit 1
    fi
    "$USBIPD" attach --wsl "$WSL_DISTRO" --busid "$USBIPD_BUSID" \
        >/dev/null 2>&1
  done
fi

make --no-print-directory program-dfu 2>&1 | tr '\r' '\n' | \
    grep -E "Downloading element|File downloaded|rror|Cannot|LIBUSB"
if [ "${PIPESTATUS[0]}" -ne 0 ]; then
  exit 1
fi

echo "Waiting for the app..."
end=$((SECONDS + 30))
until "$USBIPD" list 2>/dev/null | tr -d '\r' | grep -q "0483:5740"; do
  if [ $SECONDS -ge $end ]; then
    echo "The app's serial port did not appear."
    exit 1
  fi
  sleep 1
done
echo "App running."
