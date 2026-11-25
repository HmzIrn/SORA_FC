#!/bin/bash
# usage: bash build_fw_sim.sh <firmware folder or CubeIDE project root> [output binary]
# Builds fw_sim: the real firmware main loop + modules compiled for the host (see fw_sim.c).
# Source files are located anywhere below <folder> (Core/Src, Application/..., etc.).
FW="${1:?firmware folder / project root}"; OUT="${2:-fw_sim}"
HERE="$(cd "$(dirname "$0")" && pwd)"
case "$OUT" in /*) ;; *) OUT="$PWD/$OUT" ;; esac
GEN="$(dirname "$OUT")"

find_file() {   # first match, ignoring build output and ST vendor folders (your own Drivers/* folders ARE searched)
  local hits n
  hits="$(find "$FW" \( -name Debug -o -name Release -o -name .git -o -name STM32F4xx_HAL_Driver -o -name CMSIS -o -name Middlewares \) -prune -o -type f -name "$1" -print)"
  n="$(printf '%s\n' "$hits" | grep -c .)"
  if [ "$n" -gt 1 ]; then echo "WARNING: $1 exists $n times, using the first:" $hits >&2; fi
  printf '%s\n' "$hits" | head -1
}
SRCS=""; INCS=""
for f in main.c config_store.c usb_command.c usb_telemetry.c mekf.c motor_mixer.c imu_calibration.c; do
  p="$(find_file "$f")"
  if [ -z "$p" ]; then echo "build_fw_sim: cannot find $f under $FW" >&2; exit 2; fi
  SRCS="$SRCS $p"; INCS="$INCS -I$(dirname "$p")"
done
for h in config_store.h usb_command.h usb_telemetry.h mekf.h motor_mixer.h imu_calibration.h; do
  p="$(find_file "$h")"
  if [ -z "$p" ]; then echo "build_fw_sim: cannot find $h under $FW" >&2; exit 2; fi
  INCS="$INCS -I$(dirname "$p")"
done
MAIN_C="$(find_file main.c)"; SRCS="${SRCS/$MAIN_C/}"      # main.c is pulled in by fw_sim.c, do not compile twice

printf '#include <stdint.h>\nextern uint8_t sim_flash[];\n' > "$GEN/simflash.h"
# stubs first: they must shadow the real HAL / mpu6050 headers of the CubeIDE project
gcc -std=gnu99 -O1 -w -I"$HERE/stubs" $INCS -I"$GEN" -I"$HERE" -include "$GEN/simflash.h" \
  -DCONFIG_STORE_SKIP_IMAGE_CHECK -DCONFIG_STORE_SECTOR_ADDR=0x08060000UL -DCONFIG_STORE_SECTOR_NUMBER=7 -DCONFIG_STORE_SECTOR_SIZE=2048 \
  '-DCONFIG_STORE_FLASH_PTR(a)=(sim_flash+((a)-0x08060000UL))' \
  "$HERE/fw_sim.c" $SRCS -lm -o "$OUT"