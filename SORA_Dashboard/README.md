# Firmware <-> dashboard compatibility kit

Checks that `dash-v2.py` and the flight-controller firmware (`v2/` sources) agree, by running both for real.

    python3 compat_check.py --fw-dir <CubeIDE project root> --dash <path>/dash-v2.py

`--fw-dir` can be the project root: the firmware files are found anywhere below it (Core/, Application/...),
ignoring Debug/, Release/, Drivers/ and Middlewares/.

Needs python3 + numpy and gcc on Linux/macOS. No Qt, pyserial or hardware required.

* **Static** (14 checks): enum order, FC field count, units, SENSOR_TO_BODY, command names, ERR reasons,
  parameter names/limits, command line length, compared between the firmware sources and the dashboard.
* **Dynamic** (47 checks): `fw_sim.c` compiles the real main loop + usb_command + usb_telemetry + config_store + mekf +
  motor_mixer + imu_calibration for the host, with a simulated IMU and a simulated user pressing the face button. The
  real dashboard classes talk to it through a fake serial port (`fake_qt.py` is a headless stand-in for Qt/pyserial):
  first-boot calibration, bias edit + send + save, calibration-value edit, range/unknown/busy errors, reboot +
  persistence, CAL BIAS, CAL FULL, plus an old DATA-only firmware.

Run it again after changing either side (new FC field, new parameter, new command, new enum value).
`--static-only` skips the process-level scenario.
