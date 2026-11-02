# V 7.0 - fix 3

`esp32_tune_V_7_0_3/esp32_tune_V_7_0_3.ino` replaces `esp32_tune_V_7_0_2`, which did not compile in the Arduino IDE ("'Port' was not declared in this scope").
Cause: the IDE inserts function prototypes above the first function, and the new PWM helper functions sat above `struct Port`. The struct now comes first. Nothing else changed: the dashboard stays `run_dashboard_V_7_0_2.bat`.
Checked on a PC by compiling the sketch with IDE-style prototypes inserted; not compiled with the real ESP32 core.
