# Temperature Alarm Monitor (PlatformIO + ESP-IDF + Wokwi)

## Layout
```
.
├── platformio.ini      # PlatformIO env: esp32dev, framework = espidf
├── CMakeLists.txt       # top-level ESP-IDF project file
├── diagram.json         # Wokwi wiring: potentiometer -> GPIO34, LED -> GPIO2
├── wokwi.toml           # tells the Wokwi extension where the built firmware is
├── src/
│   ├── CMakeLists.txt    # registers main.c as an ESP-IDF component
│   └── main.c            # temperature alarm monitor (tasks, queue, mutex)
├── include/              # (empty) project headers go here if you add any
├── lib/                  # (empty) private libraries go here if you add any
└── test/                 # (empty) PlatformIO unit tests go here if you add any
```

## Build
```
pio run
```

## Simulate in Wokwi (VS Code extension)
1. Install the "Wokwi Simulator" and "PlatformIO IDE" VS Code extensions.
2. Build once with `pio run` so `.pio/build/esp32dev/firmware.bin` exists.
3. Open `diagram.json` and press the green play button, or run
   "Wokwi: Start Simulator" from the command palette.
4. Turn the potentiometer knob to raise/lower the simulated ADC voltage —
   this stands in for the temperature sensor. Above the threshold set in
   `main.c` (`TEMP_THRESHOLD_C`), the LED on GPIO2 turns on.

## Notes
- The potentiometer wiper feeds GPIO34 (ADC1_CHANNEL_6) in place of a real
  analog temperature sensor (e.g. LM35) — swap the part in `diagram.json`
  for a real sensor part if Wokwi adds one, no code changes needed as long
  as it outputs an analog voltage on the same pin.
- Threshold, hysteresis, and sample interval are `#define`s at the top of
  `main.c`.
