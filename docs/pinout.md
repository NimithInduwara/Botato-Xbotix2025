# Pinout

## Main Controller — Arduino Mega 2560 (competition sketches)

### Motor Driver

| Motor | Signal | Pin | Notes        |
|-------|--------|-----|--------------|
| Left  | IN1    | 6   | Direction    |
| Left  | IN2    | 7   | Direction    |
| Left  | EN     | 11  | PWM (speed)  |
| Right | IN1    | 5   | Direction    |
| Right | IN2    | 4   | Direction    |
| Right | EN     | 12  | PWM (speed)  |

### Wheel Encoders

| Encoder | Channel | Pin | Notes                |
|---------|---------|-----|----------------------|
| Left    | A       | 19  | Interrupt-capable    |
| Left    | B       | 18  | Interrupt-capable    |
| Right   | A       | 3   | Interrupt-capable    |
| Right   | B       | 2   | Interrupt-capable    |

### Ultrasonic Sensors

| Sensor      | Signal  | Pin | Notes                      |
|-------------|---------|-----|----------------------------|
| All (L/F/R) | TRIG    | 48  | Shared trigger line        |
| Left        | ECHO    | 42  |                            |
| Front       | ECHO    | 44  |                            |
| Right       | ECHO    | 46  |                            |

### Side IR Sensors

| Sensor | Pin |
|--------|-----|
| Left   | 52  |
| Right  | 50  |

### DIP Switches

| Switch | Pin |
|--------|-----|
| 1      | 22  |
| 2      | 41  |
| 3      | 43  |
| 4      | 45  |
| 5      | 47  |
| 6      | 49  |
| 7      | 51  |
| 8      | 53  |

---
