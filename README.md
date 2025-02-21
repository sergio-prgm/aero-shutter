# Aero shutters

## Description

The development shall emulate the control of the operation of window shutters
in a home, considering, initially, 2 shutters. The operation should be opening, closing and
stopping, being able to stop the movement in the middle of the whole course. The system
will also have a light sensor that, depending on its value, will change the status of the
shutters. The system will be developed using FreeRTOS, and will have to consider, at least,
the following specifications.

## Operative modes

At any time, the system may receive an instruction from the outer world indicateing a change in the mode.

### Manual mode

Telling which shutter is affected, and the opening status (in percentage of the opening course). The automatic mode is disabled.

### Automatic mode

1. If the light sensor provides an intensity between 35% and 90% of its maximum value, the shutters will open completely.

```plaintext
Light: 35 < x < 90
Shutter position: 100
```

2. If the light sensor provides an intensity higher than 90%, the shutters will be open only at its 50%.

```plaintext
Light: 90 < x < 100
Shutter position: 50
```

3. If the light sensor provides an intensity lower than 35%, the shutters will be closed completely.

```plaintext
Light: 0 < x < 35
Shutter position: 0
```

4. While the light sensor keeps its values between one of the 3 ranges described above, the shutters won’t be changed, unless the change has been stable during the **last 5 seconds** (as to avoid oscillations in light values close to the thresholds).

### Holidays mode

Where a certain cyclic sequence of opening and closing of the shutters will be defined (individually and different from one shutter than the other). Only 4 different levels of opening will be considered. The sequence shall be randomly chosen, each time this mode is started. This mode will also disable the automatic mode.

> Note: In order to emulate this mode, the system will change the sequence step after 2 or 3 seconds.

## General assumptions

1. The solution shall be well structured
2. The development can be done using, or not, RTOS.
3. The use of low-power consumption modes will be welcome.

## Implementation details/notes/ideas

- Board: STM32 B-L4S5I-IOT01A
- OS: CMSIS (FreeRTOS interface layer for Cortex-based hardware)
- Peripherals:
  - **Stepper Motor(s)**: 5 VDC with ULN2003 driver. Connected to GPIO out pins. Need external power supply.
  - **Photosensitive Sensor**: with three cables. Connected to ADC1_IN5 pin.
  - *0.96in OLED*: for demonstration purposes.

---
**WIP**

### Tasks

- [x] Read light sensor
- Read serial communication (for ["outer world" instructions](#operative-modes))
- [x] Run motor
- Manage vacation mode

### Queues

- User comands (serial messages)
- Shutter actions

### Mutexes

- Light sensor reading/variable (`0-100`)
- Op Mode variable (`MANUAL_MODE`, `AUTO_MODE`, `VACATION_MODE`)
- Opening percentage (`0-100`)
