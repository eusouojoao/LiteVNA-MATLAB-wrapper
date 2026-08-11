# LiteVNA (MATLAB) wrapper

MATLAB interface for the **LiteVNA 62/64** over a serial connection. 
Configure sweeps, read S-parameters (S11/S21), and run basic calibration routines (SOL/TR/ER).

LiteVNA info: https://www.zeenko.tech/litevna and https://groups.io/g/liteVNA/

## Features
- Serial control (MATLAB `serialport`)
- Sweep configuration (`start`/`step`/`points`/`averaging`)
- Raw Measurements: `rawMeasureS11`, `rawMeasureS21`, `rawMeasureS11S21`
- Calibration helper: `performCalibration`
- Save/load calibration to `.mat`
- Calibrated Measurements: `calMeasureS11`, `calMeasureS21`, `calMeasureS11S21`

## Requirements
- MATLAB R2019b+ (needs `serialport`)
- LiteVNA connected as a serial device (e.g. `COM<num>` on Windows, `/dev/ttyACM<num>` on Linux)

## Install

### MATLAB Add-On (recommended)
Cf. https://www.mathworks.com/help/matlab/add-ons.html

### Manual (from source)
1. Clone or download this repo.
2. Add it to your MATLAB path:
```matlab
addpath(genpath("path/to/repo"));
savepath; % optional
vna = litevna.LiteVNA("/dev/ttyACM<num>", 115200);
```
