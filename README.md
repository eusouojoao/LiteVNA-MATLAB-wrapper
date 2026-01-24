# LiteVNA (MATLAB) wrapper

MATLAB interface for the **LiteVNA 62/64** over serial. Configure sweeps, read S-parameters (S11/S21), and run basic calibration routines (SOL/TR/ER).

## Features
- Serial control (MATLAB `serialport`)
- Sweep configuration (start/step/points/averaging)
- Measurements: `rawMeasureS11`, `rawMeasureS21`, `rawMeasureS11S21`
- Calibration helper + apply cal: `performCalibration`, `calMeasureS11`, `calMeasureS21`, `calMeasureS11S21`
- Save/load calibration to `.mat`

## Requirements
- MATLAB R2019b+ (needs `serialport`)
- LiteVNA connected as a serial device (e.g. `COMx` on Windows, `/dev/ttyACMx` on Linux)
