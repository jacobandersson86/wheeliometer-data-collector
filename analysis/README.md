# Wheeliometer Data Analysis Tools

This directory contains tools for analyzing IMU data collected by the Wheeliometer data collector.

## Data Format

The data collector creates binary files containing timestamped IMU samples:

- **File format**: Binary
- **Sample rate**: 1000 Hz (configurable)
- **Sensors**: 6-axis IMU (3-axis accelerometer + 3-axis gyroscope)
- **Filename pattern**: `imu_YYYY_MM_DD_HH_MM_SS.bin`

### File Structure

Each binary file contains:

#### File Header (24 bytes, packed)
- `magic` (4 bytes, uint32_t): 0x31554D49 ('IMU1' in little endian)
- `version` (4 bytes, uint32_t): Format version (currently 1)
- `start_time_us` (8 bytes, uint64_t): Recording start timestamp in microseconds
- `sample_rate` (4 bytes, uint32_t): Sample rate in Hz (typically 1000)
- `accel_fsr` (2 bytes, uint16_t): Accelerometer full-scale range (e.g., 8 for ±8G)
- `gyro_fsr` (2 bytes, uint16_t): Gyroscope full-scale range (e.g., 2000 for ±2000 DPS)

#### Batch Records (repeated until EOF)
Each batch record consists of:
- `base_timestamp_us` (8 bytes, uint64_t): Base timestamp in microseconds
- `sample_count` (2 bytes, uint16_t): Number of samples in this batch
- `samples` (14 bytes per sample × sample_count):
  - `accel_x` (2 bytes, int16_t): Raw accelerometer X-axis
  - `accel_y` (2 bytes, int16_t): Raw accelerometer Y-axis
  - `accel_z` (2 bytes, int16_t): Raw accelerometer Z-axis
  - `gyro_x` (2 bytes, int16_t): Raw gyroscope X-axis
  - `gyro_y` (2 bytes, int16_t): Raw gyroscope Y-axis
  - `gyro_z` (2 bytes, int16_t): Raw gyroscope Z-axis
  - `temperature` (2 bytes, int16_t): Raw temperature sensor value

## Tools

### Available
- Binary file parser (`analysis.imu_binary_parser`)
- Shared orientation fusion library (`analysis.orientation_fusion`)

### Coming Soon
- Data visualization
- Signal processing utilities
- Feature extraction

## Requirements

TBD

## Usage

Import from other Python scripts:

```python
from analysis import parse_imu_file, iter_imu_samples, read_imu_header

header = read_imu_header("imu_2026_02_19_10_00_00.bin")
print(header.sample_rate)

parsed = parse_imu_file("imu_2026_02_19_10_00_00.bin")
print(len(parsed.samples))

for sample in iter_imu_samples("imu_2026_02_19_10_00_00.bin"):
  print(sample.timestamp_us, sample.accel_x, sample.gyro_z)
  break
```

Available API:
- `read_imu_header(path)` → validates file and returns `FileHeader`
- `iter_imu_samples(path)` → memory-efficient generator of `ImuSample`
- `parse_imu_file(path)` → parses whole file and returns `ParsedImuFile`

Quick summary helper:

```bash
python analysis/summarize_imu_bin.py imu_2026_02_19_10_00_00.bin
```

The script prints:
- sample count
- duration
- measured sample rate
- min/max ranges for accel/gyro/temperature

Plot helper:

```bash
python analysis/plot_imu_bin.py imu_2026_02_19_10_00_00.bin
```

This shows 3 stacked plots:
- Acceleration X/Y/Z in `g` (y-limited to ±`accel_fsr` from file header)
- Gyro X/Y/Z in `dps` (y-limited to ±`gyro_fsr` from file header)
- Temperature in `°C` using MPU6886 conversion (`temp_c = raw / 326.8 + 25`), y-limited to `-40°C` to `+85°C`

3D replay helper:

```bash
python analysis/replay_imu_3d.py imu_2026_02_19_10_00_00.bin
```

Pipeline in this script:
1. Fuse gyro + accelerometer to quaternion (complementary/Mahony-style correction)
2. Convert quaternion to rotation matrix
3. Rotate a 3D cube model with that matrix
4. Render real-time replay from the bin file

Drift handling in fusion:
- Startup auto-gyro-bias: ignores first `500 ms`, then estimates bias over next `2 s`
- Startup sanity checks verify stillness (gyro + accel norm close to 1g)
- Clear warnings are printed if startup window is not still enough
- Runtime stillness detector adapts gyro bias slowly (ZARU-style)
- Accelerometer correction is gated to samples near `1g` to reduce motion-induced errors

Useful options:
- `--speed 1.0` replay speed multiplier
- `--render-fps 60` render frame rate
- `--cube-size 1.0` cube side length
- `--model path/to/model.stl` explicit STL model path (optional)
- `--model-size-mm 48,24,13.5` force STL to real-world size (mm), scaled uniformly into meters
- `--stl-unit mm|m` interpret STL units when `--model-size-mm` is not provided (default: `mm`)
- `--kp 1.8` accelerometer correction gain
- `--sensor-to-body r11,r12,r13,r21,r22,r23,r31,r32,r33` sensor→body mapping matrix (row-major CSV)
- `--translation-scale 1.0` scale factor for estimated translation from integrated linear acceleration
- `--rotation-only` render orientation without applying translation (translation is still estimated)

Translation notes:
- Position starts at `(0, 0, 0)` and velocity starts at `(0, 0, 0)`.
- During detected stillness, velocity is forced back to zero (zero-velocity update).
- Translation is dead-reckoned from IMU acceleration, so it can drift during long or aggressive motion.

STL sizing notes:
- By default, STL coordinates are interpreted as millimeters (`--stl-unit mm`) and converted to meters.
- Use `--model-size-mm` to enforce physical size and make model scale consistent with IMU translation units (meters).
- Use `--stl-unit m` if your STL coordinates are already in meters.
- On startup, scripts print raw STL bounds and fitted bounds so you can verify sizing.
- For your device dimensions: `--model-size-mm 48,24,13.5`

Default sensor→body mapping in script is:

`-1,0,0,0,-1,0,0,0,1`

This fixes roll/pitch sign for the current M5Stick setup (roll-left displays as roll-left, pitch-up as pitch-up).
If you want no remapping, set identity explicitly:

`--sensor-to-body 1,0,0,0,1,0,0,0,1`

Replay controls:
- `Space` → play/pause
- `←` / `→` → step one frame backward/forward
- Timeline slider → drag to jump to any point

### Sensor-to-body mapping calibration

Use this when model movement axes/signs do not match your physical motion.

1. Define your **body frame** (recommended):
  - +X: forward along the stick
  - +Y: left side of stick
  - +Z: out of display
2. Record 3 short motions (one axis at a time):
  - pure roll (about body +X)
  - pure pitch (about body +Y)
  - pure yaw (about body +Z)
3. For each motion, identify which sensor gyro axis dominates (`gyro_x`, `gyro_y`, or `gyro_z`) and whether the sign is positive or negative for positive body rotation.
4. Build a 3x3 mapping matrix where each row gives one body axis as sensor-axis combination:
  - row 1 = body X from sensor axes
  - row 2 = body Y from sensor axes
  - row 3 = body Z from sensor axes

Example: if body X = +sensor Y, body Y = -sensor X, body Z = +sensor Z:

`--sensor-to-body 0,1,0,-1,0,0,0,0,1`

This matrix is applied to both accelerometer and gyroscope before fusion.

By default, the script automatically loads the first `.stl` file found in `analysis/3d_models/proprietary`.
If no STL is found there, it falls back to a cube.

This 3D replay script uses `pyqtgraph` OpenGL rendering (instead of Matplotlib) for better STL performance.
Required packages:
- `pyqtgraph`
- `PyQt6`
- `numpy`
- `PyOpenGL`

IMU + video comparison helper:

```bash
python analysis/compare_imu_video.py data/imu_2026_02_19_11_19_31.bin path/to/video.mp4
```

This opens a side-by-side view with:
- left: 3D replay
- right: video replay
- one shared timeline + play/pause controls for both

Video sync options:
- `--video-offset-s <seconds>` sets initial video offset versus IMU timeline
  - positive offset: video is delayed relative to IMU timeline
  - negative offset: video is advanced relative to IMU timeline
- `--translation-scale <factor>` scales IMU-estimated translation in the 3D view
- `--rotation-only` renders only rotation in 3D view while still computing translation internally
- `--model-size-mm 48,24,13.5` enforces real-world STL size (mm) with uniform scaling
- `--stl-unit mm|m` sets default STL coordinate unit (default is `mm`)

Interactive sync controls (compare tool):
- `,` / `.` nudges video offset by `-20 ms / +20 ms`
- `<` / `>` nudges video offset by `-100 ms / +100 ms`
- Current offset is shown in the bottom control bar

Performance tip:
- If playback is slow, force lightweight 3D with `--use-cube`
- Example: `python analysis/compare_imu_video.py data/imu_2026_02_19_11_19_31.bin path/to/video.mp4 --use-cube`

Additional package needed:
- `opencv-python`

Refactor note:
- `replay_imu_3d.py` and `compare_imu_video.py` both use `analysis.orientation_fusion`
- Quaternion math, stillness detection, startup bias, gyro+accel fusion, and rotation matrix conversion are shared in one place
- Tuning these in `orientation_fusion.py` affects both tools consistently
