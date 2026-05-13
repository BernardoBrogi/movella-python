# Arm Motion Tracking System with Movella Xsens IMU Sensors

This repository contains a complete pipeline for tracking the motion of an arm using Movella Xsens IMU sensors. The system is designed to process motion data, perform calibration using Principal Component Analysis (PCA) to extract a one-dimensional control signal.

## Folder Structure

This folder contains a Python wrapper for communicating with Movella Xsens IMU sensors via Bluetooth. It collects IMU data, supports calibration and kernel computation, and can transmit motion data through a UDP socket.

**Main Features:**
- Establishes Bluetooth connection with Movella Xsens sensors.
- Streams IMU data in real-time.
- Calibrates motion signals with PCA.
- Computes normalized kernel values for downstream control.
- Sends motion data through UDP.
  
**Key Script:**

**`movella_streamer_class.py`**
   - Provides the `MovellaStreamer` class used for initialization, streaming, calibration, and kernel generation.
   - Is the preferred entry point for Python scripts and Jupyter notebooks.

### MovellaStreamer Python Class

The `MovellaStreamer` class wraps the Movella Xsens DOT SDK and provides a simple pipeline for data collection and processing.

#### Constructor

```python
MovellaStreamer(config_path, setup_name, udp_ip=None, udp_port=None, frequency=60, n_trackers=5)
```

Parameters:
- `config_path`: Path to `config.json` with the tracker MAC addresses.
- `setup_name`: Key inside the configuration file, for example `UNISI`.
- `udp_ip`, `udp_port`: Optional UDP destination for streaming.
- `frequency`: Sampling frequency in Hz.
- `n_trackers`: Number of trackers to use. Supported values in the class are `1`, `2`, `4`, and `5`.

#### Main Methods

`initialize()`
- Scans for Movella DOT devices, connects to them, and starts measurement mode.

`get_latest_data()`
- Returns the most recent sample as a dictionary with timestamp, number of working trackers, and quaternion data.

`stream_loop()`
- Continuously prints the latest tracker data in the terminal.

`stream_udp_loop()`
- Continuously streams the latest data over UDP.

`calibrate(calibration_name="movellaValue1Phase")`
- Records repeated motions, computes PCA, and saves calibration parameters for one-phase control.

`calibrate_2arm(imu="2arm")`
- Records a start and end quaternion for two-arm trigger calibration.

`compute_kernel(send_ip="172.16.0.1", send_port=8052)`
- Computes the normalized kernel value and sends it over UDP.

`compute_easy_kernel(send_ip="172.16.0.1", send_port=8052)`
- Simplified kernel computation for hand-only or hand-plus-second-arm setups.

`cleanup()`
- Stops measurement, disables logging, and resets device orientation.


### Prerequisites
- Install Python 3.8 (tested also with 3.9 and 3.10).
- Set up the Movella Xsens IMU sensors.
- Install the Movella DOT PC SDK from this [link](https://base.xsens.com/s/article/Movella-DOT-PC-SDK-Guide?language=en_US)
- Python dependencies: `requirements.txt`

### Installation

Install dependencies:

```bash
pip install -r requirements.txt
```

Then install the Movella SDK wheel (`.whl`) that matches your Python version and operating system.
The repository already includes two wheel files:

- `movelladot_pc_sdk-2023.6.0-cp310-none-win_amd64.whl` -> for **Windows + Python 3.10**
- `movelladot_pc_sdk-2023.6.0-cp38-none-linux_x86_64.whl` -> for **Linux + Python 3.8**

Examples:

```bash
# Windows + Python 3.10
pip install .\movelladot_pc_sdk-2023.6.0-cp310-none-win_amd64.whl
```

```bash
# Linux + Python 3.8
pip install ./movelladot_pc_sdk-2023.6.0-cp38-none-linux_x86_64.whl
```

If `pip` points to a different Python version, run pip through the target interpreter (for example `python -m pip install ...`).

### Python Notebook Workflow

The Python side is notebook-friendly, especially for calibration and live signal inspection. A typical notebook flow is:

1. Import the class and create a tracker instance.
2. Run `initialize()` once to connect the sensors.
3. Use `get_latest_data()` or `stream_loop()` to inspect live quaternions.
4. Run `calibrate()` or `calibrate_2arm()` to generate `.mat` calibration files.
5. Run `compute_kernel()` or `compute_easy_kernel()` to stream normalized control values.
6. Call `cleanup()` at the end of the notebook to stop measurements safely.

Example notebook cells:

```python
from movella_streamer_class import MovellaStreamer

tracker = MovellaStreamer("config.json", "UNISI", udp_ip="127.0.0.1", udp_port=8051, n_trackers=4)
tracker.initialize()
```

```python
latest = tracker.get_latest_data()
latest
```

```python
tracker.stream_udp_loop()
```

```python
tracker.cleanup()
```

