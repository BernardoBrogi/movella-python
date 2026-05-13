import socket
import struct
import time
import numpy as np
import json
from xdpchandler import *
from sys import version_info as python_version_info
from sklearn.decomposition import PCA
from scipy.io import savemat, loadmat
from scipy.signal import find_peaks
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import getRelativeRotation_movella 
from pynput.keyboard import Listener, Key
import keyboard  # New import for keyboard handling

class MovellaStreamer:
    def __init__(self, config_path, setup_name, udp_ip=None, udp_port=None, frequency=60, n_trackers=5):
        # Load configuration
        with open(config_path, "r") as file:
            self.mac_config = json.load(file)
        self.setup_name = setup_name
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.frequency = frequency
        self.sample_rate = 1 / frequency
        self.sock = None
        self.xdpcHandler = None
        self.ID_tracker = []
        self.idx_sorted_tracker = []
        self.working_tracker = []
        self.orientationResetDone = False
        self.first_flag = True
        self.first_data = np.empty(0)
        self.start_time = None
        self.lastSend = 0
        self.n_trackers = n_trackers  # Number of trackers to use (4 or 5)
        self.end = False

        # Import correct SDK version
        if python_version_info.minor == 8:
            import movelladot_pc_sdk.movelladot_pc_sdk_py38_64 as movelladot_pc_sdk
        elif python_version_info.minor == 9:
            import movelladot_pc_sdk.movelladot_pc_sdk_py39_64 as movelladot_pc_sdk
        elif python_version_info.minor == 10:
            import movelladot_pc_sdk.movelladot_pc_sdk_py310_64 as movelladot_pc_sdk
        else:
            print("Unsupported Python version. Please use Python 3.8, 3.9, or 3.10.")
            exit(-1)
        self.movelladot_pc_sdk = movelladot_pc_sdk

        # Setup keyboard listener
        self.listener = Listener(on_press=self._on_press)
        self.listener.start()
    
    def _on_press(self, key):
        # This function runs on the background and checks if a keyboard key was pressed
        if key == Key.esc:
            self.end = True

    def _init_realtime_plot(self, n_signals):
        """Initialize a realtime plot for one or two streamed values."""
        plt.ion()
        self._plot_histories = [[] for _ in range(n_signals)]

        if n_signals == 2:
            self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
            self.line1, = self.ax1.plot([], [], 'r-')
            self.line2, = self.ax2.plot([], [], 'b-')
            self.ax1.set_ylabel('Control Value')
            self.ax2.set_ylabel('Trigger Value')
            self.ax2.set_xlabel('Sample')
            self.ax1.set_ylim(0, 1)
            self.ax2.set_ylim(0, 1)
        else:
            self.fig, self.ax1 = plt.subplots(figsize=(10, 4))
            self.line1, = self.ax1.plot([], [], 'r-')
            self.ax1.set_ylabel('Control Value')
            self.ax1.set_xlabel('Sample')
            self.ax1.set_ylim(0, 1)

        self.ax1.grid(True)
        if n_signals == 2:
            self.ax2.grid(True)
        self.fig.tight_layout()
        plt.show(block=False)

    def _update_realtime_plot(self, values):
        """Update the realtime plot with the latest streamed values."""
        if not hasattr(self, '_plot_histories'):
            return

        if not isinstance(values, (list, tuple, np.ndarray)):
            values = [values]

        for history, value in zip(self._plot_histories, values):
            history.append(float(value))

        max_points = 400
        for history in self._plot_histories:
            if len(history) > max_points:
                del history[:-max_points]

        x_data = np.arange(len(self._plot_histories[0]))
        self.line1.set_data(x_data, self._plot_histories[0])
        self.ax1.relim()
        self.ax1.autoscale_view(scalex=True, scaley=False)
        self.ax1.set_ylim(0, 1)

        if len(self._plot_histories) == 2:
            self.line2.set_data(x_data, self._plot_histories[1])
            self.ax2.relim()
            self.ax2.autoscale_view(scalex=True, scaley=False)
            self.ax2.set_ylim(0, 1)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

    def _close_realtime_plot(self):
        """Close the realtime plot if it was created."""
        if hasattr(self, 'fig'):
            plt.close(self.fig)

    def initialize(self):
        self.xdpcHandler = XdpcHandler()
        if not self.xdpcHandler.initialize():
            self.xdpcHandler.cleanup()
            raise RuntimeError("Failed to initialize XdpcHandler.")

        self.xdpcHandler.scanForDots()
        if len(self.xdpcHandler.detectedDots()) == 0:
            self.xdpcHandler.cleanup()
            raise RuntimeError("No Movella DOT device(s) found.")

        self.xdpcHandler.connectDots()
        if len(self.xdpcHandler.connectedDots()) == 0:
            self.xdpcHandler.cleanup()
            raise RuntimeError("Could not connect to any Movella DOT device(s).")

        for device in self.xdpcHandler.connectedDots():
            filterProfiles = device.getAvailableFilterProfiles()
            print("Available filter profiles:")
            for f in filterProfiles:
                print(f.label())
            print(f"Current profile: {device.onboardFilterProfile().label()}")
            if device.setOnboardFilterProfile("General"):
                print("Successfully set profile to General")
            else:
                print("Setting filter profile failed!")
            print("Putting device into measurement mode.")
            if not device.startMeasurement(self.movelladot_pc_sdk.XsPayloadMode_CompleteQuaternion):
                print(f"Could not put device into measurement mode. Reason: {device.lastResultText()}")

        # Assign MAC addresses dynamically
        if self.n_trackers == 1:
            all_ids = [
                self.mac_config[self.setup_name]["hand"]
            ]
        elif self.n_trackers == 2:
            all_ids = [
                self.mac_config[self.setup_name]["arm"], # hand
                self.mac_config[self.setup_name]["forearm"] # arm2
            ]
        elif self.n_trackers == 4:
            all_ids = [
                self.mac_config[self.setup_name]["hand"],
                self.mac_config[self.setup_name]["forearm"],
                self.mac_config[self.setup_name]["arm"],
                self.mac_config[self.setup_name]["chest"]
            ]
        elif self.n_trackers == 5:
            all_ids = [
                self.mac_config[self.setup_name]["hand"],
                self.mac_config[self.setup_name]["forearm"],
                self.mac_config[self.setup_name]["arm"],
                self.mac_config[self.setup_name]["chest"],
                self.mac_config[self.setup_name]["arm2"]
            ]

        self.ID_tracker = all_ids[:self.n_trackers]
        self.ID_tracker = all_ids[:self.n_trackers]
        self.working_tracker = [device.bluetoothAddress() for device in self.xdpcHandler.connectedDots()]
        self.idx_sorted_tracker = [self.working_tracker.index(mac) for mac in self.ID_tracker]
        print("Order of trackers:", self.idx_sorted_tracker)

        # Setup UDP socket if needed
        if self.udp_ip and self.udp_port:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.start_time = time.time_ns()
        self.lastSend = 0
        self.orientationResetDone = False
        self.first_flag = True
        self.first_data = np.empty(0)

    def get_latest_data(self):
        """Retrieve the latest data from the trackers (does not send UDP)."""
        if self.xdpcHandler.packetsAvailable():
            data = np.empty(0)
            n_working_movella = 0

            # Reset heading orientation to match reference frames
            if not self.orientationResetDone:
                for device in self.xdpcHandler.connectedDots():
                    print(f"\nResetting heading for device {device.portInfo().bluetoothAddress()}: ", end="", flush=True)
                    if device.resetOrientation(self.movelladot_pc_sdk.XRM_Heading):
                        print("OK", end="", flush=True)
                    else:
                        print(f"NOK: {device.lastResultText()}", end="", flush=True)
                print("\n", end="", flush=True)
                self.orientationResetDone = True

            for j in range(self.n_trackers):
                packet = self.xdpcHandler.getNextPacket(self.working_tracker[self.idx_sorted_tracker[j]])
                if packet.containsOrientation():
                    quaternion = packet.orientationQuaternion()
                    data = np.concatenate((data, quaternion))
                    n_working_movella += 1
                else:
                    zeros = np.zeros(4)
                    data = np.concatenate((data, zeros))
                    print("Tracker " + self.working_tracker[self.idx_sorted_tracker[j]] + " is not working.")

            # On first valid full packet, store as baseline to remove inter-tracker offsets
            if self.first_flag and len(data) >= 4 * self.n_trackers:
                self.first_data = data.copy()
                self.first_flag = False

            # Apply baseline compensation if available
            if len(data) >= 4 * self.n_trackers and len(self.first_data) >= 4 * self.n_trackers:
                for i in range(1, self.n_trackers):
                    data[i*4:(i+1)*4] = data[i*4:(i+1)*4] - (self.first_data[i*4:(i+1)*4] - self.first_data[0:4])

            timestamp = (time.time_ns() - self.start_time) / 1e9
            return {
                "start_message": 1,
                "timestamp": timestamp,
                "n_working_movella": n_working_movella,
                "data": data
            }
        return None

    def stream_loop(self):
        """Continuously retrieve data (does not send UDP)."""
        self.end = False
        try:

            print("Starting streaming loop. Press ESC to stop.")
            while not self.end:
                latest = self.get_latest_data()
                if latest:
                    # Do something with latest, e.g., print or store
                    print(latest["data"])
                time.sleep(self.sample_rate)  # Reduce CPU usage
        except KeyboardInterrupt:
            print("Stopping data retrieval loop...")
            self.cleanup()
        finally:
            print("Data retrieval loop ended.")
            

    def stream_udp_loop(self):
        """Continuously stream data over UDP."""
        self.end = False
        if not self.sock:
            raise RuntimeError("UDP socket not initialized. Provide udp_ip and udp_port in constructor.")
        try:

            print("Starting UDP streaming loop. Press ESC to stop.")
            while not self.end:
                latest = self.get_latest_data()
                if latest:
                    data = latest["data"]
                    timestamp = latest["timestamp"]
                    n_working_movella = latest["n_working_movella"]
                    format = "=IfI" + str(len(data)) + "d"
                    packed_data = struct.pack(format, 1, timestamp, n_working_movella, *data)
                    if (timestamp - self.lastSend) > self.sample_rate:
                        self.sock.sendto(packed_data, (self.udp_ip, self.udp_port))
                        self.lastSend = timestamp
                
                time.sleep(self.sample_rate)  # Reduce CPU usage
        except KeyboardInterrupt:
            print("Stopping UDP streaming loop...")
            if self.sock:
                self.sock.close()
                print("UDP server closed.")
    
            self.cleanup()

        finally:
            if self.sock:
                self.sock.close()
                print("UDP server closed.")
    

    def calibrate(self, calibration_name="movellaValue1Phase", saved_data=True):
        """
        Calibrate the Movella system by recording data and computing PCA parameters
        
        Args:
            calibration_name (str): Name for the calibration file
        """

        def deduplicate_peaks(idxs, pks, mode="max"):
            if len(idxs) == 0:
                return [], []
            sorted_idx = np.argsort(idxs)
            idxs, pks = idxs[sorted_idx], pks[sorted_idx]
            kept = []
            kept_pks = []
            i = 0
            while i < len(idxs):
                close = np.where((idxs >= idxs[i]) & (idxs <= idxs[i] + 5))[0]
                if mode == "max":
                    best = close[np.argmax(pks[close])]
                else:  # mode == "min"
                    best = close[np.argmin(pks[close])]
                kept.append(idxs[best])
                kept_pks.append(pks[best])
                i = close[-1] + 1
            return np.array(kept), np.array(kept_pks)
        
        # Initialize data storage
        all_data = []
        timestamps = []
        all_data_save = []

        self.end = False

        print(f"Starting calibration")

        # Warm-up: discard a short sequence to let sensors and filters stabilise
        warmup_samples = max(5, int(self.frequency))
        print(f"Warming up: discarding {warmup_samples} samples...")
        for _ in range(warmup_samples):
            _ = self.get_latest_data()
            time.sleep(self.sample_rate)
        
        print("Perform the calibration movement now.")

        # Record data for the specified duration
        while not self.end:
            latest = self.get_latest_data()
            if latest and latest["n_working_movella"] == self.n_trackers:
                # Reshape data to (n_trackers, 4) and store
                quat_data_save = latest["data"]
                quat_data = quat_data_save.reshape(-1, 4)
                timestamps.append(latest["timestamp"])
                all_data.append(quat_data)
                all_data_save.append(quat_data_save)
            time.sleep(self.sample_rate)
        
        if not all_data:
            raise RuntimeError("No data collected during calibration")
        
        if saved_data:
            # Ask for filename
            filename_prefix = input("PHASE 1: Enter filename to save data (without extension): ").strip()
            if not filename_prefix:
                print("PHASE 1: No filename provided. Data not saved.")
            else:
                import datetime
                currentDateTime = datetime.datetime.now().strftime('%Y_%m_%d_%H%M%S')
                filename = f"data/{filename_prefix}_{currentDateTime}.mat"

                savemat(filename, {
                    'tracked_data': all_data_save,
                })

                print(f"PHASE 1: Data saved to {filename}")
        
        # Convert to numpy array
        all_data = np.array(all_data)  # Shape: (n_samples, n_trackers, 4)
        
        # If using 5 trackers, exclude the last tracker's data (last 4 columns)
        if self.n_trackers == 5:
            all_data = all_data[:, :-1, :]
        
        # Compute relative quaternions
        quat_relative = []
        for i in range(all_data.shape[0]):
            data_sample = all_data[i]  # Shape: (n_trackers, 4)
            # Use your external function to get relative rotations
            relative_rots = getRelativeRotation_movella.getRelativeRotation_movella(data_sample)
            # Convert to compact form (flattened array)
            compact_quats = np.concatenate([q.elements for q in relative_rots])
            quat_relative.append(compact_quats)
        
        quat_relative = np.array(quat_relative)
        
        # Perform PCA
        pca = PCA()
        score = pca.fit_transform(quat_relative)
        coeff = pca.components_.T
        explained = pca.explained_variance_ratio_ * 100
        mu = pca.mean_

        # Use first component as calibration signal
        calibration_signal = score[:, 0]

        # Plot the calibration signal
        # plt.figure()
        # plt.plot(calibration_signal)
        # plt.title("Calibration Signal (First Principal Component)")
        # plt.xlabel("Sample")
        # plt.ylabel("Calibration Signal Value")
        # plt.show()
        
        # Check if first component explains at least 80% of variance
        n_components = np.where(np.cumsum(explained) > 60)[0]
        # print(explained)
        if n_components[0] != 0:
            raise RuntimeError("The first PC does not describe the 80% of variance. Calibrate again.")
        
                
       # Map calibration signal between 0 and 1
        calibration_signal_01 = (calibration_signal - calibration_signal.min()) / (
            calibration_signal.max() - calibration_signal.min())
        
        # Step 1: Apply threshold BEFORE deduplication (like MATLAB)
        # Maxima
        max_indxs, _ = find_peaks(calibration_signal_01)
        max_pks = calibration_signal_01[max_indxs]

        # Minima (by finding peaks of inverted signal)
        min_indxs, _ = find_peaks(-calibration_signal_01)
        min_pks = calibration_signal_01[min_indxs]  # already positive, no need to negate


        max_mask = max_pks > 0.5
        min_mask = min_pks < 0.5
        max_indxs, max_pks = max_indxs[max_mask], max_pks[max_mask]
        min_indxs, min_pks = min_indxs[min_mask], min_pks[min_mask]

        # Step 2: Deduplicate peaks within 5 samples (like MATLAB)
        max_indxs, max_pks = deduplicate_peaks(max_indxs, max_pks, "max")
        min_indxs, min_pks = deduplicate_peaks(min_indxs, min_pks, "min")

        # Step 3: Apply scaling threshold
        new_scaling_val = 0.7
        max_pks = np.where(max_pks > new_scaling_val, max_pks, np.nan)
        min_pks = np.where(min_pks < (1 - new_scaling_val), min_pks, np.nan)

        # Step 4: Replace extreme outliers with NaN (instead of deleting)
        if np.nanmax(max_pks) == np.nanmax(max_pks):
            max_pks[np.nanargmax(max_pks)] = np.nan
        if np.nanmin(min_pks) == np.nanmin(min_pks):
            min_pks[np.nanargmin(min_pks)] = np.nan

        # Step 5: Compute means ignoring NaN (like MATLAB's "omitnan")
        max_calibration_signal_01 = np.nanmean(max_pks)
        min_calibration_signal_01 = np.nanmean(min_pks)

        # Step 6: Convert back to original scale
        max_calibration_signal = max_calibration_signal_01 * (
            calibration_signal.max() - calibration_signal.min()) + calibration_signal.min()
        min_calibration_signal = min_calibration_signal_01 * (
            calibration_signal.max() - calibration_signal.min()) + calibration_signal.min()

        
        # Save calibration parameters
        calibration_data = {
            "mu": mu,
            "coeff": coeff,
            "explained": explained,
            "min_calibration_signal": min_calibration_signal,
            "max_calibration_signal": max_calibration_signal
        }
        
        # Save to file
        filename = f"{calibration_name}.mat"
        savemat(filename, calibration_data)
        
        print(f"Calibration completed and saved to {filename}")
        print(f"Span of motion: {max_calibration_signal - min_calibration_signal}")
        
        # Optional: Plot calibration signal
        if True:  # Set to True to enable plotting
            # Create time vector
            timeVec = np.array(timestamps) 
            timeVec = timeVec - timeVec[0]  # Start from zero
            
            # Create figure with specified style
            plt.figure(figsize=(10, 6))
            plt.plot(timeVec, calibration_signal, 'b', linewidth=2)
            
            # Add horizontal lines for min and max
            plt.axhline(y=min_calibration_signal, color='r', linestyle='--', linewidth=2)
            plt.axhline(y=max_calibration_signal, color='g', linestyle='--', linewidth=2)
            
            # Set titles and labels
            plt.title('Signal projected on main direction of motion', fontsize=14)
            plt.xlabel('Time [s]', fontsize=12)
            plt.ylabel('Values', fontsize=12)
            plt.legend(['Projection', 'Mean local Minima', 'Mean local Maxima'], loc='best')
            plt.grid(True)

            plt.show()
        
        
        
        return calibration_data
    
    def calibrate_2arm(self, imu = "2arm", saved_data=True): # imu can be "hand"(right) or "2arm"(left)
        """
        Calibrate the Movella system for 2-arm trigger signal.
        Press 's' to set START quaternion, 'e' to set END quaternion, 'q' to finish and save.
        """
        all_data = []
        all_data_save = []
        timestamps = []
        q_start = None
        q_end = None
        q_start_array = np.zeros(4)
        q_end_array = np.zeros(4)

        print("Starting 2-arm calibration.")
        print("Press 's' to set START quaternion, 'e' to set END quaternion, 'esc' to finish and save.")

        self.end = False

        while not self.end:
            latest = self.get_latest_data()
            if latest and latest["n_working_movella"] == self.n_trackers:
                quat_data_save = latest["data"]
                quat_data = quat_data_save.reshape(-1, 4)
                timestamps.append(latest["timestamp"])
                all_data.append(quat_data)
                all_data_save.append(quat_data_save)

                if keyboard.is_pressed('s'):
                    if imu == "hand":
                        q_start = quat_data[0].copy()  # First tracker as trigger
                    else:
                        q_start = quat_data[-1].copy()  # Last tracker as trigger
                    print("START quaternion set:", q_start)
                    self.start_quat = False
                    time.sleep(0.2)  # Debounce

                if keyboard.is_pressed('e'):
                    if imu == "hand":
                        q_end = quat_data[0].copy()  # First tracker as trigger
                    else:
                        q_end = quat_data[-1].copy()
                    print("END quaternion set:", q_end)
                    self.end_quat = False
                    time.sleep(0.2) # Debounce

            time.sleep(self.sample_rate)

        if not all_data:
            raise RuntimeError("No data collected during calibration")

        # Save all raw data
        all_data = np.array(all_data)  # (n_samples, n_trackers, 4)

        # Prepare arrays for saving
        q_start_array = q_start if q_start is not None else np.zeros(4)
        q_end_array = q_end if q_end is not None else np.zeros(4)

        if saved_data:
            # Ask for filename
            filename_prefix = input("PHASE 2: Enter filename to save data (without extension): ").strip()
            if not filename_prefix:
                print("PHASE 2: No filename provided. Data not saved.")
            else:
                import datetime
                currentDateTime = datetime.datetime.now().strftime('%Y_%m_%d_%H%M%S')
                if self.n_trackers <=2:
                    filename = f"data/{filename_prefix}_2arm_{imu}_{currentDateTime}.mat"
                else:
                    filename = f"data/{filename_prefix}_2arm_{currentDateTime}.mat"

                savemat(filename, {
                    'tracked_data_2arm': all_data_save,
                    'q_start': q_start,
                    'q_end': q_end,
                    'q_start_array': q_start_array,
                    'q_end_array': q_end_array
                })

                print(f"PHASE 2: Data saved to {filename}")

        if self.n_trackers <=2:
            savemat(f"movellaValue_{imu}.mat", {
                'tracked_data_2arm': all_data,
                'q_start_array': q_start_array,
                'q_end_array': q_end_array
            })
             
        else:
            savemat("movellaValue2Phase.mat", {
                'tracked_data_2arm': all_data,
                'q_start_array': q_start_array,
                'q_end_array': q_end_array
            })

    
    
    def compute_kernel(self, send_ip="172.16.0.1", send_port=8052, plot_data=False):

        SEND_IP = send_ip
        SEND_PORT1 = send_port
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      
        # Load PCA
        pca_data = loadmat ("movellaValue1Phase.mat")
        mu = pca_data["mu"].flatten()
        coeff = pca_data["coeff"]
        explained = pca_data["explained"].flatten()
        min_cal = float(pca_data["min_calibration_signal"])
        max_cal = float(pca_data["max_calibration_signal"])
        n_components = np.where(np.cumsum(explained) > 80)[0]
        sampleRate = 1 / 90  # 90 Hz (adjust as necessary)

        # State variables
        controlVal_01 = 0.0
        flip_sign_control = 1  # Variable to track sign flipping

        self.end = False

        if self.n_trackers == 5:
            # Load Trigger calibration 
            trigger_data = loadmat("movellaValue2Phase.mat")
            # trigger_data = loadmat("movellaValue2Phase.mat", squeeze_me=True)


            q_start = trigger_data["q_start_array"].flatten()  # Start quaternion
            q_end = trigger_data["q_end_array"].flatten()  # End quaternion

            # --- Calibration rotation ---
            q_start_r = R.from_quat([q_start[1], q_start[2], q_start[3], q_start[0]])
            q_end_r   = R.from_quat([q_end[1],   q_end[2],   q_end[3],   q_end[0]])

            # Relative quaternion: q_end * conj(q_start)
            q_calib = q_end_r * q_start_r.inv()
            rotvec_calib = q_calib.as_rotvec()

            flip_sign_trigger = 1  # Variable to track sign flipping
            triggerVal_01 = 0.0

        if plot_data:
            self._init_realtime_plot(2 if self.n_trackers == 5 else 1)

        try:
            print("Starting kernel loop. Press ESC to stop.")
            while not self.end:

                if keyboard.is_pressed("c"):  # Check if "s" is pressed
                    flip_sign_control *= -1  # Toggle between 1 and -1
                    time.sleep(0.1)  # Add a small delay to prevent multiple triggers

                if self.n_trackers == 5:   
                    if keyboard.is_pressed("t"):  # Check if "w" is pressed
                        flip_sign_trigger *= -1   # Reset to default
                        time.sleep(0.1)
                    
                latest = self.get_latest_data()  # from your class
                data = latest["data"] if latest is not None else None

                if data is None:
                    # print("No data received from sensors. Retrying...")
                    continue

                data = data.reshape(-1, 4)  # Make sure it's (N, 4)
                data_control = data[0:4, :]  # Use the first 4 quaternions for control

                
                quatRelative = getRelativeRotation_movella.getRelativeRotation_movella(data_control)  # returns list of quaternions
                quatRelative = np.concatenate([q.elements for q in quatRelative])
                dataProjected = (quatRelative - mu) @ coeff

                if n_components[0] == 0:
                    controlVal = dataProjected[0]
                else:
                    controlVal = np.linalg.norm(dataProjected[:n_components[0]])

                controlVal_01 = (controlVal - min_cal) / (max_cal - min_cal)

                if flip_sign_control == -1:
                    controlVal_01 = 1 - controlVal_01

                controlVal_01 = max(0.0, min(1.0, controlVal_01))

                # print(f"Control Value: {controlVal_01:.3f}", end="")

                # Trigger computation

                if self.n_trackers == 5:
                    data_trigger = data[4:, :]  # Use the remaining quaternions for trigger

                    if data_trigger.shape[0] < 1:
                        print("Not enough trigger data.")
                        continue

                    q_meas = R.from_quat([data_trigger[0, 1], data_trigger[0, 2], data_trigger[0, 3], data_trigger[0, 0]])

                    q_meas_rel = q_meas * q_start_r.inv()
                    rotvec_meas = q_meas_rel.as_rotvec()

                    # --- Projection onto calibration axis ---
                    triggerVal_01 = np.dot(rotvec_meas, rotvec_calib) / np.linalg.norm(rotvec_calib)**2

                    if flip_sign_trigger == -1:
                        triggerVal_01 = 1 - triggerVal_01

                    # Clamp to [0,1]
                    triggerVal_01 = max(0.0, min(1.0,triggerVal_01))

                    if plot_data:
                        self._update_realtime_plot([controlVal_01, triggerVal_01])

                    # Prepare and send UDP packet
                    arr = np.array([controlVal_01, triggerVal_01], dtype=np.float64)
                    sock.sendto(arr.tobytes(), (SEND_IP, SEND_PORT1))
                else:
                    if plot_data:
                        self._update_realtime_plot([controlVal_01])
                    arr = np.array([controlVal_01], dtype=np.float64)
                    sock.sendto(arr.tobytes(), (SEND_IP, SEND_PORT1))
                
                time.sleep(sampleRate)  # Reduce CPU usage

        except KeyboardInterrupt:
            print("UDP transmission interrupted by the user.")
            self.cleanup()
            sock.close()

        except Exception as e:
            print(f"An error occurred: {e}")
            self.cleanup()
            sock.close()

        finally:
            # Close the socket after finishing
            print("Sensor handler stopped.")
            sock.close()
            if plot_data:
                self._close_realtime_plot()
            print("UDP socket closed.")

    def compute_easy_kernel(self, send_ip="172.16.0.1", send_port=8052, plot_data=False):
        SEND_IP = send_ip
        SEND_PORT1 = send_port
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sampleRate = 1 / 90  # 90 Hz (adjust as necessary)


        # State variables
        controlVal_hand = 0.0
        flip_sign_hand = 1  # Variable to track sign flipping

        # Load data calibration 
        hand_data = loadmat("movellaValue_hand.mat")
        # trigger_data = loadmat("movellaValue2Phase.mat", squeeze_me=True)

        q_start = hand_data["q_start_array"].flatten()  # Start quaternion
        q_end = hand_data["q_end_array"].flatten()  # End quaternion

        # --- Calibration rotation ---
        q_start_r_hand = R.from_quat([q_start[1], q_start[2], q_start[3], q_start[0]])
        q_end_r_hand   = R.from_quat([q_end[1],   q_end[2],   q_end[3],   q_end[0]])

        # Relative quaternion: q_end * conj(q_start)
        q_calib_hand = q_end_r_hand * q_start_r_hand.inv()
        rotvec_calib_hand = q_calib_hand.as_rotvec()

        self.end = False

        if self.n_trackers == 2:
            controlVal_2arm = 0.0
            flip_sign_2arm = 1  # Variable to track sign flipping

             # Load data calibration 
            secondArm_data = loadmat("movellaValue_2arm.mat")
            # trigger_data = loadmat("movellaValue2Phase.mat", squeeze_me=True)

            q_start = secondArm_data["q_start_array"].flatten()  # Start quaternion
            q_end = secondArm_data["q_end_array"].flatten()  # End quaternion

            # --- Calibration rotation ---
            q_start_r_2arm = R.from_quat([q_start[1], q_start[2], q_start[3], q_start[0]])
            q_end_r_2arm   = R.from_quat([q_end[1],   q_end[2],   q_end[3],   q_end[0]])

            # Relative quaternion: q_end * conj(q_start)
            q_calib_2arm = q_end_r_2arm * q_start_r_2arm.inv()
            rotvec_calib_2arm = q_calib_2arm.as_rotvec()

        if plot_data:
            self._init_realtime_plot(2 if self.n_trackers == 2 else 1)

        try:
            print("Starting kernel loop. Press ESC to stop.")
            while not self.end:

                if keyboard.is_pressed("c"):  # Check if "s" is pressed
                    flip_sign_hand *= -1  # Toggle between 1 and -1
                    time.sleep(0.1)  # Add a small delay to prevent multiple triggers

                if self.n_trackers == 2:   
                    if keyboard.is_pressed("t"):  # Check if "w" is pressed
                        flip_sign_2arm *= -1   # Reset to default
                        time.sleep(0.1)
                    
                latest = self.get_latest_data()  # from your class
                data = latest["data"] if latest is not None else None

                if data is None:
                    # print("No data received from sensors. Retrying...")
                    continue

                data = data.reshape(-1, 4)  # Make sure it's (N, 4)

                data_hand = data[0:1, :]  # Use the first 4 quaternions for control

                if data_hand.shape[0] < 1:
                        print("Not enough trigger data.")
                        continue

                q_meas_hand = R.from_quat([data_hand[0, 1], data_hand[0, 2], data_hand[0, 3], data_hand[0, 0]])

                q_meas_rel_hand = q_meas_hand * q_start_r_hand.inv()
                rotvec_meas_hand = q_meas_rel_hand.as_rotvec()

                # --- Projection onto calibration axis ---
                controlVal_hand = np.dot(rotvec_meas_hand, rotvec_calib_hand) / np.linalg.norm(rotvec_calib_hand)**2
                
               
                if flip_sign_hand == -1:
                    controlVal_hand = 1 - controlVal_hand

                controlVal_hand = max(0.0, min(1.0, controlVal_hand))

                # Trigger computation

                if self.n_trackers == 2:
                    data_2arm = data[1:, :]  # Use the remaining quaternions for trigger

                    if data_2arm.shape[0] < 1:
                        print("Not enough trigger data.")
                        continue

                    q_meas_2arm = R.from_quat([data_2arm[0, 1], data_2arm[0, 2], data_2arm[0, 3], data_2arm[0, 0]])

                    q_meas_rel_2arm = q_meas_2arm * q_start_r_2arm.inv()
                    rotvec_meas_2arm = q_meas_rel_2arm.as_rotvec()

                    # --- Projection onto calibration axis ---
                    controlVal_2arm = np.dot(rotvec_meas_2arm, rotvec_calib_2arm) / np.linalg.norm(rotvec_calib_2arm)**2

                    if flip_sign_2arm == -1:
                        controlVal_2arm = 1 - controlVal_2arm

                    # Clamp to [0,1]
                    controlVal_2arm = max(0.0, min(1.0,controlVal_2arm))

                    if plot_data:
                        self._update_realtime_plot([controlVal_hand, controlVal_2arm])

                    # Prepare and send UDP packet
                    arr = np.array([controlVal_hand, controlVal_2arm], dtype=np.float64)
                    sock.sendto(arr.tobytes(), (SEND_IP, SEND_PORT1))
                    # print(f"Hand: {controlVal_hand:.3f}, 2arm: {controlVal_2arm:.3f}")
                else:
                    if plot_data:
                        self._update_realtime_plot([controlVal_hand])
                    arr = np.array([controlVal_hand], dtype=np.float64)
                    sock.sendto(arr.tobytes(), (SEND_IP, SEND_PORT1))
                
                time.sleep(sampleRate)  # Reduce CPU usage

        except KeyboardInterrupt:
            print("UDP transmission interrupted by the user.")
            self.cleanup()
            sock.close()

        except Exception as e:
            print(f"An error occurred: {e}")
            self.cleanup()
            sock.close()

        finally:
            # Close the socket after finishing
            print("Sensor handler stopped.")
            sock.close()
            if plot_data:
                self._close_realtime_plot()
            print("UDP socket closed.")
            

    def cleanup(self):

        for device in self.xdpcHandler.connectedDots():
            try:
                if device.resetOrientation(self.movelladot_pc_sdk.XRM_Heading):
                    print(f"Reset orientation for device {device.portInfo().bluetoothAddress()}: OK", flush=True)
                else:
                    print(f"Reset orientation for device {device.portInfo().bluetoothAddress()}: NOK: {device.lastResultText()}", flush=True)
            except Exception as e:
                print(f"Error resetting orientation for device {device.portInfo().bluetoothAddress()}: {e}", flush=True)

        print("\nStopping measurement...")
        for device in self.xdpcHandler.connectedDots():
            try:
                if not device.stopMeasurement():
                    print("Failed to stop measurement.")
                if not device.disableLogging():
                    print("Failed to disable logging.")
            except Exception as e:
                print(f"Error stopping measurement for device {device.portInfo().bluetoothAddress()}: {e}", flush=True)

        self.xdpcHandler.cleanup()
