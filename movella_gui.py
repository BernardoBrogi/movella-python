import threading
import time
import sys
import tkinter as tk
from tkinter import scrolledtext, ttk, messagebox, simpledialog

from movella_streamer_class import MovellaStreamer


class TextRedirector:
    def __init__(self, text_widget):
        self.text_widget = text_widget

    def write(self, s):
        if not s:
            return
        # Insert in GUI thread
        def insert():
            self.text_widget.insert(tk.END, s)
            self.text_widget.see(tk.END)
        try:
            self.text_widget.after(0, insert)
        except Exception:
            pass

    def flush(self):
        pass


class MovellaGUI:
    def __init__(self, master):
        self.master = master
        master.title("Movella Controller GUI")

        self.tracker = None
        self.init_thread = None
        self.compute_thread = None

        frm = ttk.Frame(master, padding=8)
        frm.grid(sticky="nsew")

        # Config entries
        ttk.Label(frm, text="Config file").grid(column=0, row=0, sticky="w")
        self.config_entry = ttk.Entry(frm, width=30)
        self.config_entry.insert(0, "config.json")
        self.config_entry.grid(column=1, row=0, sticky="w")

        ttk.Label(frm, text="Setup name").grid(column=0, row=1, sticky="w")
        self.setup_entry = ttk.Entry(frm, width=20)
        self.setup_entry.insert(0, "UNISI")
        self.setup_entry.grid(column=1, row=1, sticky="w")

        ttk.Label(frm, text="# Trackers").grid(column=0, row=2, sticky="w")
        self.n_entry = ttk.Entry(frm, width=6)
        self.n_entry.insert(0, "4")
        self.n_entry.grid(column=1, row=2, sticky="w")

        ttk.Label(frm, text="Robot IP").grid(column=0, row=3, sticky="w")
        self.send_ip_entry = ttk.Entry(frm, width=20)
        self.send_ip_entry.insert(0, "172.16.0.1")
        self.send_ip_entry.grid(column=1, row=3, sticky="w")

        ttk.Label(frm, text="Robot Port").grid(column=0, row=4, sticky="w")
        self.send_port_entry = ttk.Entry(frm, width=8)
        self.send_port_entry.insert(0, "8052")
        self.send_port_entry.grid(column=1, row=4, sticky="w")

        # Buttons
        btn_frame = ttk.Frame(frm)
        btn_frame.grid(column=0, row=5, columnspan=2, pady=8)

        self.init_btn = ttk.Button(btn_frame, text="Initialize", command=self.initialize)
        self.init_btn.grid(column=0, row=0, padx=4)

        self.calibrate_btn = ttk.Button(btn_frame, text="Calibrate", command=self.calibrate)
        self.calibrate_btn.grid(column=1, row=0, padx=4)

        self.calibrate_2arm_btn = ttk.Button(btn_frame, text="Calibrate 2-Arm", command=self.calibrate_2arm)
        self.calibrate_2arm_btn.grid(column=2, row=0, padx=4)

        self.start_btn = ttk.Button(btn_frame, text="Start Compute", command=self.start_compute)
        self.start_btn.grid(column=3, row=0, padx=4)

        self.cleanup_btn = ttk.Button(btn_frame, text="Cleanup", command=lambda: self.cleanup(close_gui=True))
        self.cleanup_btn.grid(column=4, row=0, padx=4)

        # Log area
        ttk.Label(frm, text="Log").grid(column=0, row=6, sticky="w")
        self.log = scrolledtext.ScrolledText(frm, width=80, height=20)
        self.log.grid(column=0, row=7, columnspan=2, pady=4)

        # Redirect stdout/stderr
        self._stdout = sys.stdout
        self._stderr = sys.stderr
        sys.stdout = TextRedirector(self.log)
        sys.stderr = TextRedirector(self.log)

        # State
        self.compute_running = False
        # Keep track of IMU names used for 2-arm calibration (append as calibrations finish)
        self.calibrated_imus = []

    def _ask_imu_name(self, title, prompt, default_value="2arm"):
        if threading.current_thread() is not threading.main_thread():
            return default_value

        value = simpledialog.askstring(title, prompt, parent=self.master, initialvalue=default_value)
        if value is None:
            return None

        value = value.strip()
        return value or None

    def initialize(self):
        if self.init_thread and self.init_thread.is_alive():
            messagebox.showinfo("Info", "Initialization already running")
            return

        def target():
            try:
                cfg = self.config_entry.get().strip()
                setup = self.setup_entry.get().strip()
                n = int(self.n_entry.get().strip())

                self.tracker = MovellaStreamer(cfg, setup, n_trackers=n)
                self.tracker.initialize()
                # Report detected trackers
                msg = f"Initialization complete. Found trackers: {len(self.tracker.working_tracker)}\n"
                print(msg)
            except Exception as e:
                print(f"Initialization error: {e}")

        self.init_thread = threading.Thread(target=target, daemon=True)
        self.init_thread.start()

    def calibrate(self):
        if not self.tracker:
            messagebox.showwarning("Warning", "Please initialize first")
            return

        def target():
            try:
                # print("Starting calibration...")
                self.tracker.calibrate(saved_data=False)  # set to True to save raw calibration data
                # print("Calibration complete!")
            except Exception as e:
                print(f"Calibration error: {e}")

        cal_thread = threading.Thread(target=target, daemon=True)
        cal_thread.start()

    def calibrate_2arm(self):
        if not self.tracker:
            messagebox.showwarning("Warning", "Please initialize first")
            return

        imu_name = "2arm"
        if self.n_entry.get().strip() == "2":
            requested_name = self._ask_imu_name(
                "2 IMU calibration",
                "Enter IMU name to save the calibration file:",
                default_value="2arm"
            )
            if not requested_name:
                print("2-Arm calibration cancelled.")
                return
            imu_name = requested_name

        def target():
            try:
                self.tracker.calibrate_2arm(imu=imu_name, saved_data=False)
                # Record the IMU name used for this 2-arm calibration so compute can use it
                try:
                    if imu_name and imu_name not in self.calibrated_imus:
                        self.calibrated_imus.append(imu_name)
                        print(f"Recorded 2-arm calibration IMU name: {imu_name}")
                except Exception:
                    pass
            except Exception as e:
                print(f"2-Arm calibration error: {e}")

        cal_thread = threading.Thread(target=target, daemon=True)
        cal_thread.start()

    def start_compute(self):
        if not self.tracker:
            messagebox.showwarning("Warning", "Please initialize first")
            return
        if self.compute_running:
            messagebox.showinfo("Info", "Compute already running")
            return

        send_ip = self.send_ip_entry.get().strip()
        try:
            send_port = int(self.send_port_entry.get().strip())
        except ValueError:
            messagebox.showerror("Error", "Invalid port")
            return

        def runner():
            try:
                self.compute_running = True
                if int(self.n_entry.get().strip()) <= 2:
                    # Prefer the last two recorded calibrated IMU names (most recent first)
                    if len(self.calibrated_imus) >= 2:
                        tracker2 = self.calibrated_imus[-1]
                        tracker1 = self.calibrated_imus[-2]
                    elif len(self.calibrated_imus) == 1:
                        tracker1 = self.calibrated_imus[0]
                        tracker2 = "hand" if tracker1 != "hand" else "2arm"
                    else:
                        tracker1 = "2arm"
                        tracker2 = "hand"
                    self.tracker.compute_easy_kernel(send_ip=send_ip, send_port=send_port, plot_data=True, tracker1=tracker1, tracker2=tracker2)
                else:
                    # Use compute_kernel with plot_data=True so the realtime plot appears
                    self.tracker.compute_kernel(send_ip=send_ip, send_port=send_port, plot_data=True)
            except Exception as e:
                print(f"Compute error: {e}")
            finally:
                self.compute_running = False
                # After compute finishes, perform cleanup and close the GUI
                try:
                    self.cleanup(close_gui=True)
                except Exception:
                    pass

        self.compute_thread = threading.Thread(target=runner, daemon=True)
        self.compute_thread.start()

    def cleanup(self, close_gui=False):
        """Cleanup tracker resources. If close_gui is True, also close the GUI and exit.

        Note: GUI close/destroy is scheduled on the main thread using `after` to be
        safe when called from worker threads.
        """
        if self.tracker:
            print("Running cleanup...")
            try:
                # Signal tracker threads to stop
                try:
                    self.tracker.end = True
                except Exception:
                    pass
                # give threads a moment
                time.sleep(0.1)
                try:
                    self.tracker.cleanup()
                except Exception as e:
                    print(f"Cleanup error during tracker.cleanup(): {e}")
            except Exception as e:
                print(f"Cleanup error: {e}")
            self.tracker = None
        else:
            print("Nothing to cleanup.")

        if close_gui:
            def do_close():
                try:
                    # restore stdout/stderr
                    sys.stdout = self._stdout
                    sys.stderr = self._stderr
                except Exception:
                    pass
                try:
                    self.master.destroy()
                except Exception:
                    pass

            try:
                # Schedule on main thread
                self.master.after(0, do_close)
            except Exception:
                # Fallback: call directly
                do_close()

    def on_closing(self):
        self.cleanup()
        # restore stdout
        sys.stdout = self._stdout
        sys.stderr = self._stderr
        self.master.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = MovellaGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
