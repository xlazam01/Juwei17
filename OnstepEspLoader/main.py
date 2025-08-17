import tkinter as tk
from tkinter import ttk, messagebox
import serial.tools.list_ports
import subprocess

class OnstepEspLoaderApp(tk.Tk):

    def __init__(self):
        super().__init__()
        self.title("OnstepEspLoader")
        self.geometry("700x500")
        self.resizable(True, True)

        # --- Top: STM32 firmware selection ---
        top_frame = ttk.Frame(self)
        top_frame.pack(pady=(10, 0), fill=tk.X)

        self.own_firmware_var = tk.BooleanVar(value=False)
        own_fw_cb = ttk.Checkbutton(top_frame, text="Own STM32 firmware", variable=self.own_firmware_var, command=self.on_own_firmware_toggle)
        own_fw_cb.pack(side=tk.LEFT, padx=(5, 10))

        # Browse button and firmware path textbox
        self.firmware_path_var = tk.StringVar()
        self.browse_btn = ttk.Button(top_frame, text="Browse", command=self.browse_firmware, state="disabled")
        self.browse_btn.pack(side=tk.LEFT)
        self.firmware_entry = ttk.Entry(top_frame, textvariable=self.firmware_path_var, width=40, state="readonly")
        self.firmware_entry.pack(side=tk.LEFT, padx=5)

        # Filename to be uploaded (caption/non-editable)
        self.upload_filename_var = tk.StringVar()
        filename_frame = ttk.Frame(self)
        filename_frame.pack(pady=(2, 0), fill=tk.X)
        ttk.Label(filename_frame, text="Firmware to be uploaded:").pack(side=tk.LEFT, padx=(10, 5))
        self.upload_filename_entry = ttk.Entry(filename_frame, textvariable=self.upload_filename_var, width=45, state="readonly")
        self.upload_filename_entry.pack(side=tk.LEFT)

        # TMC radio buttons and homing checkbox (below Browse)
        tmc_homing_frame = ttk.Frame(self)
        tmc_homing_frame.pack(pady=(0, 0), fill=tk.X)
        self.tmc_var = tk.StringVar(value="TMC2209")
        self.homing_var = tk.BooleanVar(value=True)
        self.tmc_var.trace_add("write", lambda *args: self.update_upload_filename())
        self.homing_var.trace_add("write", lambda *args: self.update_upload_filename())
        self.tmc_frame = ttk.LabelFrame(tmc_homing_frame, text="TMC Module")
        self.tmc_radio_2209 = ttk.Radiobutton(self.tmc_frame, text="TMC2209", variable=self.tmc_var, value="TMC2209")
        self.tmc_radio_2130 = ttk.Radiobutton(self.tmc_frame, text="TMC2130", variable=self.tmc_var, value="TMC2130")
        self.tmc_radio_5160 = ttk.Radiobutton(self.tmc_frame, text="TMC5160", variable=self.tmc_var, value="TMC5160")
        self.tmc_radio_2209.pack(side=tk.LEFT, padx=2)
        self.tmc_radio_2130.pack(side=tk.LEFT, padx=2)
        self.tmc_radio_5160.pack(side=tk.LEFT, padx=2)
        self.tmc_frame.pack(side=tk.LEFT, padx=10)
        self.homing_cb = ttk.Checkbutton(tmc_homing_frame, text="Enable Homing", variable=self.homing_var)
        self.homing_cb.pack(side=tk.LEFT, padx=10)

        # Serial port selection
        self.serial_ports = self.get_serial_ports()
        self.selected_port = tk.StringVar()
        port_frame = ttk.Frame(self)
        port_frame.pack(pady=(15, 0))
        ttk.Label(port_frame, text="Select Serial Port:").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(port_frame, textvariable=self.selected_port, values=self.serial_ports, state="readonly", width=30)
        self.port_combo.pack(side=tk.LEFT, padx=5)
        ttk.Button(port_frame, text="Refresh", command=self.refresh_ports).pack(side=tk.LEFT)
        if self.serial_ports:
            self.port_combo.current(0)

        # Action buttons
        btn_frame = ttk.Frame(self)
        btn_frame.pack(pady=10)
        ttk.Button(btn_frame, text="Check STM32", command=self.read_stm32_id).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Upload STM32", command=self.flash_stm32).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Check OnStepX", command=self.check_onstex).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="SWS Upload Mode", command=self.switch_onstep_mode).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Upload ESP", command=self.upload_esp8266).pack(side=tk.LEFT, padx=5)

        # Status/console output
        ttk.Label(self, text="Console Output:").pack(pady=(10, 0))
        self.console = tk.Text(self, height=18, width=90, state="disabled", wrap="word", bg="#222", fg="#eee")
        self.console.pack(padx=10, pady=(0, 10), fill=tk.BOTH, expand=True)

        # Initial state
        self.update_upload_filename()
        self.on_own_firmware_toggle()

    def on_own_firmware_toggle(self):
        # Only set 'state' on widgets that support it (Radiobutton, Checkbutton, Button, Entry)
        if self.own_firmware_var.get():
            self.browse_btn.config(state="normal")
            self.firmware_entry.config(state="readonly")
            self.tmc_radio_2209.config(state="disabled")
            self.tmc_radio_2130.config(state="disabled")
            self.tmc_radio_5160.config(state="disabled")
            self.homing_cb.config(state="disabled")
        else:
            self.browse_btn.config(state="disabled")
            self.firmware_entry.config(state="readonly")
            self.tmc_radio_2209.config(state="normal")
            self.tmc_radio_2130.config(state="normal")
            self.tmc_radio_5160.config(state="normal")
            self.homing_cb.config(state="normal")
        self.update_upload_filename()

    def browse_firmware(self):
        from tkinter import filedialog
        file_path = filedialog.askopenfilename(title="Select STM32 Firmware", filetypes=[("Binary Files", "*.bin"), ("All Files", "*.*")])
        if file_path:
            self.firmware_path_var.set(file_path)
        self.update_upload_filename()

    def check_onstex(self):
        import threading
        import serial
        port = self.selected_port.get()
        if not port:
            self.append_console("[ERROR] No serial port selected.")
            return
        self.append_console(f"[INFO] Checking OnStepX firmware on {port} at 9600 baud...")

        def worker():
            try:
                with serial.Serial(port, 9600, timeout=2) as ser:
                    # Send :GVP# to get firmware name
                    ser.write(":GVP#\r\n".encode('ascii'))
                    name_resp = ser.read(256).decode(errors='replace').strip()
                    if name_resp:
                        name_resp = name_resp[:-1]
                    self.append_console(f"[INFO] Firmware name response: {name_resp}")
                    # Send :GVN# to get firmware version
                    ser.write(":GVN#\r\n".encode('ascii'))
                    ver_resp = ser.read(256).decode(errors='replace').strip()
                    if ver_resp:
                        ver_resp = ver_resp[:-1]
                    self.append_console(f"[INFO] Firmware version response: {ver_resp}")
                    # Try to extract and display as 'OnStepX 3.16'
                    name = name_resp.split()[0] if name_resp else "?"
                    version = ver_resp.split()[0] if ver_resp else "?"
                    self.append_console(f"[SUCCESS] {name} {version}")
            except serial.SerialException as e:
                self.append_console(f"[ERROR] Serial error: {e}")
            except Exception as e:
                self.append_console(f"[ERROR] Exception: {e}")

        threading.Thread(target=worker, daemon=True).start()

    def get_serial_ports(self):
        return [port.device for port in serial.tools.list_ports.comports()]

    def refresh_ports(self):
        ports = self.get_serial_ports()
        self.port_combo['values'] = ports
        self.serial_ports = ports
        if ports:
            self.port_combo.current(0)
            self.selected_port.set(ports[0])
        else:
            self.selected_port.set("")

    def append_console(self, text):
        self.console.configure(state="normal")
        self.console.insert(tk.END, text + "\n")
        self.console.see(tk.END)
        self.console.configure(state="disabled")

    def read_stm32_id(self):
        import threading
        port = self.selected_port.get()
        if not port:
            self.append_console("[ERROR] No serial port selected.")
            return
        self.append_console(f"[INFO] Reading STM32 Device ID on {port} using stm32loader CLI...")

        def worker():
            try:
                process = subprocess.Popen([
                    "stm32loader",
                    "-p", port,
                    "-b", "115200",
                    "-f", "F4"
                ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1, universal_newlines=True)
                output_lines = []
                for line in process.stdout:
                    line = line.rstrip()
                    output_lines.append(line)
                    self.append_console(line)
                process.wait()
                if process.returncode == 0:
                    self.append_console("[SUCCESS] STM32 check complete.")
                else:
                    self.append_console("[ERROR] STM32 check failed.")
            except FileNotFoundError:
                self.append_console("[ERROR] stm32loader not found. Please ensure it is installed and in your PATH.")
            except Exception as e:
                self.append_console(f"[ERROR] Exception: {e}")

        threading.Thread(target=worker, daemon=True).start()

    def flash_stm32(self):
        import threading
        port = self.selected_port.get()
        # Determine firmware file
        if self.own_firmware_var.get():
            firmware = self.firmware_path_var.get()
            if not firmware:
                self.append_console("[ERROR] No firmware file selected.")
                return
        else:
            tmc = self.tmc_var.get()
            homing = self.homing_var.get()
            if tmc == "TMC5160":
                firmware = "firmware_5160_wH.bin" if homing else "firmware_5160_woH.bin"
            elif tmc == "TMC2130":
                firmware = "firmware_2130_wH.bin" if homing else "firmware_2130_woH.bin"
            else:
                firmware = "firmware_2209_wH.bin" if homing else "firmware_2209_woH.bin"

        if not port:
            self.append_console("[ERROR] No serial port selected.")
            return
        self.append_console(f"[INFO] Flashing STM32 on {port} with {firmware} using stm32loader...")

        def worker():
            try:
                process = subprocess.Popen([
                    "stm32loader",
                    "-p", port,
                    "-b", "115200",
                    "--family", "F4",
                    "--erase",
                    "--write",
                    "--verify",
                    firmware
                ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1, universal_newlines=True)
                output_lines = []
                for line in process.stdout:
                    line = line.rstrip()
                    output_lines.append(line)
                    self.append_console(line)
                process.wait()
                if process.returncode == 0:
                    self.append_console("[SUCCESS] Flash and verify complete.")
                    self.append_console("[INFO] Starting OnStepX...")
                    # Run go-address command
                    try:
                        go_proc = subprocess.Popen([
                            "stm32loader",
                            "--family", "F4",
                            "-p", port,
                            "-b", "115200",
                            "--go-address", "0x08000000"
                        ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1, universal_newlines=True)
                        for go_line in go_proc.stdout:
                            go_line = go_line.rstrip()
                            self.append_console(go_line)
                        go_proc.wait()
                        if go_proc.returncode == 0:
                            self.append_console("[SUCCESS] OnStepX started.")
                        else:
                            self.append_console("[ERROR] Failed to start OnStepX.")
                    except Exception as e:
                        self.append_console(f"[ERROR] Exception during OnStepX start: {e}")
                else:
                    self.append_console("[ERROR] Flash/verify failed.")
            except FileNotFoundError:
                self.append_console("[ERROR] stm32loader not found. Please ensure it is installed and in your PATH.")
            except Exception as e:
                self.append_console(f"[ERROR] Exception: {e}")

        threading.Thread(target=worker, daemon=True).start()

    def switch_onstep_mode(self):
        import threading
        import serial
        port = self.selected_port.get()
        if not port:
            self.append_console("[ERROR] No serial port selected.")
            return
        self.append_console(f"[INFO] Switching OnStep to SWS Upload Mode on {port} at 9600 baud...")

        def worker():
            try:
                with serial.Serial(port, 9600, timeout=2) as ser:
                    command = ":hF#:ESPFLASH#"
                    ser.write(command.encode('ascii'))
                    self.append_console(f"[INFO] Sent: {command.strip()}")
                    # Read response (up to 256 bytes or until timeout)
                    response = ser.read(256)
                    response_str = response.decode(errors='replace').strip()
                    if response_str:
                        self.append_console(f"[SUCCESS] STM32 response: {response_str}")
                    else:
                        self.append_console("[WARNING] No response from STM32.")
            except serial.SerialException as e:
                self.append_console(f"[ERROR] Serial error: {e}")
            except Exception as e:
                self.append_console(f"[ERROR] Exception: {e}")

        threading.Thread(target=worker, daemon=True).start()

    def upload_esp8266(self):
        import threading
        port = self.selected_port.get()
        firmware = "SmartWebServer.ino.bin"
        if not port:
            self.append_console("[ERROR] No serial port selected.")
            return
        self.append_console(f"[INFO] Flashing ESP8266 on {port} with {firmware} using esptool...")

        def worker():
            import os
            esptool_path = os.path.abspath("esptool.exe")
            # Common parameters for ESP8266 flashing
            # Example: esptool.exe --port COMx --baud 115200 write_flash --flash_size detect 0x00000 SmartWebServer.ino.bin
            try:
                process = subprocess.Popen([
                    esptool_path,
                    "--port", port,
                    "--baud", "115200",
                    "write_flash",
                    "--flash_size", "detect",
                    "0x00000", firmware
                ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1, universal_newlines=True)
                for line in process.stdout:
                    line = line.rstrip()
                    self.append_console(line)
                process.wait()
                if process.returncode == 0:
                    self.append_console("[SUCCESS] ESP8266 flash complete. Now turn OnStepX off and on again to start the new firmware.")
                else:
                    self.append_console("[ERROR] ESP8266 flash failed.")
            except FileNotFoundError:
                self.append_console("[ERROR] esptool.exe not found. Please ensure it is in the application directory.")
            except Exception as e:
                self.append_console(f"[ERROR] Exception: {e}")

        threading.Thread(target=worker, daemon=True).start()

    def update_upload_filename(self):
        if self.own_firmware_var.get():
            import os
            path = self.firmware_path_var.get()
            filename = os.path.basename(path) if path else ""
        else:
            tmc = self.tmc_var.get()
            homing = self.homing_var.get()
            if tmc == "TMC5160":
                filename = "firmware_5160_wH.bin" if homing else "firmware_5160_woH.bin"
            elif tmc == "TMC2130":
                filename = "firmware_2130_wH.bin" if homing else "firmware_2130_woH.bin"
            else:
                filename = "firmware_2209_wH.bin" if homing else "firmware_2209_woH.bin"
        self.upload_filename_var.set(filename)


if __name__ == "__main__":
    app = OnstepEspLoaderApp()
    app.mainloop()

