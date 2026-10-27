#!/usr/bin/env python3
# ===========================================================================
# Graphical User Interface for the AquaPing acoustic leak sensor
# Uses an I2CDriver (Excamera Labs) as the USB-to-I2C interface
# Requires the i2cdriver package on the host PC:  pip install i2cdriver
# When run from a terminal CLI, the I2C command bytes are displayed
# This code differs from aquaping_gui.py in the method of serial communication
# with the I2CDriver hardware. The I2CDriver and AquaPing firmware do not
# have to be modified.
# Consult the program user guide for more information
# MicroPhonon 2026. Licensed under Creative Commons
# ===========================================================================
import queue
import threading
import time
import tkinter as tk
from datetime import datetime
from tkinter import ttk, messagebox

import serial.tools.list_ports   # only used to list the I2CDriver's USB serial port
from i2cdriver import I2CDriver

SLAVE_ADDR = 0x77        # 7-bit I2C address of the AquaPing
I2C_SPEED_KHZ = 100      
FRAME_LEN = 12           # 12 status bytes read from the AquaPing per frame
READ_PERIOD = 1.0        # Time between I2C sensor reads

LEDS_ON = bytes([0x6C, 0x31])
LEDS_OFF = bytes([0x6C, 0x30])
SLEEP_ACTIVE = bytes([0x70, 0x31])
SLEEP_DISABLED = bytes([0x70, 0x30])
RESET = bytes([0x72, 0x72])
DELAY_CYCLES = 5
MIN_INPUT = 10
MAX_INPUT = 255

Periods = [
            "1 sec",
            "2 sec",
            "3 sec",
            "5 sec",
            "10 sec",
            "15 sec",
            "20 sec",
            "25 sec",
            "30 sec"
]

HEXPERIODS = {
            "1 sec": 0x01,
            "2 sec": 0x02,
            "3 sec": 0x03,
            "5 sec": 0x04,
            "10 sec": 0x05,
            "15 sec": 0x06,
            "20 sec": 0x07,
            "25 sec": 0x08,
            "30 sec": 0x09
}

class SerialApp:
    def __init__(self, root):
        root.title("AquaPing I2CDriver Interface")
        self.root = root
        self.q = queue.Queue()
        self.data = []  
        self.variable = tk.StringVar()
        self.data0 = tk.StringVar() 
        self.data1 = tk.StringVar() 
        self.data2 = tk.StringVar() 
        self.data3 = tk.StringVar()
        self.data4 = tk.StringVar()
        self.data8 = tk.IntVar(value=2)
        self.data5 = tk.IntVar(value=30)
        self.data6 = tk.IntVar(value=20)
        self.data7 = tk.IntVar(value=18)
        self.period = tk.StringVar()
        self.background = tk.StringVar()
        self.events = tk.StringVar()
        self.trigger = tk.StringVar()
        self.counter = tk.IntVar(value=DELAY_CYCLES) 
        self.pflag = tk.StringVar(value="1") #Determines what alert to display on count reset
        self.led = tk.IntVar(value=1)   #LEDs on by default
        self.sleep = tk.IntVar(value=0)   #Sleep is off by default
        self.default_period = tk.StringVar()       
        
        self.i2c = None
        self.i2c_lock = threading.Lock()   # serializes I2C transactions between threads
        self.running = False

        top = tk.Frame(root)
        top.pack(fill="x", padx=8, pady=8)

        self.name = tk.Label(top, text="MicroPhonon", fg="#040ed1", font=("Helvetica", 14, "bold"))
        self.name.pack(side="left", padx=10,pady=3)
        
        self.port_var = tk.StringVar()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if ports:
            self.port_var.set(ports[0])
        tk.OptionMenu(top, self.port_var, *(ports or ["(none)"])).pack(side="left")

        self.btn = tk.Button(top, text="Connect", command=self.toggle)
        self.btn.pack(side="left", padx=10,pady=3)
        
        self.name = tk.Label(top, text="I2CDriver Interface", fg="black", font=("Helvetica", 12))
        self.name.pack(side="left", padx=10,pady=3)
        
        lframe1 = ttk.LabelFrame(root, text="Streaming Data", padding=3)
        lframe1.pack(padx=10, pady=3, fill="both", expand=True)
         
        for i in range(12):
            lframe1.columnconfigure(i, weight=1)
            
        s0 = tk.Label(lframe1, textvariable=self.variable,  font=("Courier", 12),
         anchor="center", padx = 10, pady = 3, relief="sunken", bg="white", width = 40)
        s0.grid(sticky="w",column=0, columnspan=12, row=0, padx=(20,0), pady=10)

        s1 = tk.Label(lframe1, text="Quiet Count", bg="#D6D3D1", font=("Helvetica", 14))
        s1.grid(sticky="w",column=0, columnspan=1, row=1, padx=(20,0), pady=10)
        
        s2 = tk.Label(lframe1, textvariable=self.data2, font=("Courier", 12),
         anchor="center", width = 3, padx = 3, pady = 3, relief="sunken", 
         bg="white")
        s2.grid(sticky="w",column=0, columnspan=1, row=1,padx=(130,0), pady=10)
         
        s3 = tk.Label(lframe1, text="Leak Count", bg="#D6D3D1", font=("Helvetica", 14))
        s3.grid(sticky="w",column=1, columnspan=1, row=1, padx=(0,0), pady=10)
        
        s4 = tk.Label(lframe1, textvariable=self.data3, font=("Courier", 12),
         anchor="center", width = 3, padx = 3, pady = 3, relief="sunken", 
         bg="white")
        s4.grid(sticky="w",column=1, columnspan=1, row=1, padx=(105,0), pady=10)
         
        s5 = tk.Label(lframe1, text="Noise Count", bg="#D6D3D1", font=("Helvetica", 14))
        s5.grid(sticky="w",column=2, columnspan=1, row=1, padx=(20,0), pady=10)
        
        s6 = tk.Label(lframe1, textvariable=self.data4, font=("Courier", 12),
         anchor="center", width = 3, padx = 3, pady = 3, relief="sunken", bg="white")
        s6.grid(sticky="w",column=2, columnspan=1, row=1, padx=(130,0), pady=10)
        
        s12 = tk.Label(lframe1, text="Monitor Size", bg="#D6D3D1", font=("Helvetica", 14))
        s12.grid(sticky="w",column=0, columnspan=1, row=2, padx=(20,0), pady=10)
        
        s13 = tk.Label(lframe1, textvariable=self.events, font=("Courier", 12),
         anchor="center", width = 3, padx = 3, pady = 2, relief="sunken", bg="white")
        s13.grid(sticky="w",column=0, columnspan=1, row=2, padx=(130,0), pady=10)
        
        s14 = tk.Label(lframe1, text="Trigger Size", bg="#D6D3D1", font=("Helvetica", 14))
        s14.grid(sticky="w",column=1, columnspan=1, row=2, padx=(0,0), pady=10) 
        
        s15 = tk.Label(lframe1, textvariable=self.trigger, font=("Courier", 12),
         anchor="center", width = 3, padx = 3, pady = 3, relief="sunken", bg="white")
        s15.grid(sticky="w",column=1, columnspan=1, row=2, padx=(110,0), pady=10)

        s9 = tk.Label(lframe1, text="Training Size", bg="#D6D3D1", font=("Helvetica", 14))
        s9.grid(sticky="w",column=2, columnspan=1, row=2, padx=(20,0), pady=10) 
        
        s10 = tk.Label(lframe1, textvariable=self.background, font=("Courier", 12),
         anchor="center", width = 3, padx = 3, pady = 3, relief="sunken", bg="white")
        s10.grid(sticky="w",column=2, columnspan=1, row=2, padx=(135,0), pady=10)
        
        s7 = tk.Label(lframe1, text="Polling Period", bg="#D6D3D1", font=("Helvetica", 14))
        s7.grid(sticky="w",column=0, columnspan=1, row=3, padx=(20,0), pady=10)
        
        s8 = tk.Label(lframe1, textvariable=self.period, font=("Courier", 12),
         anchor="center", width = 6, padx = 3, pady = 3, relief="sunken", bg="white")
        s8.grid(sticky="w",column=0, columnspan=1, row=3, padx=(140,0), pady=10)
        
         
        lframe2 = ttk.LabelFrame(root, text="Status", padding=3)
        lframe2.pack(padx=10, pady=3, fill="both", expand=True)
        
        self.alert1 = tk.Label(lframe2, text="Not Connected", bg="white", fg="black", 
         relief="sunken", height= 2,  width = 20, font=("Helvetica", 16, "bold"))
        self.alert1.pack(padx=(20, 20), pady=20, side="left", fill="x")
         
        self.alert2 = tk.Label(lframe2, text="Not Connected", bg="white", 
         relief="sunken", height= 2, width = 20, fg="black", font=("Helvetica", 16, "bold"))
        self.alert2.pack(padx=(20, 20), pady=(20,20), side="left", fill="x")
        
        lframe3 = ttk.LabelFrame(root, text="Control Panel", padding=3)
        lframe3.pack(padx=10, pady=3, fill="both", expand=True)
        
        for i in range(12):
            lframe3.columnconfigure(i, weight=1)
            
        c9 = tk.Label(lframe3, text="Monitor Size", bg="#D6D3D1", font=("Helvetica", 14))
        c9.grid(sticky="w",column=0, columnspan=1, row=0, padx=(20,0), pady=10)
        
        mcmd = (root.register(self.validate_typing), "%P")
        self.c11 = tk.Entry(
            lframe3, 
            validate="key", 
            validatecommand=mcmd,
            width=4, 
            justify="center",
            bg="white",
            font=("Helvetica", 12) 
        )
        self.c11.grid(sticky="w",column=0, columnspan=1, row=0, padx=(140,0), pady=10, ipady=3)
        self.c11.insert(0,20)
        
        c8 = tk.Label(lframe3, text="Allowed values for Monitor Set Size: 10--255", 
         bg="#D6D3D1", font=("Helvetica", 12))
        c8.grid(sticky="w",column=0, columnspan=1, row=0, padx=(195,0), pady=10)
        
        c10 = tk.Label(lframe3, text="Trigger Size", bg="#D6D3D1", font=("Helvetica", 14))
        c10.grid(sticky="w",column=0, columnspan=1, row=1, padx=(20,0), pady=10)
        
        trcmd = (root.register(self.validate_typing), "%P")
        self.c12 = tk.Entry(
            lframe3, 
            validate="key", 
            validatecommand=trcmd,
            width=4, 
            justify="center",
            bg="white",
            font=("Helvetica", 12) 
        )
        self.c12.grid(sticky="w",column=0, columnspan=1, row=1, padx=(140,0), pady=10, ipady=3)
        self.c12.insert(0,18)
        
        c13 = tk.Label(lframe3, text="Must be <= Monitor Set Size", 
         bg="#D6D3D1", font=("Helvetica", 12))
        c13.grid(sticky="w",column=0, columnspan=1, row=1, padx=(195,0), pady=10) 
        
        c1 = tk.Label(lframe3, text="Polling Period", bg="#D6D3D1", font=("Helvetica", 14))
        c1.grid(sticky="w",column=0, columnspan=1, row=2, padx=(20,0), pady=10)
        
        self.c2 = tk.Spinbox(
            lframe3,
            values=tuple(HEXPERIODS),   # displays the strings in spinbox
            state="readonly",        # user can only pick from the list
            wrap=True,               # loops from last back to first
            width=6,
            justify="center",
            readonlybackground="white",
            textvariable=self.default_period,
            font=("Helvetica", 12)        
        )
        self.c2.grid(sticky="w",column=0, columnspan=1, row=2, padx=(140,0), pady=10, ipadx=5, ipady=3)
        self.default_period.set("2 sec") #This is the default value
        
        c3 = tk.Button(lframe3, text="Set", relief="raised", fg="white", bg="#040ed1", width=8, 
            font=("Helvetica", 14, "bold"), command=self.send_all)
        c3.grid(sticky="w",column=0, columnspan=1, row=3, padx=(20,0), pady=0)
        
        c5 = tk.Checkbutton(lframe3, text="Enable LEDs", bg="#D6D3D1", command=self.send_led, 
            variable=self.led, font=("Helvetica", 16))
        c5.grid(sticky="w",column=0, columnspan=1, row=4, padx=20, pady=10)
        
        c6 = tk.Checkbutton(lframe3, text="Sleep", bg="#D6D3D1", command=self.send_sleep,
            variable=self.sleep, font=("Helvetica", 16))
        c6.grid(sticky="w",column=0, columnspan=1, row=4, padx=(260,0), pady=10)
        
        c7 = tk.Button(lframe3, text="Reset Training", relief="raised", fg="white", bg="#008000", 
            width=12, font=("Helvetica", 14, "bold"), command=self.retrain)
        c7.grid(sticky="w",column=0, columnspan=1, row=5, padx=(20,0), pady=10)

        bcmd = (root.register(self.validate_typing), "%P")
        self.c4 = tk.Entry(
            lframe3, 
            validate="key", 
            validatecommand=bcmd,
            width=4, 
            justify="center",
            bg="white",
            font=("Helvetica", 12) 
        )
        self.c4.grid(sticky="w",column=0, columnspan=1, row=5, padx=(175,0), pady=10)
        self.c4.insert(0,30)
        
        c8 = tk.Label(lframe3, text="Allowed Training Size values: 10--255", 
         bg="#D6D3D1", font=("Helvetica", 12))
        c8.grid(sticky="w",column=0, columnspan=1, row=5, padx=(235,0), pady=10) 

        c14 = tk.Button(lframe3, text="Sensor Reset", relief="raised", fg="black", bg="#c2c2a3", 
            width=12, font=("Helvetica", 14, "bold"), command=self.reset)
        c14.grid(sticky="w",column=0, columnspan=2, row=6, padx=(20,0), pady=10)

        root.protocol("WM_DELETE_WINDOW", self.close)
        self.poll()
        
    def show_not_connected(self):
        self.alert1.config(text="Not Connected", fg="black")
        self.alert2.config(text="Not Connected", fg="black") 

    def toggle(self):
        if self.running:
            self.disconnect()
        else:
            self.connect()
            
    def validate_typing(self, numberin):
        """Allow only digits (no minus sign, no decimals)."""
        return numberin == "" or (numberin.isascii() and numberin.isdigit())

    def connected(self):
        return self.i2c is not None and self.running

    def i2c_write(self, payload):
        """Send payload to the sensor in one I2C write transaction.
        Returns True on success; shows an error dialog and returns False otherwise."""
        try:
            with self.i2c_lock:
                ack = self.i2c.start(SLAVE_ADDR, 0)
                if ack:
                    self.i2c.write(payload)
                self.i2c.stop()
        except Exception as e:
            messagebox.showerror("I2C error", str(e))
            return False
        if not ack:
            messagebox.showwarning("No ACK",
                                   f"Sensor at 0x{SLAVE_ADDR:02X} did not acknowledge. "
                                   "It may be asleep; try again.")
            return False
        print(f"Sent {len(payload)}/{len(payload)} bytes: "
              f"{' '.join(f'0x{b:02X}' for b in payload)}")
        return True
                      
    def send_all(self):
    # Set button clicked. Validate everything first so nothing is sent if any field is bad
        monitor = self.read_int(self.c11, "Monitor Size", MIN_INPUT, MAX_INPUT)
        if monitor is None:
            return
        trigger = self.read_int(self.c12, "Trigger Size", 1, MAX_INPUT)
        if trigger is None:
            return
        if trigger > monitor:
            messagebox.showerror("Out of range", "Trigger Size must be <= Monitor Size.")
            self.c12.focus_set()
            self.c12.select_range(0, tk.END)
            return

        period_code = HEXPERIODS[self.c2.get()]   # 0x01..0x09 from the spinbox

        payload = bytes([0x6F, period_code, 0x65, monitor, 0x74, trigger])
        
        if not self.connected():
            messagebox.showinfo("Not connected", "Connect to a port first.")
            return  # not connected; connect() will sync the state
        self.i2c_write(payload)
            
    def retrain(self):
        training = self.read_int(self.c4, "Training Size", MIN_INPUT, MAX_INPUT)
        if training is None:
            return
        payload = bytes([0x62, training])
        
        if not self.connected():
            messagebox.showinfo("Not connected", "Connect to a port first.")
            return  # not connected; connect() will sync the state
        if self.i2c_write(payload):
            self.pflag.set("1")
            self.counter.set(DELAY_CYCLES)
     
    
    def read_int(self, entry, name, lo, hi):
        """Return the int in an Entry, or None (after showing an error) if invalid."""
        text = entry.get().strip()
        if not text:
            messagebox.showerror("Invalid input", f"Please enter a value for {name}.")
            entry.focus_set()
            return None
        value = int(text)   # safe: validator only allows digits
        if not (lo <= value <= hi):
            messagebox.showerror("Out of range", f"{name} must be between {lo} and {hi}.")
            entry.focus_set()
            entry.select_range(0, tk.END)
            return None
        return value

            
    def send_led(self):
        cmd = LEDS_ON if self.led.get() else LEDS_OFF
        if not self.connected():
            return  # not connected; connect() will sync the state
        if self.i2c_write(cmd):
            self.pflag.set("0")
            self.counter.set(DELAY_CYCLES)
            
    def send_sleep(self):
        cmd = SLEEP_DISABLED if self.sleep.get() else SLEEP_ACTIVE
        if not self.connected():
            return  # not connected; connect() will sync the state
        self.i2c_write(cmd)
            
    def reset(self):
        if not self.connected():
            return  
        if self.i2c_write(RESET):
            self.pflag.set("1")
            self.counter.set(DELAY_CYCLES)
        
    def connect(self):
        try:
            self.i2c = I2CDriver(self.port_var.get())
            self.i2c.setspeed(I2C_SPEED_KHZ)
        except Exception as e:
            messagebox.showerror("I2CDriver error", str(e))
            self.i2c = None
            self.show_not_connected()
            return
        self.running = True
        self.btn.config(text="Disconnect")
        threading.Thread(target=self.reader, daemon=True).start()
        
    def disconnect(self):
        self.running = False
        self.btn.config(text="Connect")
        if self.i2c is not None:
            with self.i2c_lock:          # wait for any transaction in progress
                try:
                    self.i2c.ser.close()
                except Exception:
                    pass
                self.i2c = None
        self.show_not_connected()        

    def reader(self):
        """Read one 12-byte frame from the sensor every READ_PERIOD seconds and
        queue it as a space-separated line, as the Arduino used to stream it."""
        while self.running:
            frame = None
            try:
                with self.i2c_lock:
                    if self.i2c is None:
                        break
                    ack = self.i2c.start(SLAVE_ADDR, 1)
                    if ack:
                        frame = self.i2c.read(FRAME_LEN)
                    self.i2c.stop()
            except Exception:
                break
            if frame is not None and len(frame) == FRAME_LEN:
                line = " ".join(str(b) for b in frame)
                stamp = datetime.now().strftime("%H:%M:%S.%f")[:-5] 
                number_read = line.split()    # Data array
                self.q.put((stamp, line, number_read))
            time.sleep(READ_PERIOD)

    def poll(self):
        try:
            while True:
                stamp, line, number_read = self.q.get_nowait()
                self.data = number_read           # array of numbers from latest line
                self.variable.set(f"[{stamp}] {line}")
                if len(self.data) == 12:
                    d2 = self.data2.get()
                    d3 = self.data3.get()
                    d4 = self.data4.get()
                    d5 = self.data5.get()
                    d6 = self.data6.get()
                    d7 = self.data7.get()
                   # print(self.pflag.get())
                    if d2=="0" and d3=="0" and d4=="0":
                        if self.pflag.get() == "1":
                            text1, fg1 = "Sensor Training", "black"
                            lflag = 0;
                        else:
                            text1, fg1 = "Counts Resetting", "black"
                            lflag = 0;
                    elif self.data0.get() == "0":
                        text1, fg1 = "Leak Monitoring On", "#178236"
                        lflag = 0;                    
                    else: 
                        text1, fg1 = "Leak Detected", "#FB2C36" 
                        lflag = 1
                    if lflag == 1:
                        text2, fg2 = "Counts Resetting", "black"
                    elif d2=="0" and d3=="0" and d4=="0":
                        if self.pflag.get() == "1":
                            text2, fg2 = "Sampling at 1 Hz", "black"
                        else:
                            text2, fg2 = "Counts Resetting", "black"
                    elif self.data1.get() == "0":
                        text2, fg2 = "Noise Monitoring On", "#178236"
                    else: 
                        text2, fg2 = "Noise Detected", "#FB2C36"
                    if self.btn.cget("text") == "Disconnect":
                        if self.alert1.cget("text") != text1:
                            self.alert1.config(text=text1, fg=fg1)
                        if self.alert2.cget("text") != text2:
                            self.alert2.config(text=text2, fg=fg2)
                    # Let the poll run for DELAY_CYCLES. This avoids a race condition when
                    # setting the alert text with pflag
                    count_index = self.counter.get()
                    if  count_index != 0:
                        e2 = "0"
                        e3 = "0"
                        e4 = "0"
                        self.counter.set(count_index-1)
                    else:        
                        e2 = self.data2.get()
                        e3 = self.data3.get()
                        e4 = self.data4.get()
                    if (e2!="0" or e3!="0" or e4!="0") and self.pflag.get()=="1": #Counting has started
                        self.pflag.set("0")
                    
                    self.data0.set(self.data[0])
                    self.data1.set(self.data[1])
                    self.data2.set(self.data[2])
                    self.data3.set(self.data[3])
                    self.data4.set(self.data[4])
                    self.data8.set(self.data[8])
                    self.data5.set(self.data[5])
                    self.data6.set(self.data[6])
                    self.data7.set(self.data[7])

                    pindex = self.data8.get() 
                    self.period.set(Periods[pindex-1])
                    self.background.set(d5)
                    self.events.set(d6)
                    self.trigger.set(d7)     
                         
        except queue.Empty:
            pass
        self.root.after(50, self.poll)

    def close(self):
        self.disconnect()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    SerialApp(root)
    root.mainloop()
