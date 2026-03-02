import serial
import matplotlib.pyplot as plt
import numpy as np

# ===== SET UP =====
PORT = 'COM5'
BAUDRATE = 115200
NUM_SAMPLES = 1500
Ts = 0.01   
# ==========================

ser = serial.Serial(PORT, BAUDRATE, timeout=1)

setpoint = []
velocity = []
pwm = []

print("Starting...")

while len(setpoint) < NUM_SAMPLES:
    line = ser.readline().decode(errors='ignore').strip()
    
    if line.startswith("DATA"):
        line = line.replace("DATA,", "")
        values = line.split(',')

        if len(values) == 3:
            try:
                sp = float(values[0])
                vel = float(values[1])
                u = float(values[2])

                setpoint.append(sp)
                velocity.append(vel)
                pwm.append(u/1023*100)

                print(f"Sample #{len(setpoint)}")

            except ValueError:
                pass

ser.close()

print("Ploting...")

# ===== VECTOR DE TIEMPO =====
time = np.arange(0, len(setpoint) * Ts, Ts)

# ===== GRAFICAR =====
plt.figure()

plt.plot(time, setpoint, label="Setpoint")
plt.plot(time, velocity, label="Speed")
#plt.plot(time, pwm, label="PWM")

plt.xlabel("Time [s]")
plt.ylabel("Value")
plt.legend()
plt.grid(True)

plt.show()