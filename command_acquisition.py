from serial import Serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
from threading import Thread
import csv
from os.path import isfile
import time

data_len = 5000
num_plots = 10

deque_list = [deque(maxlen=data_len) for _ in range(num_plots + 1)]
x, PT_01, PT_02, PT_E1, PT_E2, PT_C1, LC1, LC2, LC3, TC1, TC2  = deque_list

plot_titles = ["PT_01", "PT_02", "PT_E1", "PT_E2", "PT_C1", "LC1", "LC2", "LC3", "TC1", "TC2"]

fig, axs = plt.subplots(2, 5) # plot arrangement

axs_list = [axs[0,0], axs[0,1], axs[0,2], axs[0,3], axs[0,4],
            axs[1,0], axs[1,1], axs[1,2], axs[1,3], axs[1,4]]

lines = [ax.plot([], [])[0] for ax in axs_list]

# for ax, title in zip(axs_list, plot_titles):
#     ax.set_title(title)


file_base = f"serialplot_{time.strftime('%Y-%m-%d', time.gmtime())}"
file_ext = ".csv"
test_num = 1
while isfile(file_base + f"_test{test_num}" + file_ext):
    test_num += 1
filename = file_base + f"_test{test_num}" + file_ext

port_num = "/dev/cu.usbserial-0001"
esp32 = Serial(port=port_num, baudrate=115200, timeout= 0.1)

def collection():

    with open(filename, "a", newline='') as f:
        writer = csv.writer(f, delimiter= ",")
        while True:
            data = esp32.readline()
            try:
                decoded_bytes = data[:len(data)-2].decode("utf-8")
                values = decoded_bytes.split(" ")

                time_ms = time.time_ns() // (10**6)
                millisecs = str(time_ms % (10**3)).zfill(3)
                writer.writerow([time.strftime("%H:%M:%S", time.gmtime(time_ms // (10**3))) + ":" + millisecs] + values)

                if len(values) == 14:
                    x.append(float(values[0])/1000)
                    PT_01.append(float(values[1]))
                    PT_02.append(float(values[2]))
                    PT_E1.append(float(values[3]))
                    PT_E2.append(float(values[4]))
                    PT_C1.append(float(values[5]))
                    LC1.append(float(values[6]) + float(values[7]) + float(values[8]))
                    LC2.append(float(values[7]))
                    LC3.append(float(values[8]))
                    TC1.append(float(values[9]))
                    TC2.append(float(values[10]))
            
            except:
                continue

t1 = Thread(target=collection)
t1.start()


def animate(i):
    fig.suptitle(f"TIME ELAPSED {list(x)[-1]}s", fontsize=12)

    for j, ax in enumerate(axs_list):
        line = lines[j]
        line.set_data(x, deque_list[j+1])
        ax.set_title(f"{plot_titles[j]}: {deque_list[j+1][-1]:.2f}", fontsize=12)
        ax.relim()
        ax.autoscale_view()
    return lines
    
ani = FuncAnimation(fig, animate, interval=10, cache_frame_data=False)
plt.show()