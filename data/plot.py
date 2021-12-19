import numpy as np 
import pandas as pd 
from scipy.signal import savgol_filter
import seaborn as sns 
import matplotlib.pyplot as plt 
sns.set_theme()

circles = pd.read_csv("spike.csv", usecols=[1, 2, 3, 4, 5, 6])

# Time 0 - 300 (5 mins)
t = np.linspace(0, 20, 102)

# Current Draw - A
c = circles["current"][1:]
c_f = savgol_filter(c, 99, 3)

# Battery %
b = circles["batteryPercentage"][1:]
b_f = savgol_filter(b, 99, 3)

# Capacity % - remaining / max
r = circles["remainingCapacity"][1:] / circles["maxCapacity"][1:]

fig, axs = plt.subplots(2, figsize=(18, 25), dpi=80)
fig.suptitle("Non-Holonomic Sq Path", fontsize=30)

axs[0].plot(t, c, label= "Current Draw (A)", color="blue", linestyle="dotted", linewidth=1.5)
axs[0].plot(t, c_f, label = "Savitzky-Golay Filtered Draw (A)", color="red", linewidth=2)

axs[1].plot(t, b, label="Battery Voltage Based (%)", color="orange", linestyle="dashdot")
axs[1].plot(t, b_f, label = "Savitzky-Golay Filtered Battery (%)", color="red", linewidth=2)
axs[1].plot(t, r * 100, label="Capacity Based (%)", color="green", linestyle="dashdot", linewidth=3.0)

axs[0].set_ylabel("Current Draw (A)", fontsize=25)
axs[1].set_ylabel("Remaining Battery (%)", fontsize=25)

axs[1].set_xlabel("Time (s)", fontsize=25)

axs[0].set_xlim(0, 20)
axs[1].set_xlim(0, 20)

axs[0].set_ylim(0, 20)
axs[1].set_ylim(0, 52)

axs[0].legend(loc="upper right", prop={"size": 15}, shadow=True, fancybox=True)
axs[1].legend(loc="lower right", prop={"size": 15}, shadow=True, fancybox=True)

plt.show()
