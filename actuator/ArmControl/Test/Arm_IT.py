import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("trajectory_log.csv")

plt.figure(figsize=(8,4.5))
plt.plot(df["t_ms"], df["pulse"])
plt.title("Cubic trajectory pulse vs time")
plt.xlabel("Time (ms)")
plt.ylabel("Servo pulse (us)")
plt.tight_layout()
plt.savefig("trajectory_plot.png", dpi=160)
print("Wrote trajectory_plot.png")
