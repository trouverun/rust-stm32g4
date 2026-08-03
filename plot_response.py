import matplotlib.pyplot as plt
import numpy as np

PERIOD_TICKS = 400
SAMPLE_RATE_HZ = 40_000.0

C_STIMULUS = "#3B82F6"
C_RESPONSE = "#E8710A"

words = np.array([int(w, 16) for w in open("capture_single.hex").read().split()], dtype=np.uint32)
data = words.view(np.float32).reshape(-1, 2)  # [:,0] iq_meas, [:,1] iq_target
settled = data[PERIOD_TICKS:]
t_ms = np.arange(settled.shape[0]) / SAMPLE_RATE_HZ * 1e3

spectrum_target = np.abs(np.fft.rfft(settled[:, 1]))
k = np.argmax(spectrum_target[1:]) + 1
f = k * SAMPLE_RATE_HZ / settled.shape[0]
zoom = slice(0, min(int(3 * SAMPLE_RATE_HZ / f), settled.shape[0]))

fig, axes = plt.subplots(2, 1, figsize=(12, 7))
ax = axes[0]
ax.plot(t_ms, settled[:, 1], color=C_STIMULUS, lw=1.0, label="iq target")
ax.plot(t_ms, settled[:, 0], color=C_RESPONSE, lw=1.0, label="iq measured")
ax.set_xlabel("time (ms)")
ax.set_ylabel("current (A)")
ax.set_title("300 Hz sine input response")
ax.legend(frameon=False, fontsize=8)

ax = axes[1]
ax.plot(t_ms[zoom], settled[zoom, 1], color=C_STIMULUS, lw=1.2, label="iq target")
ax.plot(t_ms[zoom], settled[zoom, 0], color=C_RESPONSE, lw=1.2, label="iq measured")
ax.set_xlabel("time (ms)")
ax.set_ylabel("current (A)")
ax.set_title("First three cycles")
ax.legend(frameon=False, fontsize=8)

fig.tight_layout()
fig.savefig("response.png", dpi=150)
plt.show()
