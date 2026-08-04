import matplotlib.pyplot as plt
import numpy as np

CAPTURE_TICKS = 6400
PERIOD_TICKS = 400
SAMPLE_RATE_HZ = 40_000.0
FUNDAMENTAL_HZ = SAMPLE_RATE_HZ / PERIOD_TICKS
h = np.array([1, 3, 5, 7, 9, 11, 13, 15, 19, 23, 27, 31, 35, 39])
f = h * FUNDAMENTAL_HZ

C_STIMULUS = "#3B82F6"
C_RESPONSE = "#E8710A"
C_NEUTRAL = "#9AA0A6"

words = np.array([int(w, 16) for w in open("capture.hex").read().split()], dtype=np.uint32)
data = words.view(np.float32).reshape(-1, 2)  # [:,0] iq_meas, [:,1] iq_target
avg = data[PERIOD_TICKS:].reshape(-1, PERIOD_TICKS, 2).mean(axis=0)  # drop settling period

M, T = np.fft.rfft(avg[:, 0]), np.fft.rfft(avg[:, 1])
H = M[h] / T[h]
mag_db = 20 * np.log10(np.abs(H))

for fi, db_i, Hi in zip(f, mag_db, H):
    print(f"{fi:6.0f} Hz  {db_i:+6.2f} dB  {np.degrees(np.angle(Hi)):+7.1f} deg")

below = np.nonzero(mag_db < -3.0)[0]
if below.size and below[0] > 0:
    i = below[0]
    logf = np.interp(-3.0, [mag_db[i], mag_db[i - 1]], np.log10([f[i], f[i - 1]]))
    bandwidth_hz = 10 ** logf
    print(f"-3 dB bandwidth: {bandwidth_hz:.0f} Hz")
else:
    bandwidth_hz = None
    print("-3 dB bandwidth: not crossed within excited lines")

fig, axes = plt.subplots(3, 1, figsize=(12, 11))
t_ms = np.arange(PERIOD_TICKS) / SAMPLE_RATE_HZ * 1e3

# stimulus rebuilt from only the excited bins: overlap = DFT amplitudes are valid
spectrum_only = np.zeros_like(T)
spectrum_only[h] = T[h]
rebuilt = np.fft.irfft(spectrum_only, PERIOD_TICKS)

ax = axes[0]
ax.plot(t_ms, avg[:, 1], color=C_STIMULUS, lw=1.2, label="iq target")
ax.plot(t_ms, avg[:, 0], color=C_RESPONSE, lw=1.2, label="iq measured")
ax.plot(t_ms, rebuilt, color="black", lw=0.8, ls="--", label="target reconstructed from excitation lines")
ax.set_xlabel("time (ms)")
ax.set_ylabel("current (A)")
ax.set_title("Multisine stimulus and response, one excitation period (coherent average of 15)")
ax.legend(frameon=False, fontsize=8)

db = lambda x: 20 * np.log10(np.maximum(np.abs(x), 1e-12))
freqs = np.arange(len(T)) * FUNDAMENTAL_HZ
ax = axes[1]
ax.plot(freqs[1:], db(T)[1:], color=C_STIMULUS, lw=0.8, label="iq target")
ax.plot(freqs[1:], db(M)[1:], color=C_RESPONSE, lw=0.8, label="iq measured")
ax.plot(f, db(T[h]), "o", color=C_STIMULUS, ms=4)
ax.plot(f, db(M[h]), "o", color=C_RESPONSE, ms=4)
ax.set_xscale("log")
ax.set_xlim(FUNDAMENTAL_HZ * 0.9, f[-1] * 1.1)
ax.set_ylim(bottom=db(T[h]).max() - 80)
ax.set_xlabel("frequency (Hz)")
ax.set_ylabel("amplitude (dB referenced to 1 A)")
ax.set_title("Amplitude spectrum from DFT, markers = multisine excitation frequencies")
ax.legend(frameon=False, fontsize=8)

ax = axes[2]
ax.plot(f, mag_db, "o-", color=C_RESPONSE, lw=1.4, ms=5)
ax.axhline(-3.0, color=C_NEUTRAL, lw=0.8, ls="--")
if bandwidth_hz is not None:
    ax.axvline(bandwidth_hz, color=C_NEUTRAL, lw=0.8, ls="--")
    ax.annotate(f"{bandwidth_hz:.0f} Hz", (bandwidth_hz, -3.0),
                textcoords="offset points", xytext=(5, 5), fontsize=8)
ax.set_xscale("log")
ax.set_xlim(FUNDAMENTAL_HZ * 0.9, f[-1] * 1.1)
ax.set_xlabel("frequency (Hz)")
ax.set_ylabel("gain (dB)")
ax.set_title("Closed-loop gain (interpolated between excitation frequencies)")

fig.tight_layout()
fig.savefig("bandwidth.png", dpi=150)
plt.show()
