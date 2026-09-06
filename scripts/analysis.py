import glob
import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import make_interp_spline

TICKS = 400
INTERP_ORDER = 2
HARMONICS = np.array([1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27])
WORDS_PER_RECORD = 8

C_STIMULUS = "#3B82F6"
C_RESPONSE = "#E8710A"
C_NEUTRAL = "#9AA0A6"

path = sys.argv[1] if len(sys.argv) > 1 else max(glob.glob("capture_*.hex"))
fs = float(sys.argv[2]) if len(sys.argv) > 2 else 40_000.0
fundamental_hz = fs / TICKS
f = HARMONICS * fundamental_hz

words = np.array([int(w, 16) for w in open(path).read().split()], dtype=np.uint16).view(np.int16)
records = words[: len(words) // WORDS_PER_RECORD * WORDS_PER_RECORD].reshape(-1, WORDS_PER_RECORD)
iq_meas = records[:, 1] / 1000.0
iq_target = records[:, 3] / 1000.0

# drop the first period as settling, coherently average the remaining whole periods
periods = (len(records) - TICKS) // TICKS
sl = slice(TICKS, TICKS + periods * TICKS)
meas = iq_meas[sl].reshape(periods, TICKS).mean(axis=0)
target = iq_target[sl].reshape(periods, TICKS).mean(axis=0)

M, T = np.fft.rfft(meas), np.fft.rfft(target)
H = M[HARMONICS] / T[HARMONICS]
mag_db = 20 * np.log10(np.abs(H))

for fi, db_i, Hi in zip(f, mag_db, H):
    print(f"{fi:6.0f} Hz  {db_i:+6.2f} dB  {np.degrees(np.angle(Hi)):+7.1f} deg")

# interpolate the gain curve between excitation lines with a spline on the log-frequency axis
logf = np.log10(f)
gain_spline = make_interp_spline(logf, mag_db, k=INTERP_ORDER)
logf_dense = np.linspace(logf[0], logf[-1], 20_000)
mag_db_dense = gain_spline(logf_dense)

below = np.nonzero(mag_db_dense < -3.0)[0]
if below.size and below[0] > 0:
    i = below[0]
    logf_bw = np.interp(-3.0, [mag_db_dense[i], mag_db_dense[i - 1]], [logf_dense[i], logf_dense[i - 1]])
    bandwidth_hz = 10 ** logf_bw
    print(f"-3 dB bandwidth: {bandwidth_hz:.0f} Hz (spline order k={INTERP_ORDER})")
else:
    bandwidth_hz = None
    print("-3 dB bandwidth: not crossed within excited lines")

fig, axes = plt.subplots(3, 1, figsize=(12, 11))
t_ms = np.arange(TICKS) / fs * 1e3

# stimulus rebuilt from the bias and the excited bins only: overlap = DFT amplitudes are valid
spectrum_only = np.zeros_like(T)
spectrum_only[0] = T[0]
spectrum_only[HARMONICS] = T[HARMONICS]
rebuilt = np.fft.irfft(spectrum_only, TICKS)

ax = axes[0]
ax.plot(t_ms, target, color=C_STIMULUS, lw=1.2, label="iq target")
ax.plot(t_ms, meas, color=C_RESPONSE, lw=1.2, label="iq measured")
ax.plot(t_ms, rebuilt, color="black", lw=0.8, ls="--", label="target reconstructed from excitation lines")
ax.set_xlabel("time (ms)")
ax.set_ylabel("current (A)")
ax.set_title(f"Multisine stimulus and response, one excitation period (coherent average of {periods})")
ax.legend(frameon=False, fontsize=8)

db = lambda x: 20 * np.log10(np.maximum(np.abs(x), 1e-12))
freqs = np.arange(len(T)) * fundamental_hz
ax = axes[1]
ax.plot(freqs[1:], db(T)[1:], color=C_STIMULUS, lw=0.8, label="iq target")
ax.plot(freqs[1:], db(M)[1:], color=C_RESPONSE, lw=0.8, label="iq measured")
ax.plot(f, db(T[HARMONICS]), "o", color=C_STIMULUS, ms=4)
ax.plot(f, db(M[HARMONICS]), "o", color=C_RESPONSE, ms=4)
ax.set_xscale("log")
ax.set_xlim(fundamental_hz * 0.9, f[-1] * 1.1)
ax.set_ylim(bottom=db(T[HARMONICS]).max() - 80)
ax.set_xlabel("frequency (Hz)")
ax.set_ylabel("amplitude (dB referenced to 1 A)")
ax.set_title("Amplitude spectrum from DFT, markers = multisine excitation frequencies")
ax.legend(frameon=False, fontsize=8)

ax = axes[2]
ax.plot(10 ** logf_dense, mag_db_dense, "-", color=C_RESPONSE, lw=1.4, label=f"spline interpolation (k={INTERP_ORDER})")
ax.plot(f, mag_db, "o", color=C_RESPONSE, ms=5, label="measured at excitation lines")
ax.axhline(-3.0, color=C_NEUTRAL, lw=0.8, ls="--")
ax.legend(frameon=False, fontsize=8)
if bandwidth_hz is not None:
    ax.axvline(bandwidth_hz, color=C_NEUTRAL, lw=0.8, ls="--")
    ax.annotate(f"{bandwidth_hz:.0f} Hz", (bandwidth_hz, -3.0),
                textcoords="offset points", xytext=(5, 5), fontsize=8)
ax.set_xscale("log")
ax.set_xlim(fundamental_hz * 0.9, f[-1] * 1.1)
ax.set_xlabel("frequency (Hz)")
ax.set_ylabel("gain (dB)")
ax.set_title("Closed-loop gain (interpolated between excitation frequencies)")

fig.tight_layout()
fig.savefig("bandwidth.png", dpi=150)
plt.show()
