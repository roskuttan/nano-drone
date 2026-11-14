"""make_audio.py - procedural soundtrack for the product video (no samples, no external tools).

    blender -b --factory-startup -P hw/mech/blender/make_audio.py

Writes hw/output/mech/blender/nanodrone_product_audio.wav (48 kHz stereo, 16-bit). Layers:
  music  : D minor pad, bass pulse, plucked arpeggio, kick and hats at 100 BPM; builds after the title card,
           drops to pad only on the spec card and fades on the outro
  drone  : motor spin-up at the reveal, a quiet prop hum under the shots, a rising whine at lift-off
  cuts   : a short whoosh on every cut and card
The cut times come from edit_product_video.py (CARD_INTRO + shot boundaries), so change both together.
"""
import math
import os
import wave

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.normpath(os.path.join(HERE, "..", "..")), "output", "mech", "blender",
                   "nanodrone_product_audio.wav")
SR = 48000
FPS = 24
CARD_INTRO = 54
SHOT_STARTS = [1, 121, 265, 385, 505, 601]          # product_video.py
N3D, CARD_SPECS, CARD_OUTRO = 744, 96, 72
CUTS = [(CARD_INTRO + s) / FPS for s in SHOT_STARTS[1:]]
T_SHOTS = (CARD_INTRO + 1) / FPS
T_SPECS = (CARD_INTRO + N3D + 1) / FPS
T_OUTRO = T_SPECS + CARD_SPECS / FPS
T_END = T_OUTRO + CARD_OUTRO / FPS
T_LIFT = (CARD_INTRO + 640) / FPS
BPM = 100.0
BEAT = 60.0 / BPM
rng = np.random.default_rng(7)

N = int(T_END * SR) + SR
t = np.arange(N) / SR
L = np.zeros(N)
R = np.zeros(N)


def midi(n):
    return 440.0 * 2 ** ((n - 69) / 12.0)


def env(n_samples, a, d, s, r, hold):
    """ADSR in seconds; hold = time at sustain level."""
    out = np.zeros(n_samples)
    i = 0
    for seg_len, v0, v1 in ((a, 0.0, 1.0), (d, 1.0, s), (hold, s, s), (r, s, 0.0)):
        k = int(seg_len * SR)
        k = min(k, n_samples - i)
        if k > 0:
            out[i:i + k] = np.linspace(v0, v1, k)
            i += k
    return out


def add(sig, t0, gain=1.0, pan=0.0):
    i = int(t0 * SR)
    k = min(len(sig), N - i)
    if k <= 0:
        return
    L[i:i + k] += sig[:k] * gain * (1.0 - max(pan, 0.0))
    R[i:i + k] += sig[:k] * gain * (1.0 + min(pan, 0.0))


def lowpass(x, cutoff):
    a = math.exp(-2 * math.pi * cutoff / SR)
    y = np.empty_like(x)
    acc = 0.0
    for i in range(len(x)):          # one-pole, fine for a few seconds of noise
        acc = a * acc + (1 - a) * x[i]
        y[i] = acc
    return y


def tone(freq, dur, harmonics=((1, 1.0), (2, 0.35), (3, 0.15)), detune=0.0):
    tt = np.arange(int(dur * SR)) / SR
    s = np.zeros(len(tt))
    for h, amp in harmonics:
        s += amp * np.sin(2 * math.pi * freq * h * (1 + detune) * tt)
    return s


# ------------------------------------------------------------------ music
# D minor: Dm (D F A), Bb (Bb D F), F (F A C), C (C E G); two bars each, repeated to fill
CHORDS = [(62, 65, 69, 74), (58, 62, 65, 70), (65, 69, 72, 77), (60, 64, 67, 72)]
BAR = 4 * BEAT
chord_len = 2 * BAR
music_start = 0.0
n_chords = int(math.ceil((T_END - music_start) / chord_len))
for ci in range(n_chords):
    t0 = music_start + ci * chord_len
    notes = CHORDS[ci % 4]
    e = env(int(chord_len * SR) + SR, 0.8, 0.5, 0.8, 1.2, chord_len - 1.3)
    for j, n in enumerate(notes):
        f = midi(n)
        pad_l = tone(f, chord_len + 1.0, ((1, 1.0), (2, 0.25), (3, 0.08), (4, 0.04)), detune=0.0015) * e
        pad_r = tone(f, chord_len + 1.0, ((1, 1.0), (2, 0.25), (3, 0.08), (4, 0.04)), detune=-0.0015) * e
        g = 0.045
        i = int(t0 * SR)
        k = min(len(pad_l), N - i)
        L[i:i + k] += pad_l[:k] * g
        R[i:i + k] += pad_r[:k] * g
    # bass: root an octave down, pulsing on eighth notes once the shots start
    root = midi(notes[0] - 24)
    if t0 + chord_len > T_SHOTS and t0 < T_SPECS:
        nb = int(chord_len / (BEAT / 2))
        for b in range(nb):
            tb = t0 + b * BEAT / 2
            if tb < T_SHOTS - 0.1 or tb > T_SPECS - 0.5:
                continue
            s = tone(root, BEAT / 2, ((1, 1.0), (2, 0.5), (3, 0.12))) * env(int(BEAT / 2 * SR), 0.01, 0.12, 0.55, 0.08, BEAT / 2 - 0.21)
            add(s, tb, 0.16)
    # arpeggio: sixteenth notes over the chord, quiet, panned around
    if t0 + chord_len > T_SHOTS + 4.8 and t0 < T_SPECS:
        order = [notes[0] + 12, notes[1] + 12, notes[2] + 12, notes[3] + 12, notes[2] + 12, notes[1] + 12]
        na = int(chord_len / (BEAT / 4))
        for a_i in range(na):
            ta = t0 + a_i * BEAT / 4
            if ta < T_SHOTS + 4.8 or ta > T_SPECS - 0.5:
                continue
            f = midi(order[a_i % len(order)])
            s = tone(f, 0.35, ((1, 1.0), (2, 0.3), (3, 0.1))) * np.exp(-np.arange(int(0.35 * SR)) / SR * 9.0)
            add(s, ta, 0.05, pan=0.5 * math.sin(a_i * 0.7))

# kick and hats: from the first cut to the spec card, with a one-bar fill before the lift-off
kick_tt = np.arange(int(0.35 * SR)) / SR
kick = np.sin(2 * math.pi * (45 * kick_tt + (150 - 45) * 0.06 * (1 - np.exp(-kick_tt / 0.06)))) * np.exp(-kick_tt / 0.11)
hat_raw = rng.standard_normal(int(0.06 * SR))
hat = (hat_raw - lowpass(hat_raw, 6000.0)) * np.exp(-np.arange(len(hat_raw)) / SR * 70.0)
tb = CUTS[0]
beat_i = 0
while tb < T_SPECS - 0.3:
    add(kick, tb, 0.42)
    add(hat, tb + BEAT / 2, 0.10)
    if beat_i % 4 == 3:
        add(hat, tb + BEAT * 0.75, 0.06)
    tb += BEAT
    beat_i += 1

# ------------------------------------------------------------------ drone sounds
def motor(t_start, dur, f0, f1, gain, curve=1.0):
    """Brushless whine: fundamental sweeping f0 -> f1 with a few harmonics and a touch of noise."""
    tt = np.arange(int(dur * SR)) / SR
    frac = (tt / dur) ** curve
    f = f0 + (f1 - f0) * frac
    phase = 2 * math.pi * np.cumsum(f) / SR
    s = np.sin(phase) + 0.45 * np.sin(2 * phase) + 0.2 * np.sin(3 * phase) + 0.08 * np.sin(5 * phase)
    s += 0.25 * lowpass(rng.standard_normal(len(tt)), 1800.0)
    return s * gain


spin = motor(T_SHOTS, 3.0, 60.0, 520.0, 1.0, curve=0.6) * env(int(3.0 * SR), 0.6, 0.0, 1.0, 0.3, 2.1)
add(spin, T_SHOTS, 0.16)
hum = motor(T_SHOTS + 3.0, T_SPECS - T_SHOTS - 3.0 + 0.5, 520.0, 540.0, 1.0)
hum *= env(len(hum), 0.3, 0.0, 1.0, 0.8, len(hum) / SR - 1.1)
add(hum, T_SHOTS + 3.0, 0.045)
lift = motor(T_LIFT, T_SPECS - T_LIFT, 540.0, 760.0, 1.0, curve=0.8) * env(int((T_SPECS - T_LIFT) * SR), 0.5, 0.0, 1.0, 0.6, T_SPECS - T_LIFT - 1.1)
add(lift, T_LIFT, 0.09)

# ------------------------------------------------------------------ whooshes on the cuts
for tc in CUTS + [T_SPECS, T_OUTRO]:
    d = 0.45
    noise = lowpass(rng.standard_normal(int(d * SR)), 2200.0)
    sweep = env(len(noise), 0.18, 0.0, 1.0, 0.25, 0.02)
    add(noise * sweep, tc - 0.22, 0.22, pan=0.0)

# ------------------------------------------------------------------ master: fade out, soft clip, write
fade = np.ones(N)
k0 = int((T_END - 2.0) * SR)
fade[k0:] = np.linspace(1.0, 0.0, N - k0)
mix = np.stack([L, R], axis=1) * fade[:, None]
mix = np.tanh(mix * 1.6) * 0.85
mix = mix[: int(T_END * SR)]
data = (mix * 32767).astype("<i2")
with wave.open(OUT, "wb") as w:
    w.setnchannels(2)
    w.setsampwidth(2)
    w.setframerate(SR)
    w.writeframes(data.tobytes())
print("wrote %s: %.1f s, peak %.2f, rms %.3f" % (OUT, len(mix) / SR, np.abs(mix).max(), np.sqrt((mix ** 2).mean())))
