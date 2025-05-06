# PCS QAM Project 2025 – 16-QAM Simulation with Pulse Shaping

This project implements a digital communication system using 16-QAM modulation with two pulse shaping methods: square pulse and sinc pulse. The system includes matched filtering, AWGN noise modeling, and performance evaluation using eye diagrams, constellation plots, and bit error rate (BER) analysis.

The goal is to study how pulse shaping and SNR impact decoding accuracy and system reliability, reflecting real-world systems used in wireless communication standards such as Wi-Fi, LTE, 5G, and satellite transmission.

## Features

- 16-QAM modulation using 4-bit symbols
- Supports both square and sinc pulse shaping
- Matched filtering on both I and Q branches
- Simulation of AWGN at SNR levels: 0 dB, 3 dB, 7 dB, and ∞ dB
- Generation of eye diagrams for the I-branch
- Constellation plots for each SNR and pulse shape
- BER vs. SNR plot with comparison between pulse shapes
- Realistic QAM modulation equation:

s(t) = I(t) * cos(2πfₐt) + Q(t) * sin(2πfₐt)


## File Descriptions

| File | Description |
|------|-------------|
| `QAM.m` | MATLAB script for full simulation |
| `Aditya Rajesh - Final Project.pdf` | Final report including results, diagrams, and analysis |
| `Final Block Diagram.png` | Final block diagram of the transmitter and receiver |
| `constellation_*.jpg/png` | Constellation plots for different SNR levels and pulse shapes |
| `eye_*.jpg/png` | Eye diagrams showing signal quality at ∞ dB and noisy conditions |
| `ber_vs_snr.jpg` | BER vs. SNR graph |
| `*.pptx` and `Final Project.pages` | Presentation and editable report drafts |

## Results Summary

- BER significantly improves as SNR increases.
- Sinc pulse performs better in terms of intersymbol interference suppression.
- Eye diagrams demonstrate clearer symbol openings at high SNR.
- Bit rate is calculated as:

Bitrate = (4 bits) / T
        = 4 / 2
        = 2 bits/sec

## How to Run

1. Open `QAM.m` in MATLAB.
2. Run the script to regenerate all diagrams and plots.
3. Outputs will be saved as `.jpg` and `.png` files in the project directory.

## Notes

- Pulse shaping plays a critical role in minimizing intersymbol interference.
- SNR = ∞ dB was implemented by omitting noise, giving a theoretical baseline.
- Constellation plots for both low and high SNR conditions illustrate the impact of noise clearly.

---

Developed by Aditya Rajesh  
Rutgers University – PCS Spring 2025
