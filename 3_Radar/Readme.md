# Radar Target Generation and Detection
## Project Overview
The goal of this project is to estimate the range and velocity of a moving object using a radar sensor. The process is as follows:
- Send a Frequency Modulated Continuous Wave (FWCW) radar signals to a moving object
- Construct beat signals based on transmitted and received signals
- Perform 2D FFT on beat signals to generate a range-doppler map
- Implement Constant False Alarm Rate (CFAR) on the range-doppler map to avoid false alarms due to clutter signals that are generally produced by the reflections from the ground, sea, buildings, trees, rain, fog etc.
<img src="img/overview.png" width="700" height="400" />

This simulation study sets the initial position = 50 m and the constant velocity = -10 m.
Based the the 2D FFT on the beat signal, the following range-doppler map is generated. The three axises are target velocity (m/s), target range (m), and signal strength (dB).
<img src="img/range-doppler-map.png" width="700" height="400" />

Due to the noise presented, the above range-doppler map may produce a false alarm. Therefore, CFAR is implemented to achive a robust estimation of the target range and velocity. The peak value successfully estimates the target range and velocity nearly equal to those set in this simulation study.
<img src="img/range-doppler-map-cfar.png" width="700" height="400" />
