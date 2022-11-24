clc;close all;clear all
factor = 5.5;
range_max = 300; % m
delta_r = 1; % m, range resolution
c = 3*10^8; % m/s, speed of light

% TODO : Find the Bsweep of chirp for 1 m resolution
Bsweep = c/(2*delta_r);

% TODO : Calculate the chirp time based on the Radar's Max Range
sweep_time = factor * 2 * range_max / c;

% TODO : define the frequency shifts 
beat_freq = [0 1.1e6 13e6 24e6]; % given beat frequencies for all four targets
calculated_range = c*beat_freq*sweep_time/(2*Bsweep);

% Display the calculated range
disp(calculated_range);