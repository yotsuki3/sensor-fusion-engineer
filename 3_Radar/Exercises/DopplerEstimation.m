clc;close all;clear all
% Doppler Velocity Calculation
c = 3*10^8;         %speed of light
frequency = 77e9;   %frequency in Hz

% TODO: Calculate the wavelength
lambda = c/frequency; % m, wavelength

% TODO: Define the doppler shifts in Hz using the information from above 
fd = [3e3, -4.5e3, 11e3, -3e3]; % Hz, doppler frequency shift

% TODO: Calculate the velocity of the targets  fd = 2*vr/lambda
vel_target = fd * lambda /2; % m/s


% TODO: Display results
disp(vel_target)