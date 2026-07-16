function phi_sequence = test_vis(nSamples, sampleTime)
%TEST_VIS Feed VIS an ordered sequence of simulated joint measurements.
%   TEST_VIS() sends 300 four-element phi_meas vectors to VIS at 20 Hz.
%
%   TEST_VIS(NSAMPLES, SAMPLETIME) selects the number of samples and the
%   delay, in seconds, between calls.  Set SAMPLETIME to 0 to run as fast
%   as MATLAB can draw.
%
%   PHI_SEQUENCE = TEST_VIS(...) also returns the samples.  Each column is
%   one phi_meas value in time order, matching the QArm signal layout:
%       [base; shoulder; elbow; wrist]

if nargin < 1 || isempty(nSamples)
    nSamples = 300;
end
if nargin < 2 || isempty(sampleTime)
    sampleTime = 0.05;
end

validateattributes(nSamples, {'numeric'}, ...
    {'scalar', 'integer', 'positive', 'finite'}, mfilename, 'nSamples');
validateattributes(sampleTime, {'numeric'}, ...
    {'scalar', 'nonnegative', 'finite'}, mfilename, 'sampleTime');

% Make one smooth, closed trip around the torus.  VIS uses elements 2 and
% 3, but all four QArm joint measurements are supplied to exercise the
% actual signal shape used by the model.
t = linspace(0, 1, nSamples);
theta1 = 2*pi*t;
theta2 = pi/2 + (pi/3)*sin(4*pi*t);

phi_sequence = [ ...
    zeros(1, nSamples); ... % base
    theta1; ...             % shoulder (used by VIS)
    theta2; ...             % elbow (used by VIS)
    zeros(1, nSamples)];    % wrist

% Reset VIS's persistent figure and path so repeated tests start cleanly.
clear vis

for k = 1:nSamples
    vis(phi_sequence(:, k));
    if sampleTime > 0
        pause(sampleTime);
    end
end
end
