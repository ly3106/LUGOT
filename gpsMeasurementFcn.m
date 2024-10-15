function y = gpsMeasurementFcn(x)
% gpsMeasurementFcn GPS measurement function for state estimation
%
% Assume the states x are:
%   [EastPosition; NorthPosition; EastVelocity; NorthVelocity]

%#codegen

% The %#codegen tag above is needed is you would like to use MATLAB Coder to 
% generate C or C++ code for your filter

y = x([1 2]); % Position states are measured
end