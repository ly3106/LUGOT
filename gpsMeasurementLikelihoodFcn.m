function likelihood = gpsMeasurementLikelihoodFcn(particles, measurement)
% gpsMeasurementLikelihoodFcn Measurement likelihood function for particle filter
%
% Inputs:
%    particles   - A 4-by-NumberOfParticles matrix representing state of each particle
%    measurement - The actual measurement vector [EastPosition; NorthPosition]
%
% Outputs:
%    likelihood  - A vector with NumberOfParticles elements whose n-th
%                  element is the likelihood of the n-th particle

%#codegen

% Extract the predicted position states (EastPosition, NorthPosition) from each particle
predictedPositions = particles(1:2,:);

% Ensure the measurement is a 2x1 column vector
measurement = measurement(:);

% Replicate the measurement across all particles to match the dimensions
measurementMatrix = repmat(measurement, 1, size(predictedPositions, 2));

% Calculate the error between the actual measurement and predicted measurement
error = predictedPositions - measurementMatrix;

% Assume the measurement noise is Gaussian with zero mean and some covariance
measurementNoiseCovariance = [1 0; 0 1]; % Replace with your actual noise covariance

% Calculate the likelihood of each particle using a multivariate normal distribution
measurementNoiseCovarianceInv = inv(measurementNoiseCovariance);
detCovariance = det(measurementNoiseCovariance);
numberOfMeasurements = size(measurement, 1);
normalizationFactor = 1 / sqrt((2 * pi) ^ numberOfMeasurements * detCovariance);
exponent = sum((error' * measurementNoiseCovarianceInv) .* error', 2);
likelihood = normalizationFactor * exp(-0.5 * exponent);

end
