function xNext = stateTransitionFcn_PF(x)
    % State Transition Function for a given system
    % 
    % Inputs:
    % x  - Current state vector
    % u  - Input vector
    
    % Ts - Sampling time
    Ts = 0.01;
    
    % Define the state transition matrix A
    A = [1 0 Ts 0;
         0 1 0 Ts;
         0 0 1 0;
         0 0 0 1];
     
%     A = repmat(A,)
    
    % Define the input control matrix G
    G = [Ts/2 0;
         0 Ts/2;
         1 0;
         0 1];
    
    noiseLevel = [1e-1; 1e-1; 1e-1; 1e-1]; % Adjust these values as needed for your system
    % Calculate the next state
    xNext = A*x + bsxfun(@times, noiseLevel, randn(size(x)));
end
