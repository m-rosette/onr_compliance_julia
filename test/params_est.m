r = linspace(22, 24, 100);          % Mount height 
k = linspace(1.713, 1.473, 100);    % Stiffness
x = linspace(4, 5, 100);            % Spring length
x_del = linspace(2.783, 3, 100);    % Max spring extension
x_def = x + x_del;                  % Spring length at max deflection
F = linspace(5.588, 5, 100);        % Max spring force

params = [r; k; x; x_def; F];       % idx 54,55 best fit: https://www.mcmaster.com/4992N784/
