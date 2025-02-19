% Create a new fuzzy inference system
fis = newfis('Test_Fuzzy', 'mamdani');

% Define input variable
fis = addvar(fis, 'input', 'input1', [-85 85]);

% Add membership functions for input variable
fis = addmf(fis, 'input', 1, 'PVDP', 'trapmf', [43 53 85 85]);
fis = addmf(fis, 'input', 1, 'PLDP', 'trapmf', [11 21 43 53]);
fis = addmf(fis, 'input', 1, 'PVS', 'trapmf', [-21 -11 11 21]);
fis = addmf(fis, 'input', 1, 'PLDN', 'trapmf', [-53 -43 -21 -11]);
fis = addmf(fis, 'input', 1, 'PVDN', 'trapmf', [-85 -85 -53 -43]);

% Define output variable
fis = addvar(fis, 'output', 'output1', [-400 400]);

% Add membership functions for output variable
fis = addmf(fis, 'output', 1, 'OPFU', 'trapmf', [220 260 400 400]);
fis = addmf(fis, 'output', 1, 'OPLU', 'trapmf', [60 100 220 260]);
fis = addmf(fis, 'output', 1, 'ZDEG', 'trapmf', [-100 -60 60 100]);
fis = addmf(fis, 'output', 1, 'OPLD', 'trapmf', [-260 -220 -100 -60]);
fis = addmf(fis, 'output', 1, 'OPFD', 'trapmf', [-400 -400 -260 -220]);

% Set fuzzy logic operators
fis.andMethod = 'min';
fis.orMethod = 'max';
fis.impMethod = 'min';
fis.aggMethod = 'max';
fis.defuzzMethod = 'centroid';

% Define fuzzy rules
ruleList = [
    1 1 1 1; % If input is PVDP, then output is OPFU
    2 2 1 1; % If input is PLDP, then output is OPLU
    3 3 1 1; % If input is PVS, then output is ZDEG
    4 4 1 1; % If input is PLDN, then output is OPLD
    5 5 1 1  % If input is PVDN, then output is OPFD
];
fis = addrule(fis, ruleList);

% Initialize input and output arrays
inputRange = -90:1:90; % Input values from -90 to 90
outputValues = zeros(size(inputRange)); % Placeholder for output values

% Evaluate the fuzzy controller
for i = 1:length(inputRange)
    outputValues(i) = evalfis(fis, inputRange(i));
end

% Combine input and output data into one matrix
data = [inputRange' outputValues'];

% Save to CSV file
csvFileName = 'fuzzy_controller_results.csv';
writematrix(data, csvFileName);

disp(['Fuzzy controller results saved to ', csvFileName]);

% Plot input-output relationship
figure;
plot(inputRange, outputValues, 'LineWidth', 2);
grid on;
xlabel('Input');
ylabel('Output');
title('Fuzzy Controller Input-Output Relationship');