
data = readtable('C:\Users\xyz\Desktop\exflap_data.xlsx');
% Assuming input variables are in columns 1, 5, and 6, and output variables in columns 2, 3, and 4
% Use the correct column names from the dataset
% Update the column names based on the actual names
inputs = data{:, {'Servo_Angle', 'Frequency'}};
outputs = data{:, {'Roll', 'Pitch', 'Yaw','Attitude'}};

% Define time instants as a sequence
time_stamps = (1:height(data))';  % or replace with actual time if available

% Create iddata object with time instants for irregular sampling
id_data = iddata(outputs, inputs, [], 'SamplingInstants', time_stamps);

% Estimate state-space model
model_order = 4;
m_sys = n4sid(id_data, model_order);

% Validate the model
compare(id_data, m_sys);

% Extract the state-space matrices
[A, B, C, D] = ssdata(m_sys);

% Display the matrices
disp('A matrix:'); disp(A);
disp('B matrix:'); disp(B);
disp('C matrix:'); disp(C);
disp('D matrix:'); disp(D);