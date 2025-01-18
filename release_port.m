% Check if there are any open serial objects
existingSerial = instrfind;

if ~isempty(existingSerial)
    fclose(existingSerial); % Close any open serial objects
    delete(existingSerial);  % Delete them
end

% Create a new serial object
serialObj = serial('COM5'); % Replace with the correct COM port
set(serialObj, 'BaudRate', 115200, 'Terminator', 'LF', 'Timeout', 10);

% Open the serial port
fopen(serialObj);

% Continuously read and display the incoming data
try
    disp('Reading data from Arduino...');
    startTime = tic; % Start timer to limit reading time
    while toc(startTime) < 0.25
        % Read for 5 seconds (adjust as necessary)
        if serialObj.BytesAvailable > 0
            data = fscanf(serialObj, '%s'); % Read the serial data as string
            disp(data); % Display the received data
        end
    end
catch ME
    % Handle any errors that occur
    disp('Error occurred during serial communication');
    disp(ME.message);
end

% Close the serial port when done
fclose(serialObj);
delete(serialObj);
clear serialObj;
disp('Serial port released.');
