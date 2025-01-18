% Check if there are any open serial objects
existingSerial = instrfind;

if ~isempty(existingSerial)
    fclose(existingSerial);
    delete(existingSerial);
end

% Create a new serial object
serialObj = serial('COM5'); % Replace with the correct COM port
set(serialObj, 'BaudRate', 115200, 'Terminator', 'LF', 'Timeout', 10);

% Open the serial port
fopen(serialObj);

% Continuously read and display the incoming data
try
    while true
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
