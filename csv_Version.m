clear;
clc;

data_num = 23;

% สร้าง UDP
u = udpport("LocalPort",10001);
flush(u);

disp("UDP Ready on port 10001");
disp("กด E = เริ่มบันทึก | กด S = save CSV");

data = [];
recording = false;

while true
    
    % อ่าน keyboard
    ch = get(gcf,'CurrentCharacter');
    
    if ~isempty(ch)
        if lower(ch) == 'e'
            disp("START RECORDING...");
            recording = true;
            set(gcf,'CurrentCharacter','@'); % reset key
            
        elseif lower(ch) == 's'
            disp("SAVING CSV...");
            break;
        end
    end

    % รับ UDP
    if recording && u.NumBytesAvailable > 0
        
        raw = read(u, u.NumBytesAvailable, "string");
        disp("Received:");
        disp(raw);

        % แปลงเป็นตัวเลข
        tmp = str2num(raw); %#ok<ST2NM>

        if length(tmp) >= data_num
            data = [data; tmp(1:data_num)];
        end
    end

    pause(0.01);
end

% ปิด UDP
clear u;

% Save CSV
if ~isempty(data)
    filename = "udp_data_" + string(datetime('now','Format','yyyyMMdd_HHmmss')) + ".csv";
    writematrix(data, filename);
    
    disp("Saved file:");
    disp(filename);
else
    disp("No data to save.");
end