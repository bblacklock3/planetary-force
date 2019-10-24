function a = InitArduino(custom_port)
searching = true;
time = clock;
start_time = time(6);
timeout = 30;
while searching
    time = clock;
    curr_time = time(6);
    ME = [];
    try
        if exist("custom_port",'var')==0
            port = 'COM5';
        else
            port = custom_port;
        end
        a = arduino(port,'Uno','Libraries','Servo');
    catch ME
        disp(['Arduino not connected...', '  Time: ', num2str(curr_time - start_time)]);
        ME
    end
    if isempty(ME)
        searching = false;
        disp('Arduino found!');
    elseif curr_time - start_time > timeout
        searching = false;
        disp('Timeout: Arduino not found');
    else
        pause(1);
    end
end
end