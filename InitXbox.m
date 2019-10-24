function joy = InitXbox
searching = true;
time = clock;
start_time = time(6);
timeout = 30;
while searching
    time = clock;
    curr_time = time(6);
    ME = [];
    try
        joy = vrjoystick(1);
    catch ME
        disp(['Xbox controller not connected...', '  Time: ', num2str(curr_time - start_time)]);
        ME
    end
    if isempty(ME)
        searching = false;
        disp('Xbox controller found!');
    elseif curr_time - start_time > timeout
        searching = false;
        disp('Timeout: Xbox controller not found');
    else
        pause(1);
    end
end
end