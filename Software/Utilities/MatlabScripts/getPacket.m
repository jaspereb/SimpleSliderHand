function [status] = getPacket(s, servo)
%getPacket Asks for and parses a single hand data packet
%   Takes a serial object and returns [position, speed, load, voltage,
%   temperature]

if(~exist('s'))
    disp('ERROR: No Serial Object Provided');
end

fprintf(s, 'a%d\n', servo);
fprintf(s, 'i');
dataPacket = fscanf(s);
% disp(dataPacket);

dataPacket = strsplit(dataPacket, ',');

status.position = str2double(dataPacket(1));
status.speed = str2double(dataPacket(2));
status.load = str2double(dataPacket(3));
status.volts = str2double(dataPacket(4));
status.temperature = str2double(dataPacket(5));
status.time = now;
disp(status);

end

