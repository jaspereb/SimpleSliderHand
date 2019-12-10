function rotateHand(s, degrees, torque)
%rotateHand Rotates the hand to a given position with a chosen torque
%   Takes a serial object, position and torque. 

rotation = degrees*16.11;
rotation = round(rotation);

if(degrees > 88)
    rotation = 1500;
    disp('Maximum Rotation Set');
end

% Limits on output
if(rotation > 1500)
    rotation = 1500;
elseif(rotation < 0)
    rotation = 0;
end

if(~exist('torque'))
    torque = 600;
end

if(torque > 700)
    disp('Caution: Running in high torque mode');
end

if(~exist('s'))
    disp('ERROR: No Serial Object Provided');
end

fprintf(s, 'a1');
fprintf(s, 't%d\n', torque);
pause(1);
fprintf(s, 'p%d\n', rotation);
pause(1);

end

