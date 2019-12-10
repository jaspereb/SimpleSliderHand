function grasp(s,torque,degrees)
%closeHand Closes the hand to a given position with a chosen torque
%   Takes a serial object, position and torque. Will synchronously close
%   the fingers to this position with the set torque.
pauseTime = 0.01;
position = 2000;

if(~exist('torque'))
    torque = 600;
end

if(torque > 700)
    disp('Caution: Running in high torque mode');
end

if(~exist('s'))
    disp('ERROR: No Serial Object Provided');
end

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

fprintf(s, 'a1');
pause(pauseTime);
fprintf(s, 't600');
pause(pauseTime);
fprintf(s, 'p%d\n', rotation);
pause(pauseTime);

pause(0.5);

fprintf(s, 'a2');
pause(pauseTime);
fprintf(s, 't%d\n', torque);
pause(pauseTime);
fprintf(s, 'a3');
pause(pauseTime);
fprintf(s, 't%d\n', torque);
pause(pauseTime);
fprintf(s, 'a4');
pause(pauseTime);
fprintf(s, 't%d\n', torque);
pause(pauseTime);

fprintf(s, 'a2');
pause(pauseTime);
fprintf(s, 'p%d\n', position);
pause(pauseTime);

fprintf(s, 'a3');
pause(pauseTime);
fprintf(s, 'p%d\n', position);
pause(pauseTime);

fprintf(s, 'a4');
pause(pauseTime);
fprintf(s, 'p%d\n', position);
pause(pauseTime);


end



