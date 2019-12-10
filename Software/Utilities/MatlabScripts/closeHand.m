function closeHand(s, position, torque)
%closeHand Closes the hand to a given position with a chosen torque
%   Takes a serial object, position and torque. Will synchronously close
%   the fingers to this position with the set torque.
pauseTime = 0.01;

if(~exist('torque'))
    torque = 600;
end

if(torque > 700)
    disp('Caution: Running in high torque mode');
end

if(~exist('s'))
    disp('ERROR: No Serial Object Provided');
end

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

