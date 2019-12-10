function closeFinger(s, finger, position, torque)
%closeHand Closes the hand to a given position with a chosen torque
%   Takes a serial object, position and torque. Will synchronously close
%   the fingers to this position with the set torque.

if(~exist('torque'))
    torque = 600;
end

if(torque > 700)
    disp('Caution: Running in high torque mode');
end

if(~exist('s'))
    disp('ERROR: No Serial Object Provided');
end

fprintf(s, 'a%d\n', finger);
fprintf(s, 't%d\n', torque);
fprintf(s, 'p%d\n', position);


end

