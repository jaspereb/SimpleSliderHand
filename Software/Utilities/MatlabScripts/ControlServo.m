%This program connects to the openCM9 over serial and moves it around to
%demonstrate that it is connected

if(~exist('s'))
    s = serial('ACM0');
    fopen(s);
    disp('Opening Serial');
end


fprintf(s, 'a1');
fprintf(s, 'p0');
pause(2);
fprintf(s, 'p1600');
pause(2);

closeTo = 1600;

disp('Closing Hand');
closeHand(s, closeTo);
pause(1);
disp('Opening Hand');
closeHand(s, 0);
disp('Initialised');

% %Do some stuff
% fprintf(s, 'a2');
% fprintf(s, 'p%d\n', closeTo);
% 
% fprintf(s, 'a3');
% fprintf(s, 'p%d\n', closeTo);
% 
% fprintf(s, 'a4');
% fprintf(s, 'p%d\n', closeTo);
% 
% pause(5);
% 
% fprintf(s, 'a2');
% fprintf(s, 'p0');
% 
% fprintf(s, 'a3');
% fprintf(s, 'p0');
% 
% fprintf(s, 'a4');
% fprintf(s, 'p0');
% 
=======
%This program connects to the openCM9 over serial and moves it around to
%demonstrate that it is connected

if(~exist('s'))
    % s = serial('COM9');
    s = serial('/dev/ttyACM0');
    
    fopen(s);
    disp('Opening Serial');
end


fprintf(s, 'a1');
fprintf(s, 'p0');
pause(2);
fprintf(s, 'p1600');
pause(2);

closeTo = 1600;

disp('Closing Hand');
closeHand(s, closeTo);
pause(1);
disp('Opening Hand');
closeHand(s, 0);

% %Do some stuff
% fprintf(s, 'a2');
% fprintf(s, 'p%d\n', closeTo);
% 
% fprintf(s, 'a3');
% fprintf(s, 'p%d\n', closeTo);
% 
% fprintf(s, 'a4');
% fprintf(s, 'p%d\n', closeTo);
% 
% pause(5);
% 
% fprintf(s, 'a2');
% fprintf(s, 'p0');
% 
% fprintf(s, 'a3');
% fprintf(s, 'p0');
% 
% fprintf(s, 'a4');
% fprintf(s, 'p0');
% 
>>>>>>> 4ed4e5e410a8082e852a74a6b53062975423a620
% pause(1);