closeHand(s, 0);
pause(1);

timeseries1 = [];
fprintf(s, 'a4');

for position = 0:100:2500
    fprintf(s, 'p%d\n', position);
    pause(0.005);
    timeseries1 = [timeseries1; getPacket(s, 4)];
   
end

pause(1);
openHand(s);

% timeseries1 = timeseries1(5:end,:);
figure();
hold on
plot([timeseries1.position],[timeseries1.load]);