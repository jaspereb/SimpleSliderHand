
closeHand(s, 0);
pause(1);

timeseries1 = [];
timeseries2 = [];
timeseries3 = [];
timeseries4 = [];


for position = 0:10:2000
   disp("Closing Hand");
   closeHand(s,position);
   pause(0.001);
   timeseries1 = [timeseries1; getPacket(s, 1)];
   timeseries2 = [timeseries2; getPacket(s, 2)];
   timeseries3 = [timeseries3; getPacket(s, 3)];
   timeseries4 = [timeseries4; getPacket(s, 4)];

end

pause(1);
openHand(s);

figure();
hold on
plot([timeseries1.time],[timeseries1.load]);
plot([timeseries2.time],[timeseries2.load]);
plot([timeseries3.time],[timeseries3.load]);
plot([timeseries4.time],[timeseries4.load]);