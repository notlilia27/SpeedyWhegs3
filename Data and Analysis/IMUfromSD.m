% %% Make sure no data exists
% %Run this section to clear data
close all
%% View Plot
%HOME/Import Data/(Check that delimiters are tabs and press ok top right)
%Change the name of the imported txt file to Data, OR fix the following
%line with an import function such as readtable with the correct file location.
data=Data;
%1780 is a close enough approximation to the MPU's gravity sensitivity
%setting based on the example MPU6050 DMP Arduino Sketch
Gsens=1780;
data.Properties.VariableNames = {'RawTime' 'Dum1' 'gyaw' 'gpitch' 'groll' 'Dum2''ax' 'ay' 'az'};
data.ax=data.ax/Gsens;
data.ay=data.ay/Gsens;
% data.az=data.az/Gsens;
data.az=(data.az+1780)/Gsens;
time=(data.RawTime-data.RawTime(1))/1000000;
figure(1)
hold on
title('Gyro Values')
plot(time,data.gyaw,'DisplayName','Yaw')
plot(time,data.gpitch,'DisplayName','Pitch')
plot(time,data.groll,'DisplayName','Roll')
ylabel('Angle [Degrees]')
xlabel('Time [Seconds]')
grid on
legend
figure(2)
hold on
title('Acceleration Values')
plot(time,data.ax,'DisplayName','X Accel')
plot(time,data.ay,'DisplayName','y Accel')
plot(time,data.az,'DisplayName','Z Accel')
ylabel('Acceleration [G''s]')
xlabel('Time [Seconds]')
grid on
legend
%% Acceleration Smoothing
n=19; %Use an odd number.
SmoothedInterval=n*(time(2)-time(1)) %#ok<NOPTS>
MiddleVals=(n/2+0.5):length(data.az)-(n/2-0.5);
newx=ones(1,length(MiddleVals))*1000;
newy=newx;
newz=newx;
for k=MiddleVals
index=(1:n)-(n/2+0.5)+k;
newx_vect=data.ax(index);
newx(k-(n/2-0.5))=sum(newx_vect)/n;
newy_vect=data.ay(index);
newy(k-(n/2-0.5))=sum(newy_vect)/n;
newz_vect=data.az(index);
newz(k-(n/2-0.5))=sum(newz_vect)/n;
end
figure(3)
hold on
title('Smoothed Acceleration')
plot(time(MiddleVals)',newx,'DisplayName','X Accel')
plot(time(MiddleVals)',newy,'DisplayName','Y Accel')
plot(time(MiddleVals)',newz,'DisplayName','Z Accel')
ylabel('Acceleration [G''s]')
xlabel('Time [Seconds]')
grid on
legend
%% Run this section of code to find line # of erroneous data in txt file.
% errortime=13.4425;
% [dummy,index2fix]=min(abs(errortime-time));
%
% % %Old method
% % lowbound=errortime-0.02;
% % highbound=errortime+0.02;
% % index2fix=find(time<highbound & time>lowbound);