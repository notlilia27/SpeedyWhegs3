close all;clear;clc
%% Things to fix
% Stall torque is relative to stall amps, which is correct, but the
% MotorSelection MatLab file uses a few different methods to estimate Stall
% torque
%% DeWalt 18v Motor
Vnominal=18; % vdc (can be overvolted)
TorqueConst=1.2; %Kt oz-in/amp
VoltageConst=1125; % RPM/volt
StallAmp=155; % amps
StallTorque=StallAmp*TorqueConst;
NoLoadSpeed=VoltageConst*Vnominal;
%% DeWalt Gearbox-----------------------
%Gb_RPM=2000;
%Gb_Max_Torque=450;
%% Sabertooth driver---------------------
%Input voltage: 6-30V nominal, 33.6V absolute max
%Output current: Up to 25A continuous per channel. Peak loads up to 50A per
Driver_I=25;
%% Analysis
%DeWalt at 18V
figure(1)
plot([0,NoLoadSpeed],[StallTorque,0],'-','MarkerSize',20,'DisplayName',...
'18v DeWalt');
xlabel('Speed [RPM]');ylabel('Torque [oz-in]');
title('DeWalt Torque Vs Speed');hold on
set(gca, 'XTickLabel', get(gca, 'XTick'));
ylim([0 250])
%DeWalt at driver limit
PWM_Voltage=8.4;
%Lower this value until DeWalt line hits
%the current*Kt limit and max speed
Vratio=PWM_Voltage/Vnominal;
plot([0,NoLoadSpeed*Vratio],[StallTorque*Vratio,0],...
'-','MarkerSize',20,'DisplayName','PWM DeWalt')
legend
%Driver Effective Line
plot([0,30000],[Driver_I*TorqueConst,Driver_I*TorqueConst],'g',...
'DisplayName','DriverCurrent*Kt')
%RPM reached
%6190 from MotorSelection.xlxs "Wheg RPM Math" tab
plot([6190,6190],[0,500],'k--','DisplayName','Max Speed SpdyWhgs1')
text(6200,40,'SpdyWhgs1 RPM');
%% Motor Torque Vs Speed curve estimate
%This point is the intersection of the Max RPM line, and the PWM DeWalt
%line.
MotorT_point1=30;
MotorRPM_point1=6190;
1.2 Speedy Whegs Torque vs. RPM Linear Estimate
clear;clc
%% From Thesis_MotorCalc_OldDeWalts.m
MotorT_point1=30;
MotorRPM_point1=6190;
%% Plotting Robot Torque Vs. RPM
Trans=13;
t_point=MotorT_point1*(Trans-1); %-1 for some frictional loses in transmission.
RPM_point=MotorRPM_point1/Trans;
RPM=0:100:500;
Torq=(t_point/RPM_point)*RPM;
figure(9)
plot(RPM,Torq,'-','MarkerSize',20,'DisplayName','Estimated Performance');
xlabel('Speed [RPM]');ylabel('Torque [oz-in]');
title('Speedy Whegs linear Torque Vs. RPM estimation');hold on
set(gca, 'XTickLabel', get(gca, 'XTick'));
grid on
% ylim([0 250])
1.3 Motor Option Comparison
% close all;
close all;clear;clc
%% Import from Excel file
T = readtable('MotorSelection.xlsx','Range','D32:t57');
T.Properties.VariableUnits=string(readcell('MotorSelection.xlsx','Range','D32:t32'
));
%% Driver Parameters
D_MaxI=75; %Assume Cooling
% cells=12;
% D_MaxV=cells*(4.2);
%% Plot Motors
figure(10);hold on;
PlotList=[1,2,10,11,16,17,21:25]; %Motors to plot
Trans=(28/12); %Output sprocket teeth / input sprocket teeth
%This line let me set individual transmissions per motor if I wanted
TransList=ones(1,length(PlotList)) * Trans; %Respective Transmissions
for X=1:length(PlotList)
Motor=PlotList(X);
NLS=T.NoLoadSpeed_RPM(Motor)/TransList(X);
if T.MaxCurrent(Motor)<D_MaxI
Tstall=T.MaxCurrent(Motor)*T.Kt(Motor)*TransList(X);
else
Tstall=D_MaxI*T.Kt(Motor)*TransList(X);
end
plot([0,NLS],[Tstall,0],'-','DisplayName',string(T.Name(Motor)));xlabel('Speed
[RPM]');ylabel('Torque [oz-in]');hold on
end
title('Comparing BLDC');
%% Plot Reference DeWalt? (This does not encorporate PWM actual values)
% DeWalt_NLS=20250;DeWalt_ST=186; %%%OLD
% plot([0,DeWalt_NLS/13],[DeWalt_ST*12,0],':','MarkerSize',20);
% text(DeWalt_NLS/2,DeWalt_ST/2,'18v DeWalt');
%% Plot Reference DeWalt?
DeWalt_NLS=9562;DeWalt_ST=87;
plot([0,DeWalt_NLS/13],[DeWalt_ST*12,0],':','MarkerSize',20,'DisplayName','De
Walt');
% text(DeWalt_NLS/2,DeWalt_ST/2,'18v DeWalt');
%Plot RPM goal
plot([1000,1000],[0,1500],'k--','DisplayName','RPM goal')
text(840,40,'Target RPM');
%Finish up figure
axis([0 4000 0 1500]);
% set(gca, 'XTickLabel', get(gca, 'XTick'));
legend
%% Plot Motor Torque vs RPM curve
m=30/6190; %Slope, from x,y intercept point of MotorSWDeWalt.m
Mrpm=50000; %Line length
Mt=m*(Mrpm);
Mrpm=Mrpm/13;Mt=Mt*12; %Account for transmission and torque losses of 1-
ish
plot([0,Mrpm],[0,Mt],'Color','#77AC30','linewidth',3,'DisplayName','Robot T vs
RPM')
%% Plotting Motor Torque at Max Current to Robot Torq/RPM
plot([0,5000],[1,1]*T.Kt(10)*D_MaxI,'linewidth',3,'DisplayName','Lim
Torque'+string(T.Name(10)))
1.4 Alien Power Systems BLDC
close all;clear;clc
figure(10);hold on;
%% Import from Excel file
T = readtable('MotorSelection.xlsx','Range','D32:t57');
T.Properties.VariableUnits=string(readcell('MotorSelection.xlsx','Range','D32:t32'
));
%Driver Parameters
D_MaxI=75; %Need cooling to reach this value
Bat_volt=44.4;
Phase_res=0.3; %ohm
%% Plot Motors
Transmission=3;
Motor=10;
NLS=T.NoLoadSpeed_RPM(Motor)/Transmission;
% Tstall=T.MaxCurrent(Motor)*T.Kt(Motor)*Transmission; %Tstall from max
cont. current
%or
Tstall=Bat_volt/Phase_res*T.Kt(Motor)*Transmission; %Tstall from resistance
plot([0,NLS],[Tstall,0],'b','linewidth',2,'DisplayName',string(T.Name(Motor)));
%% Plot DeWalt - DOUBLE CHECK PREVIOUS WORK HERE
DeWalt_NLS=9562;DeWalt_ST=87;
plot([0,DeWalt_NLS/13],[DeWalt_ST*12,0],':','MarkerSize',20,'DisplayName','De
Walt');
%% Plot Resultant RPM
Res_rpm=1600;
plot([Res_rpm,Res_rpm],[0,2000],'k--','DisplayName','Resultant RPM')
text(Res_rpm+50,50,'Resultant RPM');
%% Plot Robot T/RPM
m=30/6190; %Slope, from x,y intercept point of MotorSWDeWalt.m
Mrpm=50000; %Line length
Mt=m*(Mrpm);
Mrpm=Mrpm/13;Mt=Mt*12; %Account for transmission and torque losses of 1-
ish
plot([0,Mrpm],[0,Mt],'Color','#77AC30','linewidth',2,'DisplayName','Robot
T/RPM')
%% Plotting Motor Torque at Max Current to Robot Torq/RPM
plot([0,5000],[1,1]*T.Kt(Motor)*D_MaxI*Transmission,'Color','#EDB120','linewid
th',2,'DisplayName','DriverCurrent*Kt')
%% Finish up figure
axis([0 3000 0 1500]);
title('Motor: APS6384-100Kv');
xlabel('Speed [RPM]');ylabel('Torque [oz-in]');
legend
grid on
1.5 Ackerman Steering Analysis
close all;clear;clc
%This code does a lot. From calculating the rack offset from the servo
%specifications, all the way to finding the turning radius of the vehicle.
%And ofcourse the main calculation which is the Ackermann steering error.
%% Variables
n=11; %Test Resolution - MUST BE ODD
Pd=0.375; %Pinion Pitch Diameter
%% Constants
mid=round(n/2); %Straight steering geometry
a=17.625; %Joint to Joint steering
p=1.75; %Pivot length
l=23.25; %Body Length
w=14; %Body Width
d=(a-w)/2; %Stub axle length
%Servo Specifics
Sr=160; %Servo Rotation Range in degrees
Rdx=2*pi*(Pd/2)*(Sr/360); %Total Rack Movement in x
RackMove=linspace(-Rdx,Rdx,n);
%Optimization Variables
% p2Offset=0.2; %p2 offset
% r=16; %rack length
% o=1.495; %rack offset
%Good Test Range
% p2Offset=linspace(-0.3,0.3,5)
% r=linspace(15,17,3)
% o=linspace(.5,1.495,4)
% %I choose G=8 from here since it has the second lowest Ackermann rating,
% %with a decently long rack length. S=0.5724in
G=0;
for p2Offset=linspace(-0.3,0.3,5)
for r=linspace(15,17,3)
for o=linspace(.5,1.495,4)
G=G+1; %Keeping track of which geometry
Vars(:,G)=[p2Offset;r;o]; %Keeping track of variables for each G
%Points that are known
p3x=-a/2; p6x=a/2;
p3y=l; p6y=p3y;
p1x=-r/2;
p1y=l-o; p4y=p1y;
%Short link length
s=sqrt((p1x-(p3x-p2Offset))^2+(p1y-(p3y-p))^2);
%Preallocate variables
Points=zeros(6,n,2);%Rows are which point, column is time/RackMove
posision, 3rd dimension is x vs y
Points(1,:,1)=p1x+RackMove;
Points(1,:,2)=p1y;%Same number
Points(3,:,1)=p3x;
Points(3,:,2)=p3y;
Points(4,:,1)=p1x+RackMove+r;
Points(4,:,2)=p1y;
Points(6,:,1)=p6x;
Points(6,:,2)=p6y;
Theta1=zeros(1,n);
Alpha1=zeros(1,n);
Theta2=zeros(1,n);
Alpha2=zeros(1,n);
for k=1:n
syms p2x p2y alpha1 th1
eq1=p2x==Points(1,k,1)+s*cosd(alpha1);
eq2=p2y==p1y+s*sind(alpha1);
eq3=p3x==p2x+p*cosd(th1);
eq4=p3y==p2y+p*sind(th1);
S1=vpasolve([eq1 eq2 eq3 eq4],[p2x p2y alpha1 th1]);
while double(S1.p2x) > Points(1,k,1) || double(S1.p2y) > Points(3,k,2)
S1=vpasolve([eq1 eq2 eq3 eq4],[p2x p2y alpha1
th1],'Random',true);
end
Points(2,k,1)=S1.p2x;
Points(2,k,2)=S1.p2y;
Theta1(k)=wrapTo360(S1.th1);
Alpha1(k)=wrapTo360(S1.alpha1);
syms p5x p5y alpha2 th2
eq1=p5x==Points(4,k,1)+s*cosd(alpha2);
eq2=p5y==p4y+s*sind(alpha2);
eq3=p6x==p5x+p*cosd(th2);
eq4=p6y==p5y+p*sind(th2);
S2=vpasolve([eq1 eq2 eq3 eq4],[p5x p5y alpha2 th2]); %I encounter
two solution errors, where vpa just chooses one. Option1 use
,'Random',true and make sure a variable does not change two much, or
option2 set guesses
while double(S2.p5x) < Points(4,k,1) || double(S2.p5y) > Points(3,k,2)
S2=vpasolve([eq1 eq2 eq3 eq4],[p5x p5y alpha2
th2],'Random',true);
end
Points(5,k,1)=S2.p5x;
Points(5,k,2)=S2.p5y;
Theta2(k)=wrapTo360(S2.th2);
Alpha2(k)=wrapTo360(S2.alpha2);
clc
fprintf('G:%2.0f \n',G)
fprintf('Progress:%2.0f out of %2.0f\n',k,length(RackMove))
end
%% Offset Fix
%This is needed if the angle of P2 to P3 (and P5 to P6) is not 90 in
center
%position since the whegs themselves will be at 90 at center position.
Offset=(90-Theta1(mid)); %If P2 is directly below P3, This section should
do nothing
Theta1=Theta1+Offset;
Theta2=Theta2-Offset;
%% Angle Math
%Calculate True Ackermann Theta2 value based on Theta1
Theta2_ack=abs(atand(tand(Theta1)-a/l));
%Redefine Theta1 & Theta2 to be more useful
Theta1=abs(Theta1-90);
Theta2=abs(Theta2-90);
%Redefine Ackermann Theta2 to be more useful
Theta2_ack=abs(Theta2_ack-90);
%% Info Return
MaxTurn(G)=abs(Theta1(1)-90);
fprintf('Wheg Turn Angle:%2.1f Degrees \n',MaxTurn(G))
fprintf('SpeedyWhegs2 1 axle Turn Radius: \n') %Only accounting for
rack and pinion on front
fprintf('SpeedyWhegs2 2 axle Turn Radius: \n')
%% Gemoetry Characteristics Plots
figure(G);
% Plot Theta1 vs Theta2
subplot(2,1,1);
grid on;
axis([-10 10 20 25])
axis equal;
mid=round(n/2);
xs=[Points(1:3,mid,1)',Points(6,mid,1),Points(5,mid,1),Points(4,mid,1),Poi
nts(1,mid,1)];
ys=[Points(1:3,mid,2)',Points(6,mid,2),Points(5,mid,2),Points(4,mid,2),Poi
nts(1,mid,2)];
MECH=line(xs,ys);
title('Geometry Display')
xlabel('X [in]')
ylabel('Y [in]')
% Plot No-turn
subplot(2,1,2);
plot(Theta1,'k','DisplayName','Theta1');hold on
plot(Theta2,'r-','DisplayName','Theta2 Geometry Results')
plot(Theta2_ack,'r:','DisplayName','Theta 2 True Ackermann');hold off
title('Ackermann Error')
ylabel('Angle [Deg]')
xlabel('Rack Dx Step')
legend
%% Calculate Ackermann Error
Approx=Theta2;
Exact=Theta2_ack;
Error(G)=sum(abs(Approx-Exact))/n;
%Should I instead do an RMS?????
RMSerror(G)=sqrt(sum((Exact-Approx).^2)/n); %punishes extremes
end
end
end
%% Final Analysis -- Worst Points
figure(101)
subplot(2,1,1);hold on
plot(Error,'k')
p4=plot([9,9],[0,30],'--r');
plot([21,21],[0,30],'--r')
plot([33,33],[0,30],'--r')
plot([45,45],[0,30],'--r')
plot([57,57],[0,30],'--r')
title('Error for each geometry, G')
ylabel('Ackermann Error')
xlabel('Geometry, G')
legend(p4,{'Max Error Points'})
axis([1 length(Error) 0 30])
%
subplot(2,1,2)
hold on
p1=plot(Vars(1,:),'Color',[0.8500, 0.3250, 0.0980]);
p2=plot(Vars(2,:),'Color',[0.4940, 0.1840, 0.5560]);
p3=plot(Vars(3,:),'Color',[0.9290, 0.6940, 0.1250]);
p4=plot([9,9],[-2,20],'--r');
plot([21,21],[-2,20],'--r');
plot([33,33],[-2,20],'--r')
plot([45,45],[-2,20],'--r')
plot([57,57],[-2,20],'--r')
title('Iteration Values')
ylabel('Value')
xlabel('Geometry, G')
legend(p4,{'Least Error Points'})
axis([1 length(Error) -2 20])
legend([p1 p2 p3 p4],{'Point2 X Offset','Rack Length','Rack Y Offset','Max Error
Points'})
hold off
%% Final Analysis -- Best Points
[ErrorValue,Index] = min(Error);
fprintf('Best geometry at G=%2.0f with an Error value of %2.2f
\n',Index,ErrorValue)
figure(102)
subplot(2,1,1);hold on
plot(Error,'k')
p4=plot([4,4],[0,30],'--','Color',[0.4660, 0.6740, 0.1880]);
plot([8,8],[0,30],'--','Color',[0.4660, 0.6740, 0.1880])
plot([16,16],[0,30],'--','Color',[0.4660, 0.6740, 0.1880])
plot([20,20],[0,30],'--','Color',[0.4660, 0.6740, 0.1880])
plot([28,28],[0,30],'--','Color',[0.4660, 0.6740, 0.1880])
title('Error for each geometry, G')
ylabel('Ackermann Error')
xlabel('Geometry, G')
legend(p4,{'Least Error Points'})
axis([1 length(Error) 0 30])
%
subplot(2,1,2)
hold on
p1=plot(Vars(1,:),'Color',[0.8500, 0.3250, 0.0980]);
p2=plot(Vars(2,:),'Color',[0.4940, 0.1840, 0.5560]);
p3=plot(Vars(3,:),'Color',[0.9290, 0.6940, 0.1250]);
p4=plot([4,4],[0,30],'--','Color',[0.4660, 0.6740, 0.1880]);
plot([8,8],[0,30],'--','Color',[0.4660, 0.6740, 0.1880])
plot([16,16],[0,30],'--','Color',[0.4660, 0.6740, 0.1880])
plot([20,20],[0,30],'--','Color',[0.4660, 0.6740, 0.1880])
plot([28,28],[0,30],'--','Color',[0.4660, 0.6740, 0.1880])
title('Iteration Values')
ylabel('Value')
xlabel('Geometry, G')
legend(p4,{'Least Error Points'})
axis([1 length(Error) -2 20])
legend([p1 p2 p3 p4],{'Point2 X Offset','Rack Length','Rack Y Offset','Least Error Points'})
hold off
%% Outside of Loop
%% Plot Specific Geometry Animation
figure;grid on;
axis([-10 10 16 28])
axis equal;
hold on
for k=1:n
xs=[Points(1:3,k,1)',Points(6,k,1),Points(5,k,1),Points(4,k,1),Points(1,k,1)];
ys=[Points(1:3,k,2)',Points(6,k,2),Points(5,k,2),Points(4,k,2),Points(1,k,2)];
MECH=line(xs,ys);
pause(.1)
drawnow
if k==1;pause(1);end
if k ~= n
delete(MECH);
end
end
hold off
%% Lessons Learned
%Large Rack Length + small Rack Y Offset gives the largest ackermann error
%The closer the Point2 x Offset is to the width of the chasis body (Larger value),
the less error.
%The larger the Rack Y Offset, the better.
1.6 Reading IMU Data from SD Card
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