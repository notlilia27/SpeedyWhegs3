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