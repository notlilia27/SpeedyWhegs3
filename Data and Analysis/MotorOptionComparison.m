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
Torque'+string(T.Name(10))