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