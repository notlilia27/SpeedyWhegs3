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