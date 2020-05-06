% Spring 2020 16-711 KDC
% Project
clear
clc
close all

slGains = [0.025 0.25 2.5 5 10];
RMSErrs = [];

for i=1:length(slGains)
    RMSErrs = [RMSErrs; testModelTeleOp(slGains(i))];
end

T = 1:size(RMSErrs,2);

figure
hold on
for i=1:length(slGains)
    plot(T,RMSErrs(i,:), 'LineWidth', 1.5)
end
xlabel('Sim time (s)')
ylabel('Tool Position RMS Err')
title('RMS Error between master and slave for different EnvGains')
legend(num2str(slGains(1)),num2str(slGains(2)),num2str(slGains(3)),...
    num2str(slGains(4)),num2str(slGains(5)))