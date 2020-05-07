clear
clc
close all

toolPosG02 = load('mode10ToolPosGain02.txt');
toolPosG03 = load('mode10ToolPosGain03.txt');
toolPosG01 = load('mode10ToolPosGain01.txt');
toolPosG005 = load('mode10ToolPosGain005.txt');
toolPosG002 = load('mode10ToolPosGain002.txt');

RMSErrs = [evaluateAxisTracking(toolPosG02);
          evaluateAxisTracking(toolPosG03);
          evaluateAxisTracking(toolPosG01);
          evaluateAxisTracking(toolPosG005);
          evaluateAxisTracking(toolPosG002)];

T = 1:size(toolPosG02,2);

figure
hold on
grid on
for i=1:5
    plot(T,RMSErrs(i,:), 'LineWidth', 1.5)
end

xlabel('Sim time (s)')
ylabel('Tool Axis RMS Err')
title('RMS Error from tool axis position for different force gains')
legend(num2str(0.2),num2str(0.3),num2str(0.1),...
    num2str(0.05),num2str(0.02))

function RMSErr = evaluateAxisTracking(toolPos)

startPt = toolPos(:,1);
endPt = toolPos(:,end);

ref = [linspace(startPt(1),endPt(1),size(toolPos,2));
       linspace(startPt(2),endPt(2),size(toolPos,2));
       linspace(startPt(3),endPt(3),size(toolPos,2))];
   
RMSErr = [];

for i=1:size(toolPos,2)
    RMSErr = [RMSErr norm(ref(:,i)-toolPos(:,i))];
end

end