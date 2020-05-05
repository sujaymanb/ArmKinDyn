function animateArm(jointPos,toolPos,gToolSurface,rotAxis)
% plot tool trajectory
plot3(toolPos(1,:), toolPos(2,:), toolPos(3,:), 'k-', 'LineWidth', 1.5)
hold on

% plot robot joints and tool
plot3(jointPos([1, 2, 4, 6, 7],1), jointPos([1, 2, 4, 6, 7],2),...
    jointPos([1, 2, 4, 6, 7],3), 'bo-', 'LineWidth', 1.5)
plot3([jointPos(7,1) toolPos(1,end)], [jointPos(7,2) toolPos(2,end)],...
    [jointPos(7,3) toolPos(3,end)], 'g-', 'LineWidth', 5)

% plot tool surface axis
plot3([gToolSurface(1,end),rotAxis(1,1)],[gToolSurface(2,end),rotAxis(2,1)],[gToolSurface(3,end),rotAxis(3,1)], 'r-', 'LineWidth', 1.5)
plot3([gToolSurface(1,end),rotAxis(1,2)],[gToolSurface(2,end),rotAxis(2,2)],[gToolSurface(3,end),rotAxis(3,2)], 'g-', 'LineWidth', 1.5)
plot3([gToolSurface(1,end),rotAxis(1,3)],[gToolSurface(2,end),rotAxis(2,3)],[gToolSurface(3,end),rotAxis(3,3)], 'b-', 'LineWidth', 1.5)

xlim([-1, 1])
ylim([-1, 1])
zlim([0, 2])
xlabel('x')
ylabel('y')
zlabel('z')
legend('Tool Path', 'Arm Links', 'Tool')

hold off
grid on

end