function SCARA_animation(t, q)
% SCARA_animation - Animerer SCARA robot
%   t: tidsvektor [n x 1]
%   q: leddvariabler [n x 3] der q = [theta1, theta2, d3]

% Bygg SCARA robot basert på DH-parametere
robot = rigidBodyTree('DataFormat','column');

% Joint 1: θ1 (variabel), d1=1, a1=1, α1=0
body1 = rigidBody('link1');
jnt1 = rigidBodyJoint('jnt1', 'revolute');
setFixedTransform(jnt1, trvec2tform([1 0 1]));  % Tx(a1) * Tz(d1)
jnt1.JointAxis = [0 0 1];
body1.Joint = jnt1;
addBody(robot, body1, 'base');

% Joint 2: θ2 (variabel), d2=0, a2=1, α2=0
body2 = rigidBody('link2');
jnt2 = rigidBodyJoint('jnt2', 'revolute');
setFixedTransform(jnt2, trvec2tform([1 0 0]));  % Tx(a2)
jnt2.JointAxis = [0 0 1];
body2.Joint = jnt2;
addBody(robot, body2, 'link1');

% Joint 3: θ3=0, d3 (variabel), a3=0, α3=π
body3 = rigidBody('link3');
jnt3 = rigidBodyJoint('jnt3', 'prismatic');
setFixedTransform(jnt3, axang2tform([1 0 0 pi]));  % Rx(α3=π)
jnt3.JointAxis = [0 0 1];  % Prismatisk langs z
body3.Joint = jnt3;
addBody(robot, body3, 'link2');

% Animasjon
figure('Position', [100 100 800 600]);
ax = gca;
view(3)
axis([-2 2 -2 2 -1 2])
hold on
grid on
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
title('SCARA Robot Animasjon')

for i = 1:10:length(t)
    config = q(i,:)';
    show(robot, config, 'Parent', ax, 'PreservePlot', false);
    title(sprintf('SCARA Robot - Tid: %.2f s', t(i)))
    drawnow
    pause(0.01)
end

% Beregn og skriv ut endeffektorposisjon og orientering
final_config = q(end,:)';
T_end = getTransform(robot, final_config, 'link3');
end_pos = T_end(1:3, 4);
R_end = T_end(1:3, 1:3);
euler_angles = rotm2eul(R_end, 'ZYX');

fprintf('\n=== Endeefektor sluttposisjon ===\n');
fprintf('Pos: [%.3f, %.3f, %.3f] m\n', end_pos(1), end_pos(2), end_pos(3));
fprintf('RPY: [%.3f, %.3f, %.3f] rad ([%.1f°, %.1f°, %.1f°])\n', ...
    euler_angles(3), euler_angles(2), euler_angles(1), ...
    rad2deg(euler_angles(3)), rad2deg(euler_angles(2)), rad2deg(euler_angles(1)));

end