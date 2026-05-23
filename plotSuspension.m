function plotSuspension(HP)

figure('Color','w');
hold on;
grid on;
axis equal;
view(135,25);

xlabel('X (left to right)');
ylabel('Y (front to back)');
zlabel('Z (up and down)');
title('Quarter Car Suspension');

% -----------------------------
% Ground reference
% -----------------------------
[xg, yg] = meshgrid(-1:2:60, -20:2:20);
zg = zeros(size(xg));
surf(xg, yg, zg, ...
    'FaceAlpha', 0.08, ...
    'EdgeColor', [0.8 0.8 0.8]);

% -----------------------------
% Steering axis intersection with ground
% -----------------------------
vSA = HP.UBJ - HP.LBJ;
tSA = -HP.LBJ(3) / vSA(3);
SA_ground = HP.LBJ + tSA * vSA;

% -----------------------------
% Inner pivot midpoints
% -----------------------------
UCA_mid = (HP.UCA_FI + HP.UCA_RI) / 2;
LCA_mid = (HP.LCA_FI + HP.LCA_RI) / 2;

% -----------------------------
% Front-view instant center
% -----------------------------
P1 = [UCA_mid(1), UCA_mid(3)];
P2 = [HP.UBJ(1),  HP.UBJ(3)];

P3 = [LCA_mid(1), LCA_mid(3)];
P4 = [HP.LBJ(1),  HP.LBJ(3)];

[xIC, zIC, isParallel] = lineIntersectionXZ(P1, P2, P3, P4);

IC3 = [xIC, 0, zIC];

% -----------------------------
% Roll center
% -----------------------------
if ~isParallel
    CPxz = [HP.CP(1), 0];

    dx = xIC - CPxz(1);
    dz = zIC - CPxz(2);

    if abs(dx) > 1e-9
        tRC = (0 - CPxz(1)) / dx;
        zRC = CPxz(2) + tRC * dz;
        RC3 = [0, 0, zRC];
        hasRC = true;
    else
        RC3 = [NaN, NaN, NaN];
        hasRC = false;
    end
else
    RC3 = [NaN, NaN, NaN];
    hasRC = false;
end

% -----------------------------
% Collect and plot all points
% -----------------------------
pts = [
    HP.UCA_FI;
    HP.UCA_RI;
    HP.LCA_FI;
    HP.LCA_RI;
    HP.UCA_FC;
    HP.UCA_RC;
    HP.LCA_FC;
    HP.LCA_RC;
    HP.UBJ;
    HP.LBJ;
    HP.HUB;
    HP.SP0;
    HP.SP1;
    HP.WC;
    HP.WT;
    HP.WB;
    HP.WTF;
    HP.WTR;
    HP.WBF;
    HP.WBR;
    HP.CP;
    HP.CP_FL;
    HP.CP_FR;
    HP.CP_RL;
    HP.CP_RR;
    HP.STEER_ARM_END;
    HP.rackCenter;
    HP.RACK_RIGHT;
    SA_ground
];

if ~isParallel
    pts = [pts; IC3];
end

if hasRC
    pts = [pts; RC3];
end

plot3(pts(:,1), pts(:,2), pts(:,3), 'ko', ...
    'MarkerFaceColor', 'k', 'MarkerSize', 7);

% -----------------------------
% Chassis / suspension box
% -----------------------------
drawLine(HP.UCA_FI, HP.UCA_RI, 'r');
drawLine(HP.LCA_FI, HP.LCA_RI, 'r');
drawLine(HP.UCA_FI, HP.LCA_FI, 'r');
drawLine(HP.UCA_RI, HP.LCA_RI, 'r');

% -----------------------------
% Connect box back to centerline
% -----------------------------
drawLine(HP.UCA_FI, HP.UCA_FC, 'r');
drawLine(HP.UCA_RI, HP.UCA_RC, 'r');
drawLine(HP.LCA_FI, HP.LCA_FC, 'r');
drawLine(HP.LCA_RI, HP.LCA_RC, 'r');

drawLine(HP.UCA_FC, HP.UCA_RC, 'k');
drawLine(HP.LCA_FC, HP.LCA_RC, 'k');
drawLine(HP.UCA_FC, HP.LCA_FC, 'k');
drawLine(HP.UCA_RC, HP.LCA_RC, 'k');

% -----------------------------
% Upper control arm
% -----------------------------
drawLine(HP.UCA_FI, HP.UBJ, 'b');
drawLine(HP.UCA_RI, HP.UBJ, 'b');

% -----------------------------
% Lower control arm
% -----------------------------
drawLine(HP.LCA_FI, HP.LBJ, 'b');
drawLine(HP.LCA_RI, HP.LBJ, 'b');

% -----------------------------
% Upright / knuckle main line
% -----------------------------
drawLine(HP.UBJ, HP.LBJ, 'm');

% -----------------------------
% Steering axis extension to ground
% -----------------------------
drawLine(HP.LBJ, SA_ground, [1 0 1]);

% -----------------------------
% Hub to spindle
% -----------------------------
drawLine(HP.HUB, HP.SP1, 'g');

plot3(HP.HUB(1), HP.HUB(2), HP.HUB(3), 'go', ...
    'MarkerFaceColor', 'g', 'MarkerSize', 8);

% -----------------------------
% Steering arm
% -----------------------------
drawLine(HP.HUB, HP.STEER_ARM_END, [0.6 0.2 0.8]);

plot3(HP.STEER_ARM_END(1), HP.STEER_ARM_END(2), HP.STEER_ARM_END(3), 'o', ...
    'MarkerFaceColor', [0.6 0.2 0.8], 'MarkerEdgeColor', 'k', 'MarkerSize', 8);

% -----------------------------
% Steering rack
% -----------------------------
drawLine(HP.rackCenter, HP.RACK_RIGHT, [0.2 0.6 0.2]);

plot3(HP.rackCenter(1), HP.rackCenter(2), HP.rackCenter(3), 'o', ...
    'MarkerFaceColor', [0.2 0.6 0.2], 'MarkerEdgeColor', 'k', 'MarkerSize', 8);

plot3(HP.RACK_RIGHT(1), HP.RACK_RIGHT(2), HP.RACK_RIGHT(3), 'o', ...
    'MarkerFaceColor', [0.2 0.6 0.2], 'MarkerEdgeColor', 'k', 'MarkerSize', 8);

% -----------------------------
% Tie rod
% -----------------------------
drawLine(HP.RACK_RIGHT, HP.STEER_ARM_END, [1 0.5 0]);

% -----------------------------
% Wheel / camber line
% -----------------------------
drawLine(HP.WB, HP.WT, 'c');

plot3(HP.WC(1), HP.WC(2), HP.WC(3), 'yo', ...
    'MarkerFaceColor', 'y', 'MarkerSize', 8);

% -----------------------------
% Tire outline
% -----------------------------
drawLine(HP.WBF, HP.WTF, 'c');
drawLine(HP.WBR, HP.WTR, 'c');
drawLine(HP.WTF, HP.WTR, 'c');
drawLine(HP.WBF, HP.WBR, 'c');

% -----------------------------
% Contact patch
% -----------------------------
drawLine(HP.CP_FL, HP.CP_FR, 'k');
drawLine(HP.CP_FR, HP.CP_RR, 'k');
drawLine(HP.CP_RR, HP.CP_RL, 'k');
drawLine(HP.CP_RL, HP.CP_FL, 'k');

plot3(HP.CP(1), HP.CP(2), HP.CP(3), 'ks', ...
    'MarkerFaceColor', 'k', 'MarkerSize', 7);

% -----------------------------
% Scrub radius line
% -----------------------------
drawLine(SA_ground, HP.CP, [0.85 0.4 0]);

plot3(SA_ground(1), SA_ground(2), SA_ground(3), 'mo', ...
    'MarkerFaceColor', 'm', 'MarkerSize', 8);

% -----------------------------
% Front-view arm construction lines to IC
% -----------------------------
if ~isParallel
    drawLine(UCA_mid, IC3, [0 0.5 1]);
    drawLine(LCA_mid, IC3, [0 0.5 1]);

    plot3(IC3(1), IC3(2), IC3(3), 'bd', ...
        'MarkerFaceColor', 'b', 'MarkerSize', 9);

    drawLine(HP.CP, IC3, [0.5 0 0.8]);
end

% -----------------------------
% Roll center marker
% -----------------------------
if hasRC
    plot3(RC3(1), RC3(2), RC3(3), 'rs', ...
        'MarkerFaceColor', 'r', 'MarkerSize', 9);

    drawLine(RC3, [5, 0, RC3(3)], [1 0 0]);
end

% -----------------------------
% Labels
% -----------------------------
labelPoint(HP.UCA_FI, 'UCA FI');
labelPoint(HP.UCA_RI, 'UCA RI');
labelPoint(HP.LCA_FI, 'LCA FI');
labelPoint(HP.LCA_RI, 'LCA RI');
labelPoint(HP.UBJ,    'UBJ');
labelPoint(HP.LBJ,    'LBJ');
labelPoint(HP.HUB,    'HUB');
labelPoint(HP.SP1,    'SP1');
labelPoint(HP.STEER_ARM_END, 'STEER ARM');
labelPoint(HP.rackCenter, 'RACK C');
labelPoint(HP.RACK_RIGHT, 'RACK R');
labelPoint(HP.WC,     'WC');
labelPoint(HP.WT,     'WT');
labelPoint(HP.WB,     'WB');
labelPoint(HP.CP,     'CP');
labelPoint(SA_ground, 'SA GND');

if ~isParallel
    labelPoint(IC3, 'IC');
end

if hasRC
    labelPoint(RC3, 'RC');
end

xlim([-5 60]);
ylim([-20 20]);
zlim([0 30]);

end

function drawLine(P1, P2, colorValue)
plot3([P1(1) P2(1)], [P1(2) P2(2)], [P1(3) P2(3)], ...
    'Color', colorValue, 'LineWidth', 2);
end

function labelPoint(P, txt)
text(P(1), P(2), P(3), ['  ' txt], ...
    'Color', 'k', 'FontSize', 11, 'FontWeight', 'bold');
end

function [xInt, zInt, isParallel] = lineIntersectionXZ(P1, P2, P3, P4)

x1 = P1(1); z1 = P1(2);
x2 = P2(1); z2 = P2(2);
x3 = P3(1); z3 = P3(2);
x4 = P4(1); z4 = P4(2);

den = (x1 - x2)*(z3 - z4) - (z1 - z2)*(x3 - x4);

if abs(den) < 1e-9
    xInt = NaN;
    zInt = NaN;
    isParallel = true;
    return;
end

xInt = ((x1*z2 - z1*x2)*(x3 - x4) - (x1 - x2)*(x3*z4 - z3*x4)) / den;
zInt = ((x1*z2 - z1*x2)*(z3 - z4) - (z1 - z2)*(x3*z4 - z3*x4)) / den;

isParallel = false;

end