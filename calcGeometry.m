function G = calcGeometry(HP)

% ------------------------------------------------
% Inner pivot midpoints
% ------------------------------------------------
UCA_mid = (HP.UCA_FI + HP.UCA_RI) / 2;
LCA_mid = (HP.LCA_FI + HP.LCA_RI) / 2;

% ------------------------------------------------
% Control arm lengths
% ------------------------------------------------
G.UCA_front_length = norm(HP.UBJ - HP.UCA_FI);
G.UCA_rear_length  = norm(HP.UBJ - HP.UCA_RI);

G.LCA_front_length = norm(HP.LBJ - HP.LCA_FI);
G.LCA_rear_length  = norm(HP.LBJ - HP.LCA_RI);

G.UCA_mid_length   = norm(HP.UBJ - UCA_mid);
G.LCA_mid_length   = norm(HP.LBJ - LCA_mid);

% ------------------------------------------------
% Knuckle / upright lengths
% ------------------------------------------------
G.upright_length       = norm(HP.UBJ - HP.LBJ);
G.upper_knuckle_length = norm(HP.UBJ - HP.HUB);
G.lower_knuckle_length = norm(HP.HUB - HP.LBJ);

% ------------------------------------------------
% Spindle length
% ------------------------------------------------
G.spindle_length = norm(HP.SP1 - HP.HUB);

% ------------------------------------------------
% Steering axis vector
% ------------------------------------------------
v = HP.UBJ - HP.LBJ;

G.KPI    = atan2d(v(1), v(3));
G.caster = atan2d(v(2), v(3));

% ------------------------------------------------
% Wheel / camber
% ------------------------------------------------
vw = HP.WT - HP.WB;
G.camber = atan2d(vw(1), vw(3));

G.camber_offset_from_KPI = G.camber - G.KPI;

% ------------------------------------------------
% Calculated steering arm angle and toe
% 0 deg arm angle = +Y forward
% toe = arm angle relative to zero-toe arm angle
% ------------------------------------------------
vArm = HP.STEER_ARM_END - HP.HUB;
G.steering_arm_angle_calc = atan2d(vArm(1), vArm(2));
G.toe_calc = G.steering_arm_angle_calc - HP.steeringArmZeroToeAngle;

% ------------------------------------------------
% Steering axis ground intersection
% ------------------------------------------------
t = -HP.LBJ(3) / v(3);
G.SA_ground = HP.LBJ + t * v;

% ------------------------------------------------
% Scrub radius
% ------------------------------------------------
G.scrub_radius_signed = HP.CP(1) - G.SA_ground(1);
G.scrub_radius_abs    = abs(G.scrub_radius_signed);

% ------------------------------------------------
% Mechanical trail
% ------------------------------------------------
G.mechanical_trail_signed = HP.CP(2) - G.SA_ground(2);
G.mechanical_trail_abs    = abs(G.mechanical_trail_signed);

% ------------------------------------------------
% Control arm slope vectors
% ------------------------------------------------
vUCA = HP.UBJ - UCA_mid;
vLCA = HP.LBJ - LCA_mid;

G.UCA_front_view_slope = atan2d(vUCA(3), vUCA(1));
G.LCA_front_view_slope = atan2d(vLCA(3), vLCA(1));

G.UCA_side_view_slope  = atan2d(vUCA(3), vUCA(2));
G.LCA_side_view_slope  = atan2d(vLCA(3), vLCA(2));

% ------------------------------------------------
% Front-view instant center
% ------------------------------------------------
P1 = [UCA_mid(1), UCA_mid(3)];
P2 = [HP.UBJ(1),  HP.UBJ(3)];

P3 = [LCA_mid(1), LCA_mid(3)];
P4 = [HP.LBJ(1),  HP.LBJ(3)];

[xIC, zIC, isParallel] = lineIntersectionXZ(P1, P2, P3, P4);

G.IC_front = [xIC, zIC];
G.IC_parallel = isParallel;

% ------------------------------------------------
% Front-view roll center
% ------------------------------------------------
if ~isParallel
    CPxz = [HP.CP(1), 0];

    dx = xIC - CPxz(1);
    dz = zIC - CPxz(2);

    if abs(dx) > 1e-9
        tRC = (0 - CPxz(1)) / dx;
        zRC = CPxz(2) + tRC * dz;

        G.roll_center = [0, zRC];
        G.roll_center_height = zRC;
    else
        G.roll_center = [NaN, NaN];
        G.roll_center_height = NaN;
    end
else
    G.roll_center = [NaN, NaN];
    G.roll_center_height = NaN;
end

% ------------------------------------------------
% Side-view instant center
% ------------------------------------------------
P1_sv = [UCA_mid(2), UCA_mid(3)];
P2_sv = [HP.UBJ(2),  HP.UBJ(3)];

P3_sv = [LCA_mid(2), LCA_mid(3)];
P4_sv = [HP.LBJ(2),  HP.LBJ(3)];

[yIC, zIC_sv, isParallel_sv] = lineIntersectionYZ(P1_sv, P2_sv, P3_sv, P4_sv);

G.IC_side = [yIC, zIC_sv];
G.IC_side_parallel = isParallel_sv;

% ------------------------------------------------
% Side-view force line angle
% ------------------------------------------------
CP_sv = [HP.CP(2), 0];
IC_sv = [yIC, zIC_sv];

dy = IC_sv(1) - CP_sv(1);
dz = IC_sv(2) - CP_sv(2);

G.force_line_angle = atan2d(dz, dy);

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

function [yInt, zInt, isParallel] = lineIntersectionYZ(P1, P2, P3, P4)

y1 = P1(1); z1 = P1(2);
y2 = P2(1); z2 = P2(2);
y3 = P3(1); z3 = P3(2);
y4 = P4(1); z4 = P4(2);

den = (y1 - y2)*(z3 - z4) - (z1 - z2)*(y3 - y4);

if abs(den) < 1e-9
    yInt = NaN;
    zInt = NaN;
    isParallel = true;
    return;
end

yInt = ((y1*z2 - z1*y2)*(y3 - y4) - (y1 - y2)*(y3*z4 - z3*y4)) / den;
zInt = ((y1*z2 - z1*y2)*(z3 - z4) - (z1 - z2)*(y3*z4 - z3*y4)) / den;

isParallel = false;

end