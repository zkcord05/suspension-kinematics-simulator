clc;
clear;
close all;

% ------------------------------------------------
% Static suspension
% ------------------------------------------------
HP = hardpoints();
G  = calcGeometry(HP);

plotSuspension(HP);

fprintf('\n');
fprintf('---------------- Suspension Geometry ----------------\n');

fprintf('Upper Control Arm Front Length : %.2f\n', G.UCA_front_length);
fprintf('Upper Control Arm Rear Length  : %.2f\n', G.UCA_rear_length);
fprintf('Lower Control Arm Front Length : %.2f\n', G.LCA_front_length);
fprintf('Lower Control Arm Rear Length  : %.2f\n', G.LCA_rear_length);

fprintf('Upright Length                 : %.2f\n', G.upright_length);
fprintf('Upper Knuckle Length           : %.2f\n', G.upper_knuckle_length);
fprintf('Lower Knuckle Length           : %.2f\n', G.lower_knuckle_length);
fprintf('Spindle Length                 : %.2f\n', G.spindle_length);
fprintf('Steering Arm Length            : %.2f\n', HP.steeringArmLength);
fprintf('Steering Arm Angle (calc)      : %.2f deg\n', G.steering_arm_angle_calc);
fprintf('Rack Half Length               : %.2f\n', HP.rackHalfLength);
fprintf('Tie Rod Length                 : %.2f\n', HP.tieRodLength);

fprintf('\n');
fprintf('KPI                            : %.2f deg\n', G.KPI);
fprintf('Caster                         : %.2f deg\n', G.caster);
fprintf('Camber                         : %.2f deg\n', G.camber);
fprintf('Toe (calculated)               : %.2f deg\n', G.toe_calc);

fprintf('UCA Front View Slope           : %.2f deg\n', G.UCA_front_view_slope);
fprintf('LCA Front View Slope           : %.2f deg\n', G.LCA_front_view_slope);
fprintf('UCA Side View Slope            : %.2f deg\n', G.UCA_side_view_slope);
fprintf('LCA Side View Slope            : %.2f deg\n', G.LCA_side_view_slope);

fprintf('\n');
fprintf('Scrub Radius (signed)          : %.2f\n', G.scrub_radius_signed);
fprintf('Mechanical Trail (signed)      : %.2f\n', G.mechanical_trail_signed);

fprintf('\n');
fprintf('Front View Instant Center X    : %.2f\n', G.IC_front(1));
fprintf('Front View Instant Center Z    : %.2f\n', G.IC_front(2));
fprintf('Roll Center Height             : %.2f\n', G.roll_center_height);

fprintf('\n');
fprintf('Side View Instant Center Y     : %.2f\n', G.IC_side(1));
fprintf('Side View Instant Center Z     : %.2f\n', G.IC_side(2));
fprintf('Side View Force Line Angle     : %.2f deg\n', G.force_line_angle);

% ------------------------------------------------
% Bump test
% ------------------------------------------------
bumpAmount = 1.0;   % inches

LBJ_new = solveLBJBump(HP, bumpAmount);
UBJ_new = solveUBJBump(HP, LBJ_new);

HPb = HP;
HPb.LBJ = LBJ_new;
HPb.UBJ = UBJ_new;

% update hub from new upright direction
u_new = HPb.UBJ - HPb.LBJ;
u_new = u_new / norm(u_new);

HPb.HUB = HPb.LBJ + HP.lowerKnuckleLength * u_new;

% keep same spindle vector relative to hub
spindleVec = HP.SP1 - HP.HUB;
HPb.SP0 = HPb.HUB;
HPb.SP1 = HPb.HUB + spindleVec;
HPb.WC  = HPb.SP1;

% solve steering arm angle from rack + tie rod
HPb.RACK_RIGHT = HPb.rackCenter + [HPb.rackHalfLength, 0, 0];

HPb.steeringArmAngle = solveSteeringArmAngle( ...
    HPb.HUB, ...
    HPb.RACK_RIGHT, ...
    HPb.steeringArmLength, ...
    HPb.steeringArmDrop, ...
    HPb.tieRodLength, ...
    HP.steeringArmAngle);

% rebuild steering arm endpoint
armVec = [sind(HPb.steeringArmAngle), cosd(HPb.steeringArmAngle), 0];
HPb.STEER_ARM_END = HPb.HUB + HPb.steeringArmLength * armVec + [0, 0, -HPb.steeringArmDrop];

% calculate bumped toe from rack geometry
Gb_temp = calcGeometry(HPb);
toe_bumped = Gb_temp.toe_calc;

% use toe result to rebuild tire direction
forwardDir = [sind(toe_bumped), cosd(toe_bumped), 0];
forwardDir = forwardDir / norm(forwardDir);

% use static camber-to-KPI offset
camber_bumped = Gb_temp.KPI + G.camber_offset_from_KPI;

% rebuild wheel from bumped camber
halfDia = HPb.tireRadius;
dx_cam_bumped = halfDia * tand(camber_bumped);

HPb.WB = HPb.WC + [ dx_cam_bumped, 0, -halfDia];
HPb.WT = HPb.WC + [-dx_cam_bumped, 0,  halfDia];

halfTread = HPb.tireWidth / 2;
HPb.WTF = HPb.WT + halfTread * forwardDir;
HPb.WTR = HPb.WT - halfTread * forwardDir;
HPb.WBF = HPb.WB + halfTread * forwardDir;
HPb.WBR = HPb.WB - halfTread * forwardDir;

% contact patch
contactPatchLength = 4.0;
contactPatchWidth  = HPb.tireWidth * 0.85;

HPb.CP = [HPb.WB(1), HPb.WC(2), 0];

halfPatchL = contactPatchLength / 2;
halfPatchW = contactPatchWidth  / 2;

HPb.CP_FL = HPb.CP + [ halfPatchL,  halfPatchW, 0];
HPb.CP_FR = HPb.CP + [ halfPatchL, -halfPatchW, 0];
HPb.CP_RL = HPb.CP + [-halfPatchL,  halfPatchW, 0];
HPb.CP_RR = HPb.CP + [-halfPatchL, -halfPatchW, 0];

% final bumped geometry
Gb = calcGeometry(HPb);

camberChange = camber_bumped - G.camber;
camberGainPerInch = camberChange / bumpAmount;

toeChange = Gb.toe_calc - G.toe_calc;
toeSteerPerInch = toeChange / bumpAmount;

fprintf('\n');
fprintf('---------------- Camber Gain ------------------------\n');
fprintf('Bump Amount                     : %.2f in\n', bumpAmount);
fprintf('Static Camber                   : %.2f deg\n', G.camber);
fprintf('Bumped Camber                   : %.2f deg\n', camber_bumped);
fprintf('Camber Change                   : %.2f deg\n', camberChange);
fprintf('Camber Gain                     : %.2f deg/in\n', camberGainPerInch);
fprintf('-----------------------------------------------------\n\n');

fprintf('---------------- Toe / Bump Steer -------------------\n');
fprintf('Static Toe                      : %.2f deg\n', G.toe_calc);
fprintf('Bumped Toe                      : %.2f deg\n', Gb.toe_calc);
fprintf('Toe Change                      : %.2f deg\n', toeChange);
fprintf('Toe Steer                       : %.2f deg/in\n', toeSteerPerInch);
fprintf('-----------------------------------------------------\n\n');

% ------------------------------------------------
% Bump sweep
% ------------------------------------------------
bumpRange = linspace(-2, 2, 41);
S = bumpSweep(HP, bumpRange);

fprintf('---------------- Bump Sweep -------------------------\n');
fprintf('Sweep completed from %.2f in to %.2f in with %d points.\n', ...
    bumpRange(1), bumpRange(end), numel(bumpRange));
fprintf('-----------------------------------------------------\n\n');

% ------------------------------------------------
% Plot bump sweep results
% ------------------------------------------------
plotBumpSweep(S);

% ------------------------------------------------
% Rack sweep
% ------------------------------------------------
rackTravelRange = linspace(-1, 1, 41);
R = rackSweep(HP, rackTravelRange);

fprintf('---------------- Rack Sweep -------------------------\n');
fprintf('Sweep completed from %.2f in to %.2f in with %d points.\n', ...
    rackTravelRange(1), rackTravelRange(end), numel(rackTravelRange));
fprintf('Steering Sensitivity Near Center: %.2f deg/in\n', R.toeSensitivityCenter);
fprintf('-----------------------------------------------------\n\n');

plotRackSweep(R);

