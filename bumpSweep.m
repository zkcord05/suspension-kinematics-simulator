function S = bumpSweep(HP, bumpRange)

n = numel(bumpRange);

S.bump = bumpRange;
S.camber = zeros(1, n);
S.toe = zeros(1, n);
S.rollCenter = zeros(1, n);
S.scrub = zeros(1, n);
S.trail = zeros(1, n);

% Static geometry for camber offset reference
G0 = calcGeometry(HP);

for i = 1:n
    bumpAmount = bumpRange(i);

    % --------------------------------------------
    % Solve bumped suspension points
    % --------------------------------------------
    LBJ_new = solveLBJBump(HP, bumpAmount);
    UBJ_new = solveUBJBump(HP, LBJ_new);

    HPb = HP;
    HPb.LBJ = LBJ_new;
    HPb.UBJ = UBJ_new;

    % --------------------------------------------
    % Update hub from new upright direction
    % --------------------------------------------
    u_new = HPb.UBJ - HPb.LBJ;
    u_new = u_new / norm(u_new);

    HPb.HUB = HPb.LBJ + HP.lowerKnuckleLength * u_new;

    % --------------------------------------------
    % Keep same spindle vector relative to hub
    % --------------------------------------------
    spindleVec = HP.SP1 - HP.HUB;
    HPb.SP0 = HPb.HUB;
    HPb.SP1 = HPb.HUB + spindleVec;
    HPb.WC  = HPb.SP1;

    % --------------------------------------------
    % Rebuild rack right point
    % --------------------------------------------
    HPb.RACK_RIGHT = HPb.rackCenter + [HPb.rackHalfLength, 0, 0];

    % --------------------------------------------
    % Solve steering arm angle from fixed tie rod
    % --------------------------------------------
    HPb.steeringArmAngle = solveSteeringArmAngle( ...
        HPb.HUB, ...
        HPb.RACK_RIGHT, ...
        HPb.steeringArmLength, ...
        HPb.steeringArmDrop, ...
        HPb.tieRodLength, ...
        HP.steeringArmAngle);

    armVec = [sind(HPb.steeringArmAngle), cosd(HPb.steeringArmAngle), 0];
    HPb.STEER_ARM_END = HPb.HUB + HPb.steeringArmLength * armVec + [0, 0, -HPb.steeringArmDrop];

    % --------------------------------------------
    % Temporary geometry for updated toe / KPI
    % --------------------------------------------
    Gb_temp = calcGeometry(HPb);

    toe_bumped = Gb_temp.toe_calc;
    camber_bumped = Gb_temp.KPI + G0.camber_offset_from_KPI;

    % --------------------------------------------
    % Rebuild wheel from bumped camber
    % --------------------------------------------
    halfDia = HPb.tireRadius;
    dx_cam_bumped = halfDia * tand(camber_bumped);

    HPb.WB = HPb.WC + [ dx_cam_bumped, 0, -halfDia];
    HPb.WT = HPb.WC + [-dx_cam_bumped, 0,  halfDia];

    % --------------------------------------------
    % Rebuild tire outline with bumped toe
    % --------------------------------------------
    forwardDir = [sind(toe_bumped), cosd(toe_bumped), 0];
    forwardDir = forwardDir / norm(forwardDir);

    halfTread = HPb.tireWidth / 2;

    HPb.WTF = HPb.WT + halfTread * forwardDir;
    HPb.WTR = HPb.WT - halfTread * forwardDir;
    HPb.WBF = HPb.WB + halfTread * forwardDir;
    HPb.WBR = HPb.WB - halfTread * forwardDir;

    % --------------------------------------------
    % Rebuild contact patch
    % --------------------------------------------
    contactPatchLength = 4.0;
    contactPatchWidth  = HPb.tireWidth * 0.85;

    HPb.CP = [HPb.WB(1), HPb.WC(2), 0];

    halfPatchL = contactPatchLength / 2;
    halfPatchW = contactPatchWidth  / 2;

    HPb.CP_FL = HPb.CP + [ halfPatchL,  halfPatchW, 0];
    HPb.CP_FR = HPb.CP + [ halfPatchL, -halfPatchW, 0];
    HPb.CP_RL = HPb.CP + [-halfPatchL,  halfPatchW, 0];
    HPb.CP_RR = HPb.CP + [-halfPatchL, -halfPatchW, 0];

    % --------------------------------------------
    % Final geometry at this bump value
    % --------------------------------------------
    Gb = calcGeometry(HPb);

    S.camber(i)     = camber_bumped;
    S.toe(i)        = Gb.toe_calc;
    S.rollCenter(i) = Gb.roll_center_height;
    S.scrub(i)      = Gb.scrub_radius_signed;
    S.trail(i)      = Gb.mechanical_trail_signed;
end

end