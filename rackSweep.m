function R = rackSweep(HP, rackTravelRange)

n = numel(rackTravelRange);

R.rackTravel = rackTravelRange;
R.toe = zeros(1, n);
R.steeringArmAngle = zeros(1, n);

for i = 1:n
    dxRack = rackTravelRange(i);

    HPi = HP;

    % Move right-side rack pickup left/right with rack travel
    HPi.RACK_RIGHT = HPi.rackCenter + [HPi.rackHalfLength + dxRack, 0, 0];

    % Solve steering arm angle from fixed tie rod length
    HPi.steeringArmAngle = solveSteeringArmAngle( ...
        HPi.HUB, ...
        HPi.RACK_RIGHT, ...
        HPi.steeringArmLength, ...
        HPi.steeringArmDrop, ...
        HPi.tieRodLength, ...
        HP.steeringArmAngle);

    % Rebuild steering arm endpoint
    armVec = [sind(HPi.steeringArmAngle), cosd(HPi.steeringArmAngle), 0];
    HPi.STEER_ARM_END = HPi.HUB + HPi.steeringArmLength * armVec + [0, 0, -HPi.steeringArmDrop];

    % Calculate geometry with new steering angle
    Gi = calcGeometry(HPi);

    R.toe(i) = Gi.toe_calc;
    R.steeringArmAngle(i) = Gi.steering_arm_angle_calc;
end

% --------------------------------------------
% Steering sensitivity near center
% --------------------------------------------
[~, i0] = min(abs(R.rackTravel));

if i0 > 1 && i0 < n
    R.toeSensitivityCenter = ...
        (R.toe(i0+1) - R.toe(i0-1)) / (R.rackTravel(i0+1) - R.rackTravel(i0-1));
else
    R.toeSensitivityCenter = NaN;
end

end