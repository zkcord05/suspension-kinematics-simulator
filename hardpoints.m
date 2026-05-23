function HP = hardpoints()

% Coordinate system:
% X = left to right
% Y = front to back
% Z = up and down

% ------------------------------------------------
% Vehicle / wheel package
% ------------------------------------------------
trackWidth = 60;
halfTrack  = trackWidth / 2;

tireRadius = 5;
tireWidth  = 10;

HP.trackWidth = trackWidth;
HP.halfTrack  = halfTrack;
HP.tireRadius = tireRadius;
HP.tireWidth  = tireWidth;

% ------------------------------------------------
% Desired wheel / spindle targets
% ------------------------------------------------
HP.targetCamber  = 2;      % deg
HP.desiredToe    = 3;      % deg, used only to build initial static geometry
HP.spindleLength = 1;      % spindle offset length
HP.spindleAngle  = 2;      % deg relative to ground in front view

% ------------------------------------------------
% Steering arm + rack
% 0 deg steering arm angle = straight forward in +Y
% ------------------------------------------------
HP.steeringArmLength       = 3.0;   % hub to steering arm tip
HP.steeringArmZeroToeAngle = 0  ;   % steering arm angle when toe = 0
HP.steeringArmDrop         = 0.5;   % vertical drop from hub to arm tip

HP.rackCenter     = [0, 0, 3.5];    % centered on vehicle centerline
HP.rackHalfLength = 8.0;            % half rack width to right-side pickup

% ------------------------------------------------
% Chassis design parameters
% ------------------------------------------------
upperChassisWidth = 12;
lowerChassisWidth = 12;

frontArmSpacing = 8;
rearArmSpacing  = -8;

upperPickupHeight = 8;
lowerPickupHeight = 4;

% ------------------------------------------------
% Chassis / suspension box points
% ------------------------------------------------
HP.UCA_FI = [upperChassisWidth, frontArmSpacing, upperPickupHeight];
HP.UCA_RI = [upperChassisWidth, rearArmSpacing,  upperPickupHeight];

HP.LCA_FI = [lowerChassisWidth, frontArmSpacing, lowerPickupHeight];
HP.LCA_RI = [lowerChassisWidth, rearArmSpacing,  lowerPickupHeight];

% ------------------------------------------------
% Centerline-side chassis points
% ------------------------------------------------
HP.UCA_FC = [0, frontArmSpacing, upperPickupHeight];
HP.UCA_RC = [0, rearArmSpacing,  upperPickupHeight];

HP.LCA_FC = [0, frontArmSpacing, lowerPickupHeight];
HP.LCA_RC = [0, rearArmSpacing,  lowerPickupHeight];

% ------------------------------------------------
% Desired suspension targets
% ------------------------------------------------
HP.targetKPI    = 15;   % deg
HP.targetCaster = 5;    % deg

HP.knuckleHeight     = 6.0;
HP.upperKnuckleSplit = 0.58;

HP.upperKnuckleLength = HP.knuckleHeight * HP.upperKnuckleSplit;
HP.lowerKnuckleLength = HP.knuckleHeight - HP.upperKnuckleLength;

% ------------------------------------------------
% Hub / knuckle center guess
% ------------------------------------------------
HP.HUB = [halfTrack - HP.spindleLength, 0, tireRadius];

% ------------------------------------------------
% Steering axis unit vector from KPI and caster
% ------------------------------------------------
ux = -tand(HP.targetKPI);
uy = -tand(HP.targetCaster);
uz = 1;

u = [ux uy uz];
u = u / norm(u);

% ------------------------------------------------
% Solve UBJ and LBJ from hub center
% ------------------------------------------------
HP.UBJ = HP.HUB + HP.upperKnuckleLength * u;
HP.LBJ = HP.HUB - HP.lowerKnuckleLength * u;

% ------------------------------------------------
% Spindle geometry
% ------------------------------------------------
sx = cosd(HP.spindleAngle);
sy = 0;
sz = sind(HP.spindleAngle);

s = [sx sy sz];
s = s / norm(s);

HP.SP0 = HP.HUB;
HP.SP1 = HP.SP0 + HP.spindleLength * s;
HP.WC  = HP.SP1;

% ------------------------------------------------
% Wheel orientation from toe
% ------------------------------------------------
forwardDir = [sind(HP.desiredToe), cosd(HP.desiredToe), 0];
forwardDir = forwardDir / norm(forwardDir);

% ------------------------------------------------
% Wheel line from camber target
% ------------------------------------------------
halfDia = HP.tireRadius;
dx_cam  = halfDia * tand(HP.targetCamber);

HP.WB = HP.WC + [ dx_cam, 0, -halfDia];
HP.WT = HP.WC + [-dx_cam, 0,  halfDia];

% ------------------------------------------------
% Tire outline points with toe
% ------------------------------------------------
halfTread = HP.tireWidth / 2;

HP.WTF = HP.WT + halfTread * forwardDir;
HP.WTR = HP.WT - halfTread * forwardDir;
HP.WBF = HP.WB + halfTread * forwardDir;
HP.WBR = HP.WB - halfTread * forwardDir;

% ------------------------------------------------
% Contact patch
% ------------------------------------------------
contactPatchLength = 4.0;
contactPatchWidth  = HP.tireWidth * 0.85;

HP.CP = [HP.WB(1), HP.WC(2), 0];

halfPatchL = contactPatchLength / 2;
halfPatchW = contactPatchWidth  / 2;

HP.CP_FL = HP.CP + [ halfPatchL,  halfPatchW, 0];
HP.CP_FR = HP.CP + [ halfPatchL, -halfPatchW, 0];
HP.CP_RL = HP.CP + [-halfPatchL,  halfPatchW, 0];
HP.CP_RR = HP.CP + [-halfPatchL, -halfPatchW, 0];

% ------------------------------------------------
% Steering rack
% ------------------------------------------------
HP.RACK_RIGHT = HP.rackCenter + [HP.rackHalfLength, 0, 0];

% ------------------------------------------------
% Initial steering arm endpoint from desired toe
% ------------------------------------------------
HP.steeringArmAngle = HP.steeringArmZeroToeAngle + HP.desiredToe;

armVec = [sind(HP.steeringArmAngle), cosd(HP.steeringArmAngle), 0];
HP.STEER_ARM_END = HP.HUB + HP.steeringArmLength * armVec + [0, 0, -HP.steeringArmDrop];

% ------------------------------------------------
% Fixed tie rod length from initial geometry
% ------------------------------------------------
HP.tieRodLength = norm(HP.STEER_ARM_END - HP.RACK_RIGHT);

end