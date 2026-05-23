function C = camberGainSweep(HP)

% bump travel range
travel = linspace(-2, 2, 21);   % inches

n = numel(travel);

C.travel = travel;
C.camber = zeros(1, n);

for i = 1:n
    dz = travel(i);

    % solve suspension in bump/rebound
    LBJ_new = solveLBJBump(HP, dz);
    UBJ_new = solveUBJBump(HP, LBJ_new);
    HPi     = updateOuterAssembly(HP, UBJ_new, LBJ_new);

    % calculate geometry
    Gi = calcGeometry(HPi);

    % store camber
    C.camber(i) = Gi.camber;
end

% approximate camber gain near ride height
mid = ceil(n/2);

if mid > 1 && mid < n
    dCamber = C.camber(mid+1) - C.camber(mid-1);
    dTravel = C.travel(mid+1) - C.travel(mid-1);
    C.camber_gain_rate = dCamber / dTravel;   % deg per inch
else
    C.camber_gain_rate = NaN;
end

end