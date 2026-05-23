function LBJ_new = solveLBJBump(HP, dz)

% Lower control arm midpoint
LCA_mid = (HP.LCA_FI + HP.LCA_RI) / 2;

% Original LCA length
LCA_len = norm(HP.LBJ - LCA_mid);

% Initial guess: move LBJ straight upward
LBJ_guess = HP.LBJ + [0 0 dz];

% Direction from midpoint to guessed point
dir = LBJ_guess - LCA_mid;
dir = dir / norm(dir);

% Project back to maintain arm length
LBJ_new = LCA_mid + LCA_len * dir;

end