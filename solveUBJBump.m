function UBJ_new = solveUBJBump(HP, LBJ_new)

% Original link lengths
UCA_front_len = norm(HP.UBJ - HP.UCA_FI);
UCA_rear_len  = norm(HP.UBJ - HP.UCA_RI);
upright_len   = norm(HP.UBJ - HP.LBJ);

% Initial guess: move UBJ same vertical amount
dz = LBJ_new(3) - HP.LBJ(3);
UBJ_guess = HP.UBJ + [0 0 dz];

% Solve nonlinear system
f = @(x) constraintUBJ(x,HP,UCA_front_len,UCA_rear_len,LBJ_new,upright_len);

opts = optimoptions('fsolve','Display','off');

UBJ_new = fsolve(f,UBJ_guess,opts);

end