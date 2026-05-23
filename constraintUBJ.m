function F = constraintUBJ(x,HP,UCA_front_len,UCA_rear_len,LBJ_new,upright_len)

F = zeros(3,1);

% UCA front link
F(1) = norm(x - HP.UCA_FI) - UCA_front_len;

% UCA rear link
F(2) = norm(x - HP.UCA_RI) - UCA_rear_len;

% Upright
F(3) = norm(x - LBJ_new) - upright_len;

end