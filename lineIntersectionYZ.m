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