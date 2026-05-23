function plotCamberGain(C)

figure;
plot(C.travel, C.camber, 'LineWidth', 2);
grid on;

xlabel('Bump Travel (in)');
ylabel('Camber (deg)');
title('Camber vs Bump');

end