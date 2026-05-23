function plotRackSweep(R)

[~, i0] = min(abs(R.rackTravel));

figure('Color','k', 'Position', [200 150 900 550]);
plot(R.rackTravel, R.toe, 'LineWidth', 2.5);
hold on;
plot(R.rackTravel(i0), R.toe(i0), 'o', ...
    'MarkerSize', 8, 'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'k');

grid on;
xlabel('Rack Travel (in)', 'Color', 'w');
ylabel('Toe (deg)', 'Color', 'w');
title('Toe vs Rack Travel', 'Color', 'w');

text(R.rackTravel(i0), R.toe(i0), sprintf('  Static: %.2f deg', R.toe(i0)), ...
    'VerticalAlignment', 'bottom', 'FontSize', 11, 'Color', 'w');

text(0.03, 0.95, sprintf('Sensitivity Near Center: %.2f deg/in', R.toeSensitivityCenter), ...
    'Units', 'normalized', ...
    'VerticalAlignment', 'top', ...
    'FontSize', 12, ...
    'Color', 'w');

ax = gca;
set(ax, 'Color', 'k');
ax.XColor = 'w';
ax.YColor = 'w';
ax.GridColor = [0.6 0.6 0.6];
ax.GridAlpha = 0.35;
ax.FontSize = 12;
box on;

yline(0, '--', 'Color', [0.7 0.7 0.7]);

end