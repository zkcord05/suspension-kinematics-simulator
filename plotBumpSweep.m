function plotBumpSweep(S)

% Static ride-height index (closest to bump = 0)
[~, i0] = min(abs(S.bump));

figure('Color','k', 'Position', [100 100 1200 700]);
tiledlayout(2,3, 'TileSpacing', 'compact', 'Padding', 'compact');

sgtitle('Front Right Suspension Bump Sweep Results', ...
    'FontSize', 16, 'FontWeight', 'bold', 'Color', 'w');

% -----------------------------
% Camber vs Bump
% -----------------------------
nexttile;
plot(S.bump, S.camber, 'LineWidth', 2.2);
hold on;
plot(S.bump(i0), S.camber(i0), 'o', ...
    'MarkerSize', 7, 'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'k');
yline(0, '--', 'Color', [0.7 0.7 0.7]);
grid on;
xlabel('Wheel Travel (in)', 'Color', 'w');
ylabel('Camber (deg)', 'Color', 'w');
title('Camber vs Bump', 'Color', 'w');
text(S.bump(i0), S.camber(i0), sprintf('  Static: %.2f deg', S.camber(i0)), ...
    'VerticalAlignment', 'bottom', 'FontSize', 10, 'Color', 'w');
styleAxesDark(gca);

% -----------------------------
% Toe vs Bump
% -----------------------------
nexttile;
plot(S.bump, S.toe, 'LineWidth', 2.2);
hold on;
plot(S.bump(i0), S.toe(i0), 'o', ...
    'MarkerSize', 7, 'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'k');
yline(0, '--', 'Color', [0.7 0.7 0.7]);
grid on;
xlabel('Wheel Travel (in)', 'Color', 'w');
ylabel('Toe (deg)', 'Color', 'w');
title('Toe vs Bump', 'Color', 'w');
text(S.bump(i0), S.toe(i0), sprintf('  Static: %.2f deg', S.toe(i0)), ...
    'VerticalAlignment', 'bottom', 'FontSize', 10, 'Color', 'w');
styleAxesDark(gca);

% -----------------------------
% Roll Center Height vs Bump
% -----------------------------
nexttile;
plot(S.bump, S.rollCenter, 'LineWidth', 2.2);
hold on;
plot(S.bump(i0), S.rollCenter(i0), 'o', ...
    'MarkerSize', 7, 'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'k');
grid on;
xlabel('Wheel Travel (in)', 'Color', 'w');
ylabel('Roll Center Height (in)', 'Color', 'w');
title('Roll Center Height vs Bump', 'Color', 'w');
text(S.bump(i0), S.rollCenter(i0), sprintf('  Static: %.2f in', S.rollCenter(i0)), ...
    'VerticalAlignment', 'bottom', 'FontSize', 10, 'Color', 'w');
styleAxesDark(gca);

% -----------------------------
% Scrub Radius vs Bump
% -----------------------------
nexttile;
plot(S.bump, S.scrub, 'LineWidth', 2.2);
hold on;
plot(S.bump(i0), S.scrub(i0), 'o', ...
    'MarkerSize', 7, 'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'k');
yline(0, '--', 'Color', [0.7 0.7 0.7]);
grid on;
xlabel('Wheel Travel (in)', 'Color', 'w');
ylabel('Scrub Radius (in)', 'Color', 'w');
title('Scrub Radius vs Bump', 'Color', 'w');
text(S.bump(i0), S.scrub(i0), sprintf('  Static: %.2f in', S.scrub(i0)), ...
    'VerticalAlignment', 'bottom', 'FontSize', 10, 'Color', 'w');
styleAxesDark(gca);

% -----------------------------
% Mechanical Trail vs Bump
% -----------------------------
nexttile;
plot(S.bump, S.trail, 'LineWidth', 2.2);
hold on;
plot(S.bump(i0), S.trail(i0), 'o', ...
    'MarkerSize', 7, 'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'k');
yline(0, '--', 'Color', [0.7 0.7 0.7]);
grid on;
xlabel('Wheel Travel (in)', 'Color', 'w');
ylabel('Mechanical Trail (in)', 'Color', 'w');
title('Mechanical Trail vs Bump', 'Color', 'w');
text(S.bump(i0), S.trail(i0), sprintf('  Static: %.2f in', S.trail(i0)), ...
    'VerticalAlignment', 'bottom', 'FontSize', 10, 'Color', 'w');
styleAxesDark(gca);

% -----------------------------
% Summary panel
% -----------------------------
nexttile;
axis off;
set(gca, 'Color', 'k');

% Approximate local slopes near ride height
if i0 > 1 && i0 < numel(S.bump)
    camberSlope = (S.camber(i0+1) - S.camber(i0-1)) / (S.bump(i0+1) - S.bump(i0-1));
    toeSlope    = (S.toe(i0+1)    - S.toe(i0-1))    / (S.bump(i0+1) - S.bump(i0-1));
else
    camberSlope = NaN;
    toeSlope    = NaN;
end

summaryText = {
    'Summary at Ride Height'
    ' '
    sprintf('Static Camber: %.2f deg', S.camber(i0))
    sprintf('Static Toe: %.2f deg', S.toe(i0))
    sprintf('Static Roll Center: %.2f in', S.rollCenter(i0))
    sprintf('Static Scrub Radius: %.2f in', S.scrub(i0))
    sprintf('Static Mechanical Trail: %.2f in', S.trail(i0))
    ' '
    sprintf('Camber Gain Near RH: %.2f deg/in', camberSlope)
    sprintf('Toe Steer Near RH: %.2f deg/in', toeSlope)
};

text(0.02, 0.95, summaryText, ...
    'Units', 'normalized', ...
    'VerticalAlignment', 'top', ...
    'FontSize', 11, ...
    'Color', 'w');

end

function styleAxesDark(ax)
set(ax, 'Color', 'k');
ax.XColor = 'w';
ax.YColor = 'w';
ax.GridColor = [0.6 0.6 0.6];
ax.GridAlpha = 0.35;
ax.MinorGridColor = [0.4 0.4 0.4];
ax.FontSize = 11;
box on;
end