function armAngle = solveSteeringArmAngle(HUB, RACK_RIGHT, armLength, armDrop, tieRodLength, angleGuess)

f = @(ang) tieRodError(ang, HUB, RACK_RIGHT, armLength, armDrop, tieRodLength);

opts = optimset('Display','off');
armAngle = fzero(f, angleGuess, opts);

end

function err = tieRodError(ang, HUB, RACK_RIGHT, armLength, armDrop, tieRodLength)

armVec = [sind(ang), cosd(ang), 0];
steerArmEnd = HUB + armLength * armVec + [0, 0, -armDrop];

err = norm(steerArmEnd - RACK_RIGHT) - tieRodLength;

end