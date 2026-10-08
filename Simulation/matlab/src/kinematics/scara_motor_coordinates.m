function [axes,steps] = scara_motor_coordinates(q,p)
% Axes A/B in rad, Z in m. B includes shoulder-to-elbow belt crosstalk.
% These are firmware-equivalent axis coordinates, not rotor shaft angles.
axes=[q(1);q(2)+q(1)/p.elbowCrosstalkRatio;q(3)];
steps=round([rad2deg(axes(1:2)).*p.stepsPerDegree;axes(3)*p.stepsPerMeterZ]);
end
