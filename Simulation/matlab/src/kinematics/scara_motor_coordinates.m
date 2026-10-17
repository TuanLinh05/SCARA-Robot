function [axes, steps] = scara_motor_coordinates(q, p)
%SCARA_MOTOR_COORDINATES Convert joint state q = [q1; q2; z] to motor axes and stepper pulses.
%
% [EN] Because both arm stepper motors (A and B) are mounted on the vertically
%      moving Z carriage rather than on the moving elbow link, rotating the
%      shoulder (q1) pulls the timing belt of the elbow pulley.
%      To keep the relative elbow angle `q2` constant when `q1` moves, motor B
%      must rotate by `B = q2 + coupling * q1` (where `coupling = 1/3`).
%      - `axes`:  [A_rad; B_rad; Z_m] in physical motor coordinates.
%      - `steps`: [Z_steps; A_steps; B_steps] rounded integer pulses in Firmware order.
%
% [VI] Chuyen doi toa do khop `q = [q1; q2; z]` sang goc truc dong co va so xung buoc.
%      Do ca hai dong co tay may (A va B) deu dat co dinh tren cum truot Z (khong dat
%      dong co B tren khop khuyu), khi khop vai `q1` quay se keo theo day dai cua khuyu.
%      De giu goc khuyu tuong doi `q2` khong doi, dong co B phai quay mot goc:
%      `B = q2 + coupling * q1` (voi he so truyen dai vi sai `coupling = 1/3`).
%      - `axes`:  [A_rad; B_rad; Z_m] theo toa do truc dong co.
%      - `steps`: [Z_steps; A_steps; B_steps] so nguyen xung theo thu tu cua Firmware.

    % -------------------------------------------------------------------------
    % Step 1: Apply differential belt coupling to find motor shaft positions
    % Buoc 1: Ap dung bu truyen dong dai vi sai de tinh vi tri truc dong co
    % -------------------------------------------------------------------------
    q = q(:);
    axisA = q(1);
    axisB = q(2) + p.coupling * q(1);
    axisZ = q(3);
    axes = [axisA; axisB; axisZ];

    % -------------------------------------------------------------------------
    % Step 2: Convert physical motor coordinates into integer stepper pulses
    % Buoc 2: Quy doi toa do truc dong co sang so nguyen xung dong co buoc
    % -------------------------------------------------------------------------
    % [EN] Note the firmware axis ordering for steps: [Z_steps; A_steps; B_steps].
    % [VI] Luu y thu tu truc cua mang xung trong Firmware: [Z_steps; A_steps; B_steps].
    zSteps = axes(3) * p.stepsPerMeterZ;
    abSteps = rad2deg(axes(1:2)) .* p.stepsPerDegree;
    steps = round([zSteps; abSteps]);
end