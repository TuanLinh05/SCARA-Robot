function q = scara_from_steps(steps, p)
%SCARA_FROM_STEPS Reconstruct joint coordinates [q1, q2, z] from firmware stepper counts.
%
% [EN] Inverse of `scara_motor_coordinates`. Converts an Nx3 matrix of integer
%      stepper pulse counts ordered `[Z_steps, A_steps, B_steps]` back into an
%      Nx3 matrix of joint coordinates `[q1_rad, q2_rad, z_m]`.
%      Removes the differential belt coupling: `q2 = B_rad - coupling * A_rad`.
%
% [VI] Ham nguoc cua `scara_motor_coordinates`. Chuyen doi ma tran Nx3 chua so
%      xung dong co buoc theo thu tu `[Z_steps, A_steps, B_steps]` sang ma tran
%      Nx3 toa do khop `[q1_rad, q2_rad, z_m]`.
%      Loai bo anh huong truyen dong dai vi sai: `q2 = B_rad - coupling * A_rad`.

    % -------------------------------------------------------------------------
    % Step 1: Convert A (column 2) and B (column 3) pulse counts to radians
    % Buoc 1: Doi so xung cua dong co A (cot 2) va B (cot 3) sang goc radian
    % -------------------------------------------------------------------------
    a = deg2rad(steps(:, 2) / p.stepsPerDegree(1));
    b = deg2rad(steps(:, 3) / p.stepsPerDegree(2));

    % -------------------------------------------------------------------------
    % Step 2: Decouple elbow angle q2 and convert Z (column 1) counts to meters
    % Buoc 2: Tach goc khuyu tuong doi q2 va doi so xung Z (cot 1) sang met
    % -------------------------------------------------------------------------
    q1 = a;
    q2 = b - p.coupling * a;
    z = steps(:, 1) / p.stepsPerMeterZ;

    q = [q1, q2, z];
end