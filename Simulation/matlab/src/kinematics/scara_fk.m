function [xyz, elbow, J] = scara_fk(q, p)
%SCARA_FK Forward kinematics and analytical Jacobian for the 3-DOF SCARA robot.
%
% [EN] Computes the 3D Cartesian brush-tip position `xyz`, the 2D `elbow`
%      joint position, and the 3x3 analytical velocity Jacobian `J = d(xyz)/dq`.
%      Coordinate convention:
%      - `q = [q1; q2; z]` where `q1` is the shoulder angle (rad), `q2` is the
%        relative elbow angle (rad), and `z` is the vertical axis height (m).
%      - Home angle `0 rad` points straight along the `+Y` world axis.
%      - Positive joint rotation is counter-clockwise (from `+Y` toward `-X`),
%        so a link of length L at angle `theta` has vector `L * [-sin(theta); cos(theta)]`.
%
% [VI] Tinh dong hoc thuan (Forward Kinematics) va ma tran Jacobian giai tich 3x3.
%      Quy uoc he toa do:
%      - `q = [q1; q2; z]` voi `q1` la goc khop vai (rad), `q2` la goc khop khuyu
%        tuong doi (rad), va `z` la do cao truc Z (m).
%      - Goc `0 rad` (vi tri HOME) huong doc theo truc `+Y` cua he toa do the gioi.
%      - Chieu quay duong la nguoc chieu kim dong ho (tu `+Y` sang `-X`), do do
%        mot khau dai L o goc `theta` co vector toa do la `L * [-sin(theta); cos(theta)]`.

    % -------------------------------------------------------------------------
    % Step 1: Extract joint angles and absolute orientation of the distal link
    % Buoc 1: Trich xuat goc khop va goc tuyet doi cua khau thu hai (khau khuyu)
    % -------------------------------------------------------------------------
    q = q(:);
    a = q(1);          % [EN] Shoulder angle q1 (rad) / [VI] Goc khop vai q1 (rad)
    b = a + q(2);      % [EN] Absolute elbow orientation q1 + q2 (rad) / [VI] Goc tuyet doi khau 2

    % -------------------------------------------------------------------------
    % Step 2: 2D rotation matrix for the distal link frame (rotated by angle b)
    % Buoc 2: Ma tran xoay 2D cua he toa do gan tren khau 2 (xoay mot goc b)
    % -------------------------------------------------------------------------
    R = [cos(b), -sin(b); ...
         sin(b),  cos(b)];

    % -------------------------------------------------------------------------
    % Step 3: Compute 2D elbow position and 2D distal-to-tip vector
    % Buoc 3: Tinh vi tri khop khuyu 2D va vector tu khop khuyu den dau but ve
    % -------------------------------------------------------------------------
    % [EN] Link 1 starts at `p.base` and extends by `p.L1` in direction `[-sin(a); cos(a)]`.
    % [VI] Khau 1 bat dau tu goc `p.base` va vuon dai `p.L1` theo huong `[-sin(a); cos(a)]`.
    elbow = p.base + p.L1 * [-sin(a); cos(a)];

    % [EN] In the distal link's local frame, link 2 lies along `[0; p.L2]` (+Y_local),
    %      plus any mounting offset `p.toolOffset = [ox; oy]`. Rotating by `R` gives `outer`.
    % [VI] Trong he toa do cuc bo cua khau 2, than khau 2 nam doc theo `[0; p.L2]`,
    %      cong them do lech ga dat but `p.toolOffset = [ox; oy]`. Nhan voi `R` ta duoc `outer`.
    outer = R * (p.toolOffset + [0; p.L2]);

    % -------------------------------------------------------------------------
    % Step 4: Assemble 3D brush-tip Cartesian coordinates [x; y; z]
    % Buoc 4: Tong hop toa do Descartes 3D cua dau but [x; y; z]
    % -------------------------------------------------------------------------
    xyz = [elbow + outer; q(3)];

    % -------------------------------------------------------------------------
    % Step 5: Compute the 3x3 analytical Jacobian matrix J = d(xyz)/dq
    % Buoc 5: Tinh ma tran Jacobian giai tich 3x3 (dao ham rieng cua xyz theo q)
    % -------------------------------------------------------------------------
    % [EN] Differentiating `elbow + outer` with respect to `q1` (a) and `q2`:
    %      d(outer)/da = d(outer)/dq2 = [-outer(2); outer(1)] (90-deg CCW rotation).
    % [VI] Lay dao ham cua `elbow + outer` theo `q1` (a) va `q2`:
    %      d(outer)/da = d(outer)/dq2 = [-outer(2); outer(1)] (quay 90 do nguoc kim dong ho).
    J = [-p.L1 * cos(a) - outer(2), -outer(2), 0; ...
         -p.L1 * sin(a) + outer(1),  outer(1), 0; ...
          0,                         0,        1];
end
