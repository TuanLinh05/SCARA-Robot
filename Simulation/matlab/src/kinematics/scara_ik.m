function q = scara_ik(xyz, p, branch, checkLimits)
%SCARA_IK Closed-form analytical Inverse Kinematics (IK) for the 3-DOF SCARA robot.
%
% [EN] Computes joint coordinates `q = [q1; q2; z]` from a desired Cartesian
%      brush-tip target `xyz = [x; y; z]`, accounting for the 2D tool offset.
%      - `branch = +1`: default working elbow configuration (`q2_eff >= 0`).
%      - `branch = -1`: alternate mirrored elbow configuration (`q2_eff <= 0`).
%      - `checkLimits`: if true, raises an error when `q` exceeds joint/Z limits.
%
% [VI] Giai bai toan Dong hoc nguoc (Inverse Kinematics - IK) bang phuong phap
%      giai tich dong cho robot SCARA 3 bac tu do, co tinh den do lech dau but 2D.
%      - `branch = +1`: nhanh khuyu tay mac dinh (`q2_eff >= 0`).
%      - `branch = -1`: nhanh khuyu tay doi xung (`q2_eff <= 0`).
%      - `checkLimits`: neu bang true, bao loi khi goc khop vuot gioi han cho phep.

    % -------------------------------------------------------------------------
    % Step 1: Set default optional arguments and validate inputs
    % Buoc 1: Gan gia tri mac dinh cho cac tham so tuy chon va kiem tra dau vao
    % -------------------------------------------------------------------------
    if nargin < 3
        branch = 1;
    end
    if nargin < 4
        checkLimits = true;
    end

    xyz = xyz(:);
    assert(numel(xyz) == 3 && all(isfinite(xyz)), 'scara:target', ...
        'Target must be three finite values in metres.');
    assert(abs(branch) == 1, 'scara:branch', 'Branch must be +1 or -1.');

    % -------------------------------------------------------------------------
    % Step 2: Convert link 2 + tool offset into an effective virtual link (L, delta)
    % Buoc 2: Quy doi khau 2 + do lech dau but thanh khau ao hieu dung (L, delta)
    % -------------------------------------------------------------------------
    % [EN] In the distal link frame, the brush tip is at `v = [ox; L2 + oy]`.
    %      This acts like a single virtual second link of length `L = norm(v)`
    %      and fixed angular shift `delta = atan2(-ox, L2 + oy)` relative to +Y_local.
    % [VI] Trong he toa do khau 2, mui but nam tai `v = [ox; L2 + oy]`.
    %      Ta coi to hop nay nhu mot khau thu hai ao co chieu dai hieu dung `L = norm(v)`
    %      va goc lech co dinh `delta = atan2(-ox, L2 + oy)` so voi truc +Y cuc bo.
    v = p.toolOffset + [0; p.L2];
    L = norm(v);
    delta = atan2(-v(1), v(2));

    % -------------------------------------------------------------------------
    % Step 3: Law of Cosines on the triangle (Base -> Elbow -> Brush Tip)
    % Buoc 3: Ap dung Dinh ly Cosin cho tam giac (Goc Base -> Khuyu -> Dau but)
    % -------------------------------------------------------------------------
    % [EN] Distance from shoulder base to target XY: r^2 = L1^2 + L^2 + 2*L1*L*cos(e).
    % [VI] Khoang cach tu tam khop vai den diem dich XY: r^2 = L1^2 + L^2 + 2*L1*L*cos(e).
    xy = xyz(1:2) - p.base;
    r = norm(xy);
    c = (r * r - p.L1^2 - L^2) / (2 * p.L1 * L);

    if abs(c) > 1 + 1e-10 || r < 1e-10
        error('scara:unreachable', 'Target outside reachable workspace or at undefined folded origin.');
    end

    % -------------------------------------------------------------------------
    % Step 4: Solve for effective elbow angle `e` and check kinematic singularity
    % Buoc 4: Tinh goc khuyu hieu dung `e` va kiem tra diem suy bien dong hoc
    % -------------------------------------------------------------------------
    % [EN] Clamp `c` to [-1, 1] to guard against tiny floating-point roundoff near boundaries.
    % [VI] Gioi han `c` trong doan [-1, 1] de tranh sai so dau phay dong sat biên.
    clampedC = max(-1, min(1, c));
    e = branch * acos(clampedC);

    if abs(sin(e)) < p.singularityMargin
        error('scara:singular', 'Target too close to a singular configuration.');
    end

    % -------------------------------------------------------------------------
    % Step 5: Solve for shoulder angle `a` (q1) and true elbow angle `q2`
    % Buoc 5: Tinh goc khop vai `a` (q1) va goc khop khuyu thuc `q2`
    % -------------------------------------------------------------------------
    % [EN] `atan2(-xy(1), xy(2))` is the target azimuth measured from `+Y` toward `-X`.
    %      Subtracting the internal triangle angle at the shoulder yields `a`,
    %      which is then wrapped into `[-pi, pi]`.
    % [VI] `atan2(-xy(1), xy(2))` la goc phuong vi cua diem dich tinh tu `+Y` sang `-X`.
    %      Tru di goc trong cua tam giac tai khop vai ta duoc `a`, sau do chuan hoa ve `[-pi, pi]`.
    targetAngle = atan2(-xy(1), xy(2));
    shoulderTriangleAngle = atan2(L * sin(e), p.L1 + L * cos(e));
    a = targetAngle - shoulderTriangleAngle;
    a = atan2(sin(a), cos(a));

    % [EN] Subtract the fixed tool offset angle `delta` to recover the physical joint angle `q2`.
    % [VI] Tru di goc lech dau but `delta` de thu duoc goc khop khuyu vat ly `q2`.
    q = [a; e - delta; xyz(3)];

    % -------------------------------------------------------------------------
    % Step 6: Verify joint and Z travel limits (if requested)
    % Buoc 6: Kiem tra gioi han hanh trinh khop quay va truc Z (neu bat checkLimits)
    % -------------------------------------------------------------------------
    if checkLimits
        jointOutOfBounds = any(q(1:2) < p.jointLimits(:, 1)) || any(q(1:2) > p.jointLimits(:, 2));
        zOutOfBounds = q(3) < p.zLimits(1) || q(3) > p.zLimits(2);
        if jointOutOfBounds || zOutOfBounds
            error('scara:limits', 'Target violates the configured (assumed) joint or Z limits.');
        end
    end
end
