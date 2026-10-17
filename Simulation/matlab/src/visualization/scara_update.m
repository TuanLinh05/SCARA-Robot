function scara_update(scene, q)
%SCARA_UPDATE Update 3D mesh vertex positions for a given joint configuration `q`.
%
% [EN] Transforms every mesh in `scene.meshes` from its local link frame (in meters)
%      into the world frame (in millimeters) according to its kinematic group:
%      - Group 0 (Static frame):     translated by `[base_x, base_y, 0]`.
%      - Group 1 (Z carriage):       translated vertically by `[base_x, base_y, z]`.
%      - Group 2 (Shoulder link L1): rotated by `q1` (`Ra`), translated to `[base_x, base_y, z]`.
%      - Group 3 (Elbow link L2):    rotated by `q1 + q2` (`Rb`), translated to `[elbow_x, elbow_y, z]`.
%      - Group 4 (Brush tool):       rotated by `q1 + q2` (`Rb`), translated to `[tip_x, tip_y, tip_z]`.
%                                    Bristle vertices are clamped at `z >= paperZ` to visualize contact.
%
% [VI] Cap nhat toa do dinh 3D cua tat ca cac khoi luoi trong `scene.meshes` tu he toa do
%      cuc bo (met) sang he toa do the gioi (milimet) theo nhom dong hoc:
%      - Nhom 0 (Khung tinh):     tinh tien theo `[base_x, base_y, 0]`.
%      - Nhom 1 (Cum truot Z):    tinh tien doc truc Z theo `[base_x, base_y, z]`.
%      - Nhom 2 (Khau vai L1):    xoay góc `q1` (`Ra`), tinh tien den `[base_x, base_y, z]`.
%      - Nhom 3 (Khau khuyu L2):  xoay goc `q1 + q2` (`Rb`), tinh tien den `[elbow_x, elbow_y, z]`.
%      - Nhom 4 (Cum co ve):      xoay goc `q1 + q2` (`Rb`), tinh tien den `[tip_x, tip_y, tip_z]`.
%                                 Cac dinh long co duoc gioi han `z >= paperZ` khi cham mat giay.

    % -------------------------------------------------------------------------
    % Step 1: Compute forward kinematics and 3D rotation matrices about +Z
    % Buoc 1: Tinh dong hoc thuan va cac ma tran xoay 3D quanh truc +Z
    % -------------------------------------------------------------------------
    p = scene.p;
    [tip, elbow] = scara_fk(q, p);
    a = q(1);
    b = a + q(2);

    Ra = [cos(a), -sin(a), 0; ...
          sin(a),  cos(a), 0; ...
          0,       0,      1];

    Rb = [cos(b), -sin(b), 0; ...
          sin(b),  cos(b), 0; ...
          0,       0,      1];

    % -------------------------------------------------------------------------
    % Step 2: Transform vertices of each rigid-body group and update patches (mm)
    % Buoc 2: Bien doi toa do dinh cua tung nhom khau va cap nhat do hoa (mm)
    % -------------------------------------------------------------------------
    % [EN] Since `V` is an Nx3 matrix of row vectors, rotating each vertex by `R`
    %      is written as `V * R'` (equivalent to `(R * V')'`).
    % [VI] Vi `V` la ma tran Nx3 gom cac vector hang, phep xoay moi dinh theo ma tran `R`
    %      duoc tinh bang `V * R'` (tuong duong voi `(R * V')'`).
    for k = 1:numel(scene.meshes)
        m = scene.meshes(k);
        V = m.vertices;

        switch m.group
            case 0
                V = V + [p.base', 0];
            case 1
                V = V + [p.base', q(3)];
            case 2
                V = V * Ra' + [p.base', q(3)];
            case 3
                V = V * Rb' + [elbow', q(3)];
            case 4
                V = V * Rb' + tip';
                if strcmp(m.name, 'Brush bristles concept')
                    V(:, 3) = max(p.paperZ, V(:, 3));
                end
        end

        set(scene.handles(k), 'Vertices', V * 1000);
    end

    % -------------------------------------------------------------------------
    % Step 3: Update the 3D brush-tip marker position (mm)
    % Buoc 3: Cap nhat vi tri dau cham bieu thi mui but trong khong gian 3D (mm)
    % -------------------------------------------------------------------------
    set(scene.tip, ...
        'XData', tip(1) * 1000, ...
        'YData', tip(2) * 1000, ...
        'ZData', tip(3) * 1000);
end
