function meshes = scara_meshes(p)
%SCARA_MESHES Load 3D STL CAD meshes and build proxy geometry for each robot link group.
%
% [EN] Assembles the 3D visual meshes for the SCARA robot in local link coordinates (meters):
%      - Group 0: Static frame (guide rods, leadscrew, aluminium extrusion uprights/crossbars).
%      - Group 1: Z carriage (motor plates, NEMA-17 stepper proxies, shoulder bearing housing).
%      - Group 2: Shoulder link L1 (top/bottom plates, side walls, elbow shaft proxy).
%      - Group 3: Elbow link L2 (top/bottom plates, side walls, distal rod hub).
%      - Group 4: Brush tool at the end-effector tip (clamp, handle, deformable bristles).
%
% [VI] Tai cac luoi 3D STL tu CAD va tao hinh khoi dai dien (proxy) cho tung nhom khau (don vi met):
%      - Nhom 0: Khung tinh (thanh truot, vit me Tr8, thanh nhom dinh hinh dung va ngang).
%      - Nhom 1: Cum truot truc Z (de gan dong co, khoi dong co 1 & 2, o bi truc vai).
%      - Nhom 2: Khau vai L1 (tam tren/duoi, vach ben trai/phai, truc khop khuyu).
%      - Nhom 3: Khau khuyu L2 (tam tren/duoi, vach ben trai/phai, dau giu truc).
%      - Nhom 4: Cum co ve tai dau cong tac (kep giu but, can but, dau long co).

    % -------------------------------------------------------------------------
    % Step 1: Define RGB material colors and initialize empty mesh struct array
    % Buoc 1: Dinh nghia bảng mau RGB va khoi tao mang cau truc luoi 3D rong
    % -------------------------------------------------------------------------
    black = [0.22 0.25 0.28];
    orange = [0.96 0.47 0.08];
    metal = [0.65 0.71 0.75];

    meshes = struct( ...
        'name', {}, ...
        'source', {}, ...
        'group', {}, ...
        'vertices', {}, ...
        'faces', {}, ...
        'color', {});

    % -------------------------------------------------------------------------
    % Step 2: Load original STL arm plates, motor mounts, and side walls
    % Buoc 2: Tai cac file STL tam khep, de dong co va vach ben cua tay may
    % -------------------------------------------------------------------------
    % [EN] Bore XY centers were inferred from mesh circles; Z stack is an assembly estimate.
    % [VI] Tam lo XY duoc suy ra tu duong tron tren luoi STL; do cao Z uoc luong theo lap rap.
    addstl('Arm_Shoulder_Plate_Top',    2, [-1 -1  1], [0 49 97],         black);
    addstl('Arm_Shoulder_Plate_Bottom', 2, [-1 -1  1], [0 44.5 70],       black);
    addstl('Arm_Elbow_Plate_Top',       3, [-1 -1  1], [0 43.327927 67],  black);
    addstl('Arm_Elbow_Plate_Bottom',    3, [ 1  1  1], [0 43.327927 40],  black);
    addstl('Arm_Base_MotorPlate',       1, [-1 -1  1], [36 -3 70],        black);
    addstl('Arm_Base_MotorPlate',       1, [-1 -1 -1], [36 -3 69],        black);

    % [EN] Mirrored side walls placed in the middle of each 98 mm link.
    % [VI] Cac vach ben doi xung dat o giua moi khau dai 98 mm.
    addstl('Arm_SideWall_Left',  2, [ 1 1 1], [ 14 46 79],        orange);
    addstl('Arm_SideWall_Right', 2, [-1 1 1], [-14 46 79],        orange);
    addstl('Arm_SideWall_Left',  3, [ 1 1 1], [ 14 46 49],        orange);
    addstl('Arm_SideWall_Right', 3, [-1 1 1], [-14 46 49],        orange);
    addstl('Arm_Elbow_RodHub',   3, [ 1 1 1], [0 p.L2 * 1000 49], orange);

    % -------------------------------------------------------------------------
    % Step 3: Add proxy cylinders and boxes for frame, motors, shafts, and brush
    % Buoc 3: Them cac khoi tru va hop dai dien cho khung, dong co, truc va co ve
    % -------------------------------------------------------------------------
    % [EN] Static frame (group 0): guide rods, Tr8 leadscrew, and aluminium extrusions.
    % [VI] Khung tinh (nhom 0): 3 thanh truot 8 mm, vit me Tr8 va khung nhom dinh hinh.
    for pos = [-38 0; 110 0; 36 -39]'
        addcylinder('8 mm guide rod', 0, [pos' / 1000, 0.01], 0.004, 0.30, metal);
    end
    addcylinder('Tr8 leadscrew proxy', 0, [0.036 0 0.01], 0.004, 0.25, black);
    addbox('Aluminium upright',     0, [-0.052 -0.035 0],    [0.02 0.04 0.30],  metal);
    addbox('Aluminium upright',     0, [ 0.104 -0.035 0],    [0.02 0.04 0.30],  metal);
    addbox('Frame crossbar bottom', 0, [-0.052 -0.035 0],    [0.176 0.02 0.02], metal);
    addbox('Frame crossbar top',    0, [-0.052 -0.035 0.28], [0.176 0.02 0.02], metal);

    % [EN] Z carriage (group 1) & shoulder/elbow shafts (groups 1 & 2).
    % [VI] Cum truot Z (nhom 1) & truc khop vai/khuyu (nhom 1 & 2).
    addbox('Motor 1 proxy', 1, [-0.021 0.004 0.019], [0.042 0.042 0.040], black);
    addbox('Motor 2 proxy', 1, [ 0.051 0.004 0.019], [0.042 0.042 0.040], black);
    addcylinder('Shoulder bearing/shaft proxy', 1, [0 0 0.059],    0.013, 0.049, metal);
    addcylinder('Elbow shaft proxy',            2, [0 p.L1 0.031], 0.004, 0.075, metal);

    % [EN] Brush tool (group 4): clamp, wooden handle, and tapered conical bristles.
    % [VI] Cum but/co ve (nhom 4): kep giu, can go va dau long co hinh non cut.
    addcylinder('Brush clamp concept',  4, [0 0 0.034], 0.011,  0.015, orange);
    addcylinder('Brush handle concept', 4, [0 0 0.008], 0.0045, 0.082, [0.31 0.22 0.12]);

    [X, Y, Z] = cylinder([0.001 0.0045], 24);
    Z = Z * 0.008;
    [F, V] = surf2patch(X, Y, Z, 'triangles');
    addmesh('Brush bristles concept', 'proxy', 4, V, F, [0.08 0.09 0.12]);

    % =========================================================================
    % NESTED HELPER FUNCTIONS / CAC HAM CON HO TRO TAO LUOI 3D
    % =========================================================================
    function addstl(stem, group, sgn, translation, color)
        % [EN] Locate STL file under Hardware/arm/STL, transform mm -> m, and scale if L1/L2 changed.
        % [VI] Tim file STL trong Hardware/arm/STL, doi mm -> m, va co gian neu thay doi L1/L2.
        folder = fullfile(p.hardwareRoot, 'arm', 'STL');
        file = dir(fullfile(folder, '**', [stem '_*.stl']));
        assert(numel(file) == 1, 'scara:mesh', 'Expected one STL matching %s.', stem);
        source = fullfile(file.folder, file.name);

        [V, F] = scara_stl(source);
        V = (V .* sgn + translation) * 0.001;

        % [EN] Visual-only link stretching when nominal lengths L1 or L2 are modified.
        % [VI] Co gian hinh anh truc quan khi nguoi dung thay doi chieu dai danh dinh L1, L2.
        if group == 2
            V(:, 2) = V(:, 2) * p.L1 / 0.098;
        end
        if group == 3 && ~strcmp(stem, 'Arm_Elbow_RodHub')
            V(:, 2) = V(:, 2) * p.L2 / 0.098;
        end
        addmesh(stem, source, group, V, F, color);
    end

    function addmesh(name, source, group, V, F, color)
        meshes(end + 1) = struct( ...
            'name', name, ...
            'source', source, ...
            'group', group, ...
            'vertices', V, ...
            'faces', F, ...
            'color', color);
    end

    function addcylinder(name, group, pos, radius, height, color)
        [X, Y, Z] = cylinder(radius, 24);
        Z = Z * height;
        [F, V] = surf2patch(X, Y, Z, 'triangles');
        V = V + pos;
        addmesh(name, 'proxy', group, V, F, color);
    end

    function addbox(name, group, pos, dim, color)
        unitCubeVertices = [ ...
            0 0 0; 1 0 0; 1 1 0; 0 1 0; ...
            0 0 1; 1 0 1; 1 1 1; 0 1 1 ...
        ];
        V = unitCubeVertices .* dim + pos;
        F = [ ...
            1 2 3; 1 3 4; 5 7 6; 5 8 7; ...
            1 5 6; 1 6 2; 2 6 7; 2 7 3; ...
            3 7 8; 3 8 4; 4 8 5; 4 5 1 ...
        ];
        addmesh(name, 'proxy', group, V, F, color);
    end
end
