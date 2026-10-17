function scene = scara_scene(ax, p)
%SCARA_SCENE Initialize the 3D axes, drawing paper plane, and STL mesh patch handles.
%
% [EN] Creates all 3D `patch` and `plot3` graphics objects once in the given axes
%      `ax` and poses the robot at `p.parkQ`. Subsequent animation frames call
%      `scara_update(scene, q)` to update only vertex coordinates for fast rendering.
%
% [VI] Khoi tao truc toa do 3D, mat phang giay ve va cac doi tuong do hoa `patch`
%      cua luoi STL mot lan duy nhat tren truc `ax`, sau do dat robot o tu the
%      do `p.parkQ`. Trong qua trinh mo phong, chi can goi `scara_update(scene, q)`
%      de cap nhat toa do dinh giup hoat anh muot va nhanh.

    % -------------------------------------------------------------------------
    % Step 1: Configure 3D axes appearance, viewing angle, and workspace bounds
    % Buoc 1: Cau hinh giao dien truc 3D, goc nhin camera va gioi han khung nhin
    % -------------------------------------------------------------------------
    f = ancestor(ax, 'figure');
    if isprop(f, 'Theme')
        f.Theme = 'light';
    end

    cla(ax);
    hold(ax, 'on');
    axis(ax, 'equal');
    grid(ax, 'on');
    set(ax, 'Color', [0.96 0.97 0.98]);
    view(ax, 145, 27);

    xlabel(ax, 'X (mm)');
    ylabel(ax, 'Y (mm)');
    zlabel(ax, 'Z (mm)');
    xlim(ax, [-180 180]);
    ylim(ax, [-70 240]);
    zlim(ax, [0 350]);

    % -------------------------------------------------------------------------
    % Step 2: Draw the rectangular drawing paper sheet at height `p.paperZ`
    % Buoc 2: Ve mat phang to giay hinh chu nhat tai do cao `p.paperZ` (don vi mm)
    % -------------------------------------------------------------------------
    paper = p.paperHalfSize * 1000;
    paperX = p.paperCenter(1) * 1000 + [-paper(1),  paper(1), paper(1), -paper(1)];
    paperY = p.paperCenter(2) * 1000 + [-paper(2), -paper(2), paper(2),  paper(2)];
    paperZ = ones(1, 4) * p.paperZ * 1000;

    patch(ax, paperX, paperY, paperZ, [1 1 1], 'EdgeColor', [0.6 0.65 0.7]);
    text(ax, p.paperCenter(1) * 1000 - paper(1), ...
        p.paperCenter(2) * 1000 + paper(2), ...
        p.paperZ * 1000 + 2, ...
        'Paper plane; measured Z input required', 'FontSize', 9);

    % -------------------------------------------------------------------------
    % Step 3: Create STL and proxy mesh patches (once) with Gouraud lighting
    % Buoc 3: Tao cac doi tuong luoi 3D (STL va khoi dai dien) voi anh sang Gouraud
    % -------------------------------------------------------------------------
    m = scara_meshes(p);
    h = gobjects(numel(m), 1);
    for k = 1:numel(m)
        h(k) = patch(ax, ...
            'Faces', m(k).faces, ...
            'Vertices', m(k).vertices * 1000, ...
            'FaceColor', m(k).color, ...
            'EdgeColor', 'none', ...
            'FaceLighting', 'gouraud');
    end
    camlight(ax, 'headlight');
    material(ax, 'dull');

    % -------------------------------------------------------------------------
    % Step 4: Create 3D trajectory trace handles and pose robot at parkQ
    % Buoc 4: Tao cac duong quy dao 3D (muc tieu, net muc, mui but) va cap nhat parkQ
    % -------------------------------------------------------------------------
    scene = struct('ax', ax, 'meshes', m, 'handles', h, 'p', p);
    scene.target = plot3(ax, nan, nan, nan, '--', 'Color', [0.15 0.55 0.80], 'LineWidth', 1);
    scene.ink = plot3(ax, nan, nan, nan, '-', 'Color', [0.15 0.17 0.2], 'LineWidth', 1.6);
    scene.tip = plot3(ax, nan, nan, nan, 'o', 'MarkerFaceColor', [0.95 0.3 0.1], 'MarkerSize', 5);

    scara_update(scene, p.parkQ);
end
