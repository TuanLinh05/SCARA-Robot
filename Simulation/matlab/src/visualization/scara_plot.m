function f = scara_plot(out, p)
%SCARA_PLOT Plot a 2x2 summary of Simulink stepper replay and save `tracking.png`.
%
% [EN] Generates a 4-panel figure summarizing the offline pulse replay:
%      1. Top-left:     Pen-down GUI target XY vs simulated pulse + mechanics XY (mm).
%      2. Top-right:    Shoulder (J1) and relative elbow (J2) angles over time (deg).
%      3. Bottom-left:  Vertical Z trajectory (mm) vs paper surface height `paperZ`.
%      4. Bottom-right: Pen-down planar XY model tracking error over time (mm).
%
% [VI] Tao cua so do thi 2x2 tong hop ket qua mo phong phat lai xung buoc:
%      1. Tren-trai:  Quy dao XY muc tieu (khi ha but) va quy dao mo phong (mm).
%      2. Tren-phai:  Goc khop vai (J1) va goc khuyu tuong doi (J2) theo thoi gian (do).
%      3. Duoi-trai:  Do cao truc Z (mm) so voi be mat giay ve `paperZ`.
%      4. Duoi-phai:  Sai so bam quy dao phang XY khi ha but theo thoi gian (mm).

    % -------------------------------------------------------------------------
    % Step 1: Create figure window and 2x2 tiled layout
    % Buoc 1: Khoi tao cua so do thi va bo cuc luoi 2x2
    % -------------------------------------------------------------------------
    f = figure('Name', 'SCARA pulse replay and sensitivity', ...
        'Color', 'w', 'Position', [70 70 1200 760]);
    if isprop(f, 'Theme')
        f.Theme = 'light';
    end
    tiledlayout(f, 2, 2, 'TileSpacing', 'compact');

    if isfield(p, 'scenarioLabel')
        sgtitle(p.scenarioLabel, 'Interpreter', 'none');
    end

    % -------------------------------------------------------------------------
    % Step 2: Panel 1 - Planar XY drawing trajectory (pen-down segments only)
    % Buoc 2: O thu 1 - Quy dao ve tren mat phang XY (chi hien thi khi ha but)
    % -------------------------------------------------------------------------
    nexttile;
    hold on;
    axis equal;
    grid on;

    r = out.reference;
    r(~out.pen, :) = nan;
    x = out.xyz;
    x(~out.pen, :) = nan;

    plot(r(:, 1) * 1000, r(:, 2) * 1000, '--', 'LineWidth', 1.2);
    plot(x(:, 1) * 1000, x(:, 2) * 1000, 'LineWidth', 1);
    xlabel('X (mm)');
    ylabel('Y (mm)');
    legend('GUI target', 'Pulse + mechanics model');
    title(sprintf('Model only: RMS %.3f / max %.3f mm', out.rmsErrorMm, out.maxErrorMm));

    % -------------------------------------------------------------------------
    % Step 3: Panel 2 - Joint angles J1 and J2 over time (degrees)
    % Buoc 3: O thu 2 - Goc khop J1 va J2 tuong doi theo thoi gian (do)
    % -------------------------------------------------------------------------
    nexttile;
    plot(out.t, rad2deg(out.q(:, 1:2)));
    grid on;
    xlabel('Time (s)');
    ylabel('Joint angle (deg)');
    legend('J1', 'J2 relative');

    % -------------------------------------------------------------------------
    % Step 4: Panel 3 - Vertical Z height over time (mm)
    % Buoc 4: O thu 3 - Do cao truc Z theo thoi gian (mm)
    % -------------------------------------------------------------------------
    nexttile;
    plot(out.t, out.q(:, 3) * 1000);
    yline(p.paperZ * 1000, '--');
    grid on;
    xlabel('Time (s)');
    ylabel('Z (mm)');
    title('Z=0 at bottom switch; +Z up');

    % -------------------------------------------------------------------------
    % Step 5: Panel 4 - Pen-down XY model tracking error over time (mm)
    % Buoc 5: O thu 4 - Sai so mo hinh XY khi ha but theo thoi gian (mm)
    % -------------------------------------------------------------------------
    nexttile;
    inkError = out.xyError;
    inkError(~out.pen) = nan;
    plot(out.t, inkError * 1000);
    grid on;
    xlabel('Time (s)');
    ylabel('Pen-down model XY error (mm)');
    title('No measured paper/encoder error');

    % -------------------------------------------------------------------------
    % Step 6: Save figure to `output/figures/tracking.png`
    % Buoc 6: Xuat va luu do thi ra file anh `output/figures/tracking.png`
    % -------------------------------------------------------------------------
    folder = fullfile(p.outputRoot, 'figures');
    if ~isfolder(folder)
        mkdir(folder);
    end
    exportgraphics(f, fullfile(folder, 'tracking.png'), 'Resolution', 140);
end
