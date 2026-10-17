function f = scara_app(visible)
%SCARA_APP Interactive 3D STL viewer and offline Simulink pulse-replay GUI.
%
% [EN] Launches an educational MATLAB GUI that lets students:
%      - Jog joints J1, J2, and Z manually with sliders and inspect FK / stepper pulses.
%      - Select a drawing pattern (`flower`, `circle`, `star`, `heart`, `square`, `BK`, `LINH`, `THOA`)
%        and run the standard-block Simulink stepper model (`scara_stepper.slx`).
%      - Toggle an illustrative sensitivity scenario (0.2 deg backlash + 10 ms lag).
%      - Open the Simulink block diagram, inspect real HOME USB logs, or export results.
%      No physical serial port or hardware motion is triggered by this viewer.
%
% [VI] Khoi chay giao dien do hoa 3D tuong tac (GUI) giup sinh vien:
%      - Keo thanh truot dieu chinh J1, J2, Z thu cong de quan sat dong hoc thuan va xung buoc.
%      - Chon hinh ve (`flower`, `circle`, `star`, `heart`, `square`, `BK`, `LINH`, `THOA`)
%        va chay mo phong Simulink (`scara_stepper.slx`) voi hoat anh 3D.
%      - Bat/tat kich ban khao sat do nhay (do ro 0.2 do + tre dong hoc 10 ms).
%      - Mo so do khoi Simulink, xem log ve HOME thuc te, hoac xuat du lieu va do thi.
%      Ung dung chay hoan toan ngoai tuyen (offline), khong ket noi cong COM phan cung.

    % -------------------------------------------------------------------------
    % Step 1: Initialize parameters, figure window, and 3D/2D axes
    % Buoc 1: Khoi tao thong so, cua so giao dien chinh va cac truc do thi 3D/2D
    % -------------------------------------------------------------------------
    if nargin < 1
        visible = 'on';
    end

    p = scara_params();
    q = p.parkQ;

    f = figure( ...
        'Name', 'SCARA | R9 pulse replay', ...
        'NumberTitle', 'off', ...
        'Visible', visible, ...
        'Color', [0.95 0.96 0.98], ...
        'Position', [60 70 1380 800]);
    if isprop(f, 'Theme')
        f.Theme = 'light';
    end

    % Left 3D STL assembly view
    ax = axes(f, 'Position', [0.04 0.21 0.56 0.68]);
    scene = scara_scene(ax, p);
    title(ax, 'STL assembly | Z/A/B pulses + measured belt coupling');

    % Top-right 2D XY paper view
    xyax = axes(f, 'Position', [0.65 0.66 0.31 0.23]);
    hold(xyax, 'on');
    grid(xyax, 'on');
    axis(xyax, 'equal');
    xlim(xyax, [-60 60]);
    ylim(xyax, [110 210]);
    xlabel(xyax, 'X (mm)');
    ylabel(xyax, 'Y (mm)');
    target = plot(xyax, nan, nan, '--', 'Color', [0.15 0.55 0.8]);
    ink = plot(xyax, nan, nan, 'Color', [0.13 0.16 0.2], 'LineWidth', 1.2);
    title(xyax, 'GUI target / simulated ink');

    % -------------------------------------------------------------------------
    % Step 2: Create right-hand control panel buttons and dropdowns
    % Buoc 2: Tao bang dieu khien ben phai (chon hinh, kich ban, nut bam)
    % -------------------------------------------------------------------------
    label('Pattern', [0.64 0.575 0.12 0.03]);
    choice = uicontrol(f, ...
        'Style', 'popupmenu', ...
        'String', {'flower', 'circle', 'star', 'heart', 'square', 'BK', 'LINH', 'THOA'}, ...
        'Units', 'normalized', ...
        'Position', [0.77 0.575 0.19 0.04]);

    sensitivity = uicontrol(f, ...
        'Style', 'checkbox', ...
        'String', 'Example: 0.2 deg backlash + 10 ms lag', ...
        'Units', 'normalized', ...
        'Position', [0.64 0.535 0.33 0.03], ...
        'Value', 0);

    runbtn = uicontrol(f, ...
        'Style', 'pushbutton', ...
        'String', 'Simulate + animate', ...
        'Units', 'normalized', ...
        'Position', [0.64 0.47 0.21 0.05], ...
        'Callback', @runexample);

    stopbtn = uicontrol(f, ...
        'Style', 'togglebutton', ...
        'String', 'Stop view', ...
        'Units', 'normalized', ...
        'Position', [0.87 0.47 0.09 0.05]);

    uicontrol(f, ...
        'Style', 'pushbutton', ...
        'String', 'Open standard-block Simulink model', ...
        'Units', 'normalized', ...
        'Position', [0.64 0.405 0.32 0.05], ...
        'Callback', @opensimulink);

    uicontrol(f, ...
        'Style', 'pushbutton', ...
        'String', 'Inspect actual HOME log', ...
        'Units', 'normalized', ...
        'Position', [0.64 0.34 0.32 0.05], ...
        'Callback', @homelog);

    uicontrol(f, ...
        'Style', 'pushbutton', ...
        'String', 'Plot / export last replay', ...
        'Units', 'normalized', ...
        'Position', [0.64 0.275 0.32 0.05], ...
        'Callback', @exportlast);

    status = uicontrol(f, ...
        'Style', 'text', ...
        'String', 'Offline. No physical position feedback.', ...
        'Units', 'normalized', ...
        'Position', [0.64 0.18 0.33 0.08], ...
        'HorizontalAlignment', 'left', ...
        'FontSize', 10);

    label(sprintf('Measured: %.6f / %.6f pulses/deg | k = %.6f | Z home = %.1f mm/s', ...
        p.stepsPerDegree, p.coupling, p.zHomeSpeed * 1000), [0.04 0.955 0.9 0.03]);
    label('Geometry/TCP/backlash scenarios are assumptions. Animation is accelerated; Z=0 is the lower switch.', ...
        [0.04 0.012 0.93 0.035]);

    % -------------------------------------------------------------------------
    % Step 3: Create manual joint-jogging sliders at the bottom
    % Buoc 3: Tao 3 thanh truot dieu chinh thu cong cac khop J1, J2, Z o phia duoi
    % -------------------------------------------------------------------------
    names = {'J1 (deg)', 'J2 relative (deg)', 'Z (mm)'};
    mins = [rad2deg(p.jointLimits(:, 1)); p.zLimits(1) * 1000];
    maxs = [rad2deg(p.jointLimits(:, 2)); p.zLimits(2) * 1000];
    vals = [rad2deg(q(1:2)); q(3) * 1000];

    sliders = gobjects(3, 1);
    labels = gobjects(3, 1);
    for j = 1:3
        x = 0.04 + (j - 1) * 0.31;
        labels(j) = label(names{j}, [x 0.11 0.28 0.03]);
        sliders(j) = uicontrol(f, ...
            'Style', 'slider', ...
            'Min', mins(j), ...
            'Max', maxs(j), ...
            'Value', vals(j), ...
            'Units', 'normalized', ...
            'Position', [x 0.068 0.28 0.03], ...
            'Callback', @manual);
    end

    last = [];
    lastref = [];
    manual();
    rotate3d(ax, 'on');

    % =========================================================================
    % NESTED CALLBACK FUNCTIONS / CAC HAM XU LY SU KIEN GIAO DIEN
    % =========================================================================
    function h = label(text, pos)
        h = uicontrol(f, ...
            'Style', 'text', ...
            'String', text, ...
            'Units', 'normalized', ...
            'Position', pos, ...
            'HorizontalAlignment', 'left');
    end

    function manual(~, ~)
        % [EN] Update 3D robot pose and pulse readout when user drags a joint slider.
        % [VI] Cap nhat tu the robot 3D va hien thi so xung khi keo thanh truot.
        q = [ ...
            deg2rad(sliders(1).Value); ...
            deg2rad(sliders(2).Value); ...
            sliders(3).Value / 1000 ...
        ];
        scara_update(scene, q);
        tip = scara_fk(q, p);
        [~, steps] = scara_motor_coordinates(q, p);

        for j = 1:3
            labels(j).String = sprintf('%s: %.2f', names{j}, sliders(j).Value);
        end
        status.String = sprintf( ...
            'XYZ %.2f / %.2f / %.2f mm\nNormalized Z/A/B pulses: %d / %d / %d', ...
            tip * 1000, steps);
    end

    function runexample(~, ~)
        % [EN] Run Simulink stepper replay for the selected pattern and animate the 3D scene.
        % [VI] Chay mo phong Simulink cho hinh da chon va dienat hoat anh tren khung 3D.
        kind = lower(choice.String{choice.Value});
        runbtn.Enable = 'off';
        set(sliders, 'Enable', 'off');
        stopbtn.Value = 0;
        status.String = 'Running standard Simulink blocks...';
        drawnow;

        try
            p.backlash = zeros(3, 1);
            p.lagSeconds = zeros(3, 1);
            p.scenarioLabel = 'Baseline: measured pulse calibration; no identified mechanics error';

            if sensitivity.Value
                p.backlash = [deg2rad(0.2); deg2rad(0.2); 0];
                p.lagSeconds = [0.01; 0.01; 0];
                p.scenarioLabel = 'Illustrative sensitivity: 0.2 deg backlash + 10 ms lag, NOT measured';
            end

            ref = scara_reference(kind, p);
            out = scara_simulink(ref, p, false);
            last = out;
            lastref = ref;

            desired = out.reference;
            desired(~out.pen, :) = nan;
            actual = out.xyz;
            actual(~out.pen, :) = nan;

            set(target, 'XData', desired(:, 1) * 1000, 'YData', desired(:, 2) * 1000);
            set(scene.target, ...
                'XData', desired(:, 1) * 1000, ...
                'YData', desired(:, 2) * 1000, ...
                'ZData', desired(:, 3) * 1000);

            stride = max(1, ceil(numel(out.t) / 250));
            frameIndices = [1:stride:numel(out.t), numel(out.t)];

            for k = frameIndices
                if ~isgraphics(f) || stopbtn.Value
                    break;
                end

                scara_update(scene, out.q(k, :)');
                values = [rad2deg(out.q(k, 1:2)), out.q(k, 3) * 1000];
                for j = 1:3
                    sliders(j).Value = max(mins(j), min(maxs(j), values(j)));
                    labels(j).String = sprintf('%s: %.2f', names{j}, values(j));
                end

                set(ink, 'XData', actual(1:k, 1) * 1000, 'YData', actual(1:k, 2) * 1000);
                set(scene.ink, ...
                    'XData', actual(1:k, 1) * 1000, ...
                    'YData', actual(1:k, 2) * 1000, ...
                    'ZData', actual(1:k, 3) * 1000);

                status.String = sprintf( ...
                    'Replay %.1f / %.1f s\nModel RMS %.3f mm; not measured paper accuracy.', ...
                    out.t(k), out.t(end), out.rmsErrorMm);
                drawnow;
                pause(0.015);
            end
        catch e
            if isgraphics(f)
                errordlg(e.message, 'Offline simulation');
            end
        end

        if isgraphics(f)
            runbtn.Enable = 'on';
            set(sliders, 'Enable', 'on');
        end
    end

    function opensimulink(~, ~)
        try
            path = fullfile(p.modelsRoot, 'scara_stepper.slx');
            if ~isfile(path)
                build_scara_simulink(false);
            end
            scara_setup(lower(choice.String{choice.Value}), p);
            open_system(path);
        catch e
            errordlg(e.message, 'Simulink');
        end
    end

    function homelog(~, ~)
        try
            scara_home_log();
        catch e
            errordlg(e.message, 'HOME log');
        end
    end

    function exportlast(~, ~)
        if isempty(last)
            status.String = 'Run a replay first.';
            return;
        end
        scara_export(last, lastref, p);
        scara_plot(last, p);
        status.String = 'Saved output/data/simulation.csv + MAT; output/figures/tracking.png';
    end
end
