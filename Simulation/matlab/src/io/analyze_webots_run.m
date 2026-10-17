function [T, f] = analyze_webots_run(csvFile)
%ANALYZE_WEBOTS_RUN Load and plot a Webots `tracking.csv` sensor log in MATLAB.
%
% [EN] Reads a `tracking.csv` file produced by the Webots `scara_draw` controller
%      (located under `Simulation/webots/output/runs/<timestamp>/tracking.csv`)
%      and plots a 2x2 diagnostic figure showing:
%      1. Reference XY trajectory vs actual drawn ink path (mm).
%      2. Planar XY tracking error over time (mm).
%      3. Brush-tip normal contact force on the paper over time (N).
%      4. Free-tip reference Z, Z carriage height, and actual deflected tip Z (mm).
%
% [VI] Doc file `tracking.csv` xuat ra tu bo dieu khien Webots `scara_draw`
%      (nam trong `Simulation/webots/output/runs/<timestamp>/tracking.csv`)
%      va ve do thi chan doan 2x2 gom:
%      1. Quy dao tham chieu XY so voi net muc thuc te tren giay (mm).
%      2. Sai so bam quy dao phang XY theo thoi gian (mm).
%      3. Luc phap tuyen cua dau co/but tac dung len mat giay theo thoi gian (N).
%      4. Do cao Z tham chieu, do cao xe truot Z va do cao mui but thuc te (mm).

    % -------------------------------------------------------------------------
    % Step 1: Select `tracking.csv` file interactively if no path was provided
    % Buoc 1: Mo hop thoai chon file `tracking.csv` neu khong truyen duong dan
    % -------------------------------------------------------------------------
    if nargin < 1
        [name, folder] = uigetfile('*.csv', 'Choose Webots tracking.csv');
        if isequal(name, 0)
            T = [];
            f = [];
            return;
        end
        csvFile = fullfile(folder, name);
    end

    % -------------------------------------------------------------------------
    % Step 2: Read CSV table and verify required telemetry columns exist
    % Buoc 2: Doc bang CSV va kiem tra day du cac cot du lieu cam bien can thiet
    % -------------------------------------------------------------------------
    T = readtable(csvFile, 'VariableNamingRule', 'preserve');
    required = { ...
        'time_s', 'X_ref_m', 'Y_ref_m', 'X_tip_m', 'Y_tip_m', ...
        'normal_force_N', 'xy_error_mm', 'ink_enabled' ...
    };
    assert(all(ismember(required, T.Properties.VariableNames)), ...
        'scara:webots', 'Select a Webots tracking.csv log.');

    % -------------------------------------------------------------------------
    % Step 3: Create 2x2 diagnostic figure
    % Buoc 3: Tao cua so do thi 2x2 hien thi ket qua phan tich
    % -------------------------------------------------------------------------
    f = figure('Name', 'Webots sensor log', 'Color', 'w');
    if isprop(f, 'Theme')
        f.Theme = 'light';
    end
    tiledlayout(f, 2, 2);

    % Subplot 1: 2D XY Reference vs Drawn Ink Path
    nexttile;
    plot(T.X_ref_m * 1000, T.Y_ref_m * 1000, '--');
    hold on;
    ink = [T.X_tip_m, T.Y_tip_m] * 1000;
    ink(~logical(T.ink_enabled), :) = nan;
    plot(ink(:, 1), ink(:, 2));
    axis equal;
    grid on;
    xlabel('X (mm)');
    ylabel('Y (mm)');
    legend('Reference', 'Ink');

    % Subplot 2: XY Tracking Error (mm)
    nexttile;
    plot(T.time_s, T.xy_error_mm);
    grid on;
    xlabel('Time (s)');
    ylabel('XY error (mm)');

    % Subplot 3: Normal Contact Force (N)
    nexttile;
    plot(T.time_s, T.normal_force_N);
    grid on;
    xlabel('Time (s)');
    ylabel('Contact force (N)');

    % Subplot 4: Vertical Z Heights (mm)
    nexttile;
    plot(T.time_s, [T.Z_ref_m, T.Z_m, T.Z_tip_m] * 1000);
    grid on;
    xlabel('Time (s)');
    ylabel('Z (mm)');
    legend('Free tip reference', 'Carriage', 'Actual tip');
end
