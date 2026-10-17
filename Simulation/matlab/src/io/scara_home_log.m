function [T, f] = scara_home_log(logfile)
%SCARA_HOME_LOG Parse and plot recorded hardware USB telemetry during the HOME sequence.
%
% [EN] Inspects actual recorded `.jsonl` status frames from the STM32/Arduino
%      stepper controller during homing, including step-counter origin resets when
%      limit switches latch. Note: these samples are USB status frames, not
%      individual step pulses or closed-loop encoder readings.
%
% [VI] Phan tich va ve do thi du lieu telemetry thu duoc qua cong USB trong qua
%      trinh robot ve vi tri goc (HOME), bao gom ca cac thoi diem dat lai bo dem
%      xung khi cham cong tac hanh trinh. Luu y: day la cac goi trang thai USB,
%      khong phai tung xung step rieng le hay du lieu tu encoder phan hoi.

    % -------------------------------------------------------------------------
    % Step 1: Locate the recorded JSONL log file (or prompt user via file dialog)
    % Buoc 1: Tim duong dan file log JSONL (hoac mo hop thoai de nguoi dung chon)
    % -------------------------------------------------------------------------
    p = scara_params();
    if nargin < 1
        repoRoot = fileparts(p.simulationRoot);
        logfile = fullfile(repoRoot, 'Doc', 'logs', p.sourceLog);
        if ~isfile(logfile)
            logfile = fullfile(repoRoot, 'Software', 'dist', ...
                'SCARA_Cartesian_NC_v5_R14', 'logs', p.sourceLog);
        end
    end

    if ~isfile(logfile)
        [name, folder] = uigetfile('*.jsonl', 'Choose recorded SCARA HOME log');
        if isequal(name, 0)
            T = [];
            f = [];
            return;
        end
        logfile = fullfile(folder, name);
    end

    % -------------------------------------------------------------------------
    % Step 2: Read JSON Lines status frames during active homing (stage > 0)
    % Buoc 2: Doc tung dong JSON trang thai trong qua trinh ve HOME (stage > 0)
    % -------------------------------------------------------------------------
    fid = fopen(logfile, 'r', 'n', 'UTF-8');
    assert(fid >= 0);
    cleanup = onCleanup(@() fclose(fid));

    d = [];
    while ~feof(fid)
        line = fgetl(fid);
        try
            e = jsondecode(line);
        catch
            continue;
        end

        if ~isfield(e, 'type') || ~strcmp(e.type, 'status')
            continue;
        end

        s = e.status;
        if s.stage == 0
            continue;
        end

        d(end + 1, :) = [ ...
            s.up_ms / 1000, ...
            s.pos(:)', ...
            s.stage, ...
            s.phase(:)', ...
            s.raw, ...
            s.switches, ...
            s.input_wait, ...
            s.timer_late, ...
            s.fault, ...
            s.coupling_ppm / 1e6 ...
        ]; %#ok<AGROW>

        % [EN] Stop once homing finishes and robot reaches stage 8 (referenced).
        % [VI] Dung doc khi qua trinh HOME hoan tat va dat trang thai tham chieu (stage 8).
        if s.referenced && s.stage == 8
            break;
        end
    end

    assert(~isempty(d), 'scara:home', 'No HOME status frames in this log.');

    % -------------------------------------------------------------------------
    % Step 3: Normalize time to start at t = 0 s and export CSV table
    % Buoc 3: Chuan hoa truc thoi gian bat dau tu 0 giay va xuat bang CSV
    % -------------------------------------------------------------------------
    d(:, 1) = d(:, 1) - d(1, 1);

    T = array2table(d, 'VariableNames', { ...
        'time_s', 'Z_raw', 'A_raw', 'B_raw', ...
        'stage', 'Z_phase', 'J1_phase', 'J2_phase', ...
        'nc_raw_bits', 'nc_filtered_bits', 'input_wait', ...
        'timer_late', 'fault', 'coupling' ...
    });

    folder = fullfile(p.outputRoot, 'data');
    if ~isfolder(folder)
        mkdir(folder);
    end
    writetable(T, fullfile(folder, 'home_telemetry.csv'));

    % -------------------------------------------------------------------------
    % Step 4: Plot 3-panel HOME telemetry overview and save PNG figure
    % Buoc 4: Ve do thi 3 ngan hien thi qua trinh ve HOME va luu anh PNG
    % -------------------------------------------------------------------------
    f = figure('Name', 'Actual HOME telemetry - no encoder', ...
        'Color', 'w', 'Position', [80 70 1100 720]);
    if isprop(f, 'Theme')
        f.Theme = 'light';
    end
    tiledlayout(f, 3, 1, 'TileSpacing', 'compact');

    % Panel 1: Raw command step counts (Z, A, B)
    nexttile;
    plot(T.time_s, [T.Z_raw, T.A_raw, T.B_raw]);
    grid on;
    legend('Z raw', 'A raw', 'B raw');
    ylabel('Command counts');
    title('Count origins reset during HOME: discontinuities are not physical jumps');

    % Panel 2: Homing finite-state-machine stages and axis phases
    nexttile;
    stairs(T.time_s, [T.stage, T.Z_phase, T.J1_phase, T.J2_phase]);
    grid on;
    legend('Stage', 'Z phase', 'J1 phase', 'J2 phase');
    ylabel('HOME state');

    % Panel 3: Limit switch bitmasks and fault/wait flags
    nexttile;
    stairs(T.time_s, [T.nc_raw_bits, T.nc_filtered_bits, T.input_wait, T.fault]);
    grid on;
    legend('Raw NC mask', 'Filtered NC mask', 'Waiting', 'Fault');
    xlabel('Actual time (s)');
    ylabel('Telemetry');

    folder = fullfile(p.outputRoot, 'figures');
    if ~isfolder(folder)
        mkdir(folder);
    end
    exportgraphics(f, fullfile(folder, 'home_telemetry.png'), 'Resolution', 140);
end
