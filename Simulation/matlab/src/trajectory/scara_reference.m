function ref = scara_reference(kind, p)
%SCARA_REFERENCE Load or generate an offline R9 stepper pulse reference trajectory.
%
% [EN] Reuses the Python GUI/R9 integer DDA trajectory planner offline instead of
%      duplicating a second planner in MATLAB. Results are cached in a `.mat` file
%      and automatically regenerated if any Python planner source or calibration
%      JSON file is newer than the cache.
%
% [VI] Tai hoac sinh quy dao xung tham chieu bang bo lap quy dao DDA so nguyen R9
%      viet bang Python (khong can viet lai bo lap quy dao thu hai trong MATLAB).
%      Ket qua duoc luu dem (cache) vao file `.mat` va tu dong cap nhat neu file
%      ma nguon Python hoac file JSON hieu chuan co thay doi moi hon.

    % -------------------------------------------------------------------------
    % Step 1: Set default arguments and validate pattern name
    % Buoc 1: Gan tham so mac dinh va kiem tra ten hinh mẫu (pattern)
    % -------------------------------------------------------------------------
    if nargin < 2
        p = scara_params();
    end
    if nargin < 1
        kind = 'flower';
    end

    kind = lower(char(kind));
    validPatterns = {'flower', 'circle', 'square', 'star', 'heart', 'bk', 'linh', 'thoa'};
    assert(ismember(kind, validPatterns), ...
        'scara:pattern', 'Choose flower/circle/square/star/heart/BK/LINH/THOA.');

    % -------------------------------------------------------------------------
    % Step 2: Check whether a valid, up-to-date .mat cache already exists
    % Buoc 2: Kiem tra xem file cache .mat da ton tai va con moi hay khong
    % -------------------------------------------------------------------------
    folder = fullfile(p.outputRoot, 'reference');
    file = fullfile(folder, [kind '.csv']);
    meta = fullfile(folder, [kind '.json']);
    cache = fullfile(folder, [kind '.mat']);

    sources = [ ...
        dir(fullfile(fileparts(p.simulationRoot), 'Software', 'src', '*.py')); ...
        dir(fullfile(p.root, 'tools', '*.py')); ...
        dir(fullfile(p.root, 'config', 'robot_calibration.json')) ...
    ];
    stamp = max([sources.datenum]);

    if isfile(cache)
        saved = load(cache, 'ref', 'sourceStamp');
        if saved.sourceStamp >= stamp
            ref = saved.ref;
            check();
            return;
        end
    end

    % -------------------------------------------------------------------------
    % Step 3: Invoke offline Python exporter if cache is missing or stale
    % Buoc 3: Goi script Python xuat quy dao neu chua co cache hoac cache da cu
    % -------------------------------------------------------------------------
    script = fullfile(p.root, 'tools', 'export_firmware_reference.py');
    if ~isfile(file) || ~isfile(meta) || isfile(cache)
        cmd = sprintf('"%s" "%s" --pattern %s --output "%s"', ...
            p.python, script, upper(kind), folder);
        [code, msg] = system(cmd);
        assert(code == 0, 'scara:python', 'Offline planner failed: %s', msg);
    end

    % -------------------------------------------------------------------------
    % Step 4: Parse exported CSV/JSON, reconstruct joint angles, and save cache
    % Buoc 4: Doc file CSV/JSON, phuc hoi goc khop tu xung va luu vao cache .mat
    % -------------------------------------------------------------------------
    d = readmatrix(file);
    ref.t = d(:, 1);
    ref.steps = d(:, 2:4);
    ref.xyz = d(:, 5:7);
    ref.pen = logical(d(:, 8));
    ref.valid = d(:, 9);
    ref.q = scara_from_steps(ref.steps, p);
    ref.meta = jsondecode(fileread(meta));
    ref.kind = kind;

    sourceStamp = stamp;
    save(cache, 'ref', 'sourceStamp');
    delete(file);
    delete(meta);

    check();

    % -------------------------------------------------------------------------
    % Nested helper: Verify sample time dt and calibration match scara_params
    % Ham con: Kiem tra chu ky lay mau dt va thong so hieu chuan khop voi p
    % -------------------------------------------------------------------------
    function check()
        assert(abs(ref.meta.dt - p.dt) < 1e-10, ...
            'scara:sample', 'Reference dt must equal model dt.');
        f = ref.meta.calibration.factors_pp_unit;
        expectedFactors = [p.stepsPerMeterZ / 1000; p.stepsPerDegree];
        assert(max(abs(f(:) - expectedFactors)) < 1e-9 ...
            && abs(ref.meta.calibration.coupling - p.coupling) < 1e-9, ...
            'scara:calibration', 'Edit config/robot_calibration.json to change the planner calibration.');
    end
end
