function p = scara_params()
%SCARA_PARAMS Load and convert all robot parameters into SI units (meters, radians, seconds).
%
% [EN] Returns a unified parameter struct `p` used across MATLAB kinematics,
%      Simulink stepper simulation, and 3D STL visualization.
%      - Geometric parameters use standard SI units (meters, radians).
%      - Electrical/stepper arrays follow the firmware axis order: [Z, J1(A), J2(B)].
%
% [VI] Ham khoi tao va quy doi tat ca thong so robot sang he don vi SI (met, radian, giay).
%      Tra ve cau truc (struct) `p` dung chung cho dong hoc, mo phong Simulink va do hoa 3D.
%      - Cac thong so hinh hoc dung don vi SI chuan (m, rad).
%      - Cac mang xung/dien tuan theo thu tu truc cua Firmware: [Z, J1(A), J2(B)].

    % =========================================================================
    % 1. DIRECTORY PATHS / DUONG DAN THU MUC
    % =========================================================================
    % [EN] Resolve project folder paths relative to this configuration file.
    % [VI] Xac dinh cac duong dan thu muc tuong doi tinh tu file cau hinh nay.
    p.root = fileparts(fileparts(mfilename('fullpath')));
    p.simulationRoot = fileparts(p.root);
    p.hardwareRoot = fullfile(fileparts(p.simulationRoot), 'Hardware');
    p.outputRoot = fullfile(p.root, 'output');
    p.modelsRoot = fullfile(p.root, 'simulink', 'models');

    % =========================================================================
    % 2. LOAD CALIBRATION SNAPSHOT / DOC DU LIEU HIEU CHUAN TU JSON
    % =========================================================================
    % [EN] Read the calibration JSON file recorded from real hardware logs.
    % [VI] Doc file JSON chua thong so hieu chuan thuc nghiem tu phan cung.
    calibrationFile = fullfile(p.root, 'config', 'robot_calibration.json');
    s = jsondecode(fileread(calibrationFile));
    c = s.config;
    h = s.calibration;

    p.firmware = s.firmware;
    p.sourceLog = s.source_log;

    % =========================================================================
    % 3. ROBOT GEOMETRY (SI UNITS) / HINH HOC ROBOT (DON VI SI: MET)
    % =========================================================================
    % [EN] Convert link lengths L1, L2 and tool offset [ox; oy] from mm to meters.
    % [VI] Doi chieu dai khau L1, L2 va do lech dau but [ox; oy] tu mm sang met.
    p.L1 = c.l1_mm / 1000;
    p.L2 = c.l2_mm / 1000;
    p.base = [0; 0];
    p.toolOffset = c.tool_offset_mm(:) / 1000;

    % =========================================================================
    % 4. STEPPER CALIBRATION & BELT COUPLING / HIEU CHUAN XUNG & TRUYEN DONG DAI
    % =========================================================================
    % [EN] factors_pp_unit is ordered [Z (pulses/mm), A (pulses/deg), B (pulses/deg)].
    %      - stepsPerDegree: [2x1] pulses per degree for arm motors A (J1) and B (J2).
    %      - stepsPerMeterZ: pulses per meter for the vertical Z leadscrew axis.
    %      - coupling: mechanical belt cross-coupling ratio (B_motor = q2 + coupling * q1).
    % [VI] factors_pp_unit co thu tu [Z (xung/mm), A (xung/do), B (xung/do)].
    %      - stepsPerDegree: [2x1] so xung tren moi do cua dong co tay may A (J1) va B (J2).
    %      - stepsPerMeterZ: so xung tren moi met cua truc vit me Z.
    %      - coupling: he so truyen dong dai vi sai (goc dong co B = q2 + coupling * q1).
    p.stepsPerDegree = h.factors_pp_unit(2:3);
    p.stepsPerDegree = p.stepsPerDegree(:);
    p.stepsPerMeterZ = h.factors_pp_unit(1) * 1000;

    p.coupling = h.coupling;
    p.beta = h.beta;
    p.microsteps = c.microsteps(:);
    p.positiveHigh = logical(c.positive_high(:));

    % =========================================================================
    % 5. WORKSPACE & JOINT LIMITS / GIOI HAN KHOP VA KHONG GIAN LAM VIEC
    % =========================================================================
    % [EN] Apply a 2-degree safety margin inward from the physical limit switches,
    %      and a 0.1 mm (0.0001 m) margin at the top and bottom of the Z travel.
    % [VI] Cong them bien an toan 2 do lui vao trong so voi cong tac hanh trinh,
    %      va bien an toan 0.1 mm (0.0001 m) o hai dau hanh trinh truc Z.
    p.jointLimits = deg2rad(c.joint_limits_deg + repmat([2 -2], 2, 1));
    p.zSpan = h.range(1) / p.stepsPerMeterZ;
    p.zLimits = [0.0001, p.zSpan - 0.0001];

    % [EN] Parking pose [q1_rad; q2_rad; z_m] after homing completes.
    % [VI] Tu the do (park pose) [q1_rad; q2_rad; z_m] sau khi ve HOME xong.
    p.parkQ = [deg2rad(c.park_deg(:)); p.zSpan - c.z_clear_mm / 1000];

    % =========================================================================
    % 6. DRAWING PAPER & TIMING PARAMETERS / THONG SO GIAY VE VA THOI GIAN
    % =========================================================================
    % [EN] Paper center/size in meters, paper surface height Z, pen lift clearance,
    %      singularity threshold |sin(q2_eff)|, sample time dt (2 ms), and watchdog timeout.
    % [VI] Tam va kich thuoc giay ve (m), do cao mat giay Z, do nhac but,
    %      nguong suy bien |sin(q2_eff)|, chu ky lay mau dt (2 ms) va thoi gian watchdog.
    p.paperCenter = [0; 0.165];
    p.paperHalfSize = [0.055; 0.045];
    p.paperZ = 0.020;
    p.penLift = 0.002;
    p.singularityMargin = 0.03;
    p.dt = 0.002;
    p.watchdogSeconds = 0.350;

    % =========================================================================
    % 7. HOMING & STEPPER RATE LIMITS / TOC DO VE HOME VA GIOI HAN TAN SO XUNG
    % =========================================================================
    p.zHomeSpeed = c.z_home_speed_mm_s / 1000;
    p.zHomePps = round(p.zHomeSpeed * p.stepsPerMeterZ);
    p.zLatchPps = c.z_latch_pps;
    p.armHomePps = c.arm_home_pps;
    p.j2HomePps = c.j2_home_pps;
    p.maxMasterRate = c.max_master_rate;

    % =========================================================================
    % 8. SENSITIVITY & FAULT-INJECTION SCENARIOS / KICH BAN DO NHAY & GIA LAP LOI
    % =========================================================================
    % [EN] Sensitivity scenarios for educational "what-if" studies, NOT identified
    %      hardware errors. By default, backlash and lag are zero (ideal baseline).
    % [VI] Cac thong so kich ban gia dinh de khao sat do nhay (KHONG phai sai so do
    %      tu phan cung thuc te). Mac dinh do ro (backlash) va tre (lag) bang 0.
    p.backlash = [0; 0; 0];       % [EN] Full deadband width: [rad; rad; m] / [VI] Do ro toan phan
    p.lagSeconds = [0; 0; 0];     % [EN] First-order time constant (s) / [VI] Hang so thoi gian bac 1 (s)
    p.actualL1 = p.L1;
    p.actualL2 = p.L2;
    p.actualToolOffset = p.toolOffset;

    % [EN] Fault injection flags: USB disconnect time and forced-open NC switch mask
    %      ordered [Z+, Z-, J1+, J1-, J2+, J2-].
    % [VI] Co gia lap loi: thoi diem mat ket noi USB va trang thai ho mach cong tac
    %      thuong dong (NC) theo thu tu [Z+, Z-, J1+, J1-, J2+, J2-].
    p.disconnectTime = inf;
    p.forceOpenNc = false(6, 1);

    p.python = 'python';
    p.scenarioLabel = 'Baseline: measured pulse calibration; no identified mechanics error';
end
