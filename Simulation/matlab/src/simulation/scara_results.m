function out = scara_results(t, q, xyz, steps, allowed, ref, p)
%SCARA_RESULTS Package Simulink replay outputs and compute XY model tracking errors.
%
% [EN] Aligns simulated joint/Cartesian trajectories with the reference command
%      timeline using zero-order-hold ('previous') interpolation, identifies when
%      the brush tip touches the paper, and computes RMS and peak XY errors (in mm)
%      across all pen-down samples.
%
% [VI] Dong goi ket qua mo phong Simulink va tinh sai so bam quy dao XY (theo mm).
%      Dong bo duong cong mo phong voi quy dao tham chieu bang phep noi suy bac 0
%      ('previous'), xac dinh trang thai cham giay va tinh sai so hieu dung (RMS)
%      cung nhu sai so cuc dai (Max) trong suot qua trinh ha but ve.

    % -------------------------------------------------------------------------
    % Step 1: Store raw simulation time series
    % Buoc 1: Luu tru cac chuoi thoi gian dau ra tu mo phong
    % -------------------------------------------------------------------------
    out.t = t;
    out.q = q;
    out.xyz = xyz;
    out.steps = steps;
    out.allowed = logical(allowed);

    % -------------------------------------------------------------------------
    % Step 2: Sample reference XYZ and pen state at simulation timestamps `t`
    % Buoc 2: Noi suy vi tri tham chieu XYZ va trang thai but tai cac moc `t`
    % -------------------------------------------------------------------------
    out.reference = interp1(ref.t, ref.xyz, t, 'previous', 'extrap');
    out.pen = logical(interp1(ref.t, double(ref.pen), t, 'previous', 'extrap'));

    % [EN] Brush draws on paper when pen-down is requested AND Z is within 0.2 mm of paperZ.
    % [VI] But thuc su ve len giay khi co lenh ha but VA do cao Z cach mat giay < 0.2 mm.
    out.draw = out.pen & abs(xyz(:, 3) - p.paperZ) < 0.0002;

    % -------------------------------------------------------------------------
    % Step 3: Compute Euclidean XY tracking error (m) and pen-down statistics (mm)
    % Buoc 3: Tinh sai so khoang cach XY (m) va thong ke RMS / Max khi ha but (mm)
    % -------------------------------------------------------------------------
    out.xyError = vecnorm(xyz(:, 1:2) - out.reference(:, 1:2), 2, 2);

    ink = out.pen;
    out.rmsErrorMm = sqrt(mean(out.xyError(ink).^2)) * 1000;
    out.maxErrorMm = max(out.xyError(ink)) * 1000;

    out.note = 'Model error only. No encoder, identified backlash, missed steps or measured pen force.';
end