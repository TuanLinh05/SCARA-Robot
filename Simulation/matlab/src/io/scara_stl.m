function [vertices, faces] = scara_stl(path)
%SCARA_STL Read a binary STL CAD mesh file without requiring any MATLAB toolbox.
%
% [EN] Parses a standard little-endian binary STL file:
%      - Bytes 1..80:   80-byte text header (ignored).
%      - Bytes 81..84:  uint32 number of triangles `n`.
%      - For each of the `n` triangles (50 bytes per triangle):
%          * 12 float32 values (48 bytes): normal vector (3 floats) +
%            3 vertex positions (9 floats: v1x,v1y,v1z, v2x,v2y,v2z, v3x,v3y,v3z).
%          * 2-byte uint16 attribute byte count (ignored).
%      Deduplicates shared vertex coordinates to return `vertices` (Vx3) and `faces` (Fx3).
%
% [VI] Doc file luoi 3D STL nhi phan (binary STL) khong can bat ky toolbox phu nao:
%      - Byte 1..80:    80 byte tieu de (bo qua).
%      - Byte 81..84:   So luong tam giac `n` (kieu uint32).
%      - Moi tam giac trong `n` tam giac chiem dung 50 byte:
%          * 12 so thuc float32 (48 byte): vector phap tuyen (3 so) +
%            toa do 3 dinh tam giac (9 so: v1x,v1y,v1z, v2x,v2y,v2z, v3x,v3y,v3z).
%          * 2 byte thuoc tinh phu (bo qua).
%      Gop cac dinh trung nhau bang `unique` de tra ve `vertices` (Vx3) va `faces` (Fx3).

    % -------------------------------------------------------------------------
    % Step 1: Open binary STL file in little-endian mode and register cleanup
    % Buoc 1: Mo file STL nhi phan theo dinh dang little-endian va tu dong dong file
    % -------------------------------------------------------------------------
    fid = fopen(path, 'rb', 'ieee-le');
    assert(fid ~= -1, 'scara:stl', 'Cannot open STL: %s', path);
    cleanup = onCleanup(@() fclose(fid));

    % -------------------------------------------------------------------------
    % Step 2: Skip 80-byte header, read triangle count `n`, and verify file size
    % Buoc 2: Bo qua 80 byte dau, doc so tam giac `n` va kiem tra kich thuoc file
    % -------------------------------------------------------------------------
    fseek(fid, 80, 'bof');
    n = fread(fid, 1, 'uint32');
    assert(~isempty(n) && n > 0 && n < 1e7, 'scara:stl', 'Invalid binary STL.');

    fseek(fid, 0, 'eof');
    assert(ftell(fid) == 84 + 50 * n, 'scara:stl', 'Unexpected STL length.');

    % -------------------------------------------------------------------------
    % Step 3: Read all 50-byte triangle records and unpack vertex coordinates
    % Buoc 3: Doc toan bo cac ban ghi 50 byte va giai nen toa do dinh tam giac
    % -------------------------------------------------------------------------
    fseek(fid, 84, 'bof');
    bytes = fread(fid, [50, n], '*uint8');

    % [EN] First 48 bytes of each column are 12 single-precision floats:
    %      columns 1..3 are the facet normal; columns 4..12 are the 3 vertices.
    % [VI] 48 byte dau cua moi cot la 12 so thuc single (float32):
    %      cot 1..3 la vector phap tuyen; cot 4..12 la toa do 3 dinh cua tam giac.
    floatValues = typecast(reshape(bytes(1:48, :), [], 1), 'single');
    data = reshape(floatValues, 12, [])';
    raw = reshape(data(:, 4:12)', 3, [])';

    % -------------------------------------------------------------------------
    % Step 4: Deduplicate vertices and construct the [Fx3] triangle connectivity
    % Buoc 4: Loc bo cac dinh trung lap va tao ma tran ket noi mat tam giac [Fx3]
    % -------------------------------------------------------------------------
    [vertices, ~, indices] = unique(double(raw), 'rows');
    faces = reshape(indices, 3, [])';
end
