"""
================================================================================
FILE: cartesian_nc_path.py
PROJECT: SCARA Cartesian Robot Control Software (Character & Drawing Studio)
================================================================================
[EN] PURPOSE:
     Offline stroke velocity lookahead planner and bounded, credit-based
     firmware FIFO upload state machine (`PathUpload`).
     - Retimes contiguous pen-down DDA segments using a quintic velocity-squared
       profile (`10*u^3 - 15*u^4 + 6*u^5`) without altering validated DDA step
       endpoints (no corner cutting).
     - Manages streaming up to 32 `SEG` commands into the STM32 hardware ring
       buffer before issuing `GO`, and refills slots as segments complete.

[VI] MỤC ĐÍCH:
     Bộ quy hoạch vận tốc nhìn trước (lookahead) cho nét vẽ và máy trạng thái
     tải bộ đệm FIFO có kiểm soát (`PathUpload`) xuống firmware.
     - Tính toán lại vận tốc đầu/cuối của các đoạn DDA hạ bút liên tiếp bằng
       quy luật đa thức bậc 5 (`10*u^3 - 15*u^4 + 6*u^5`) mà không làm thay đổi
       tọa độ xung DDA đã kiểm duyệt (không cắt góc).
     - Quản lý việc nạp trước tối đa 32 lệnh `SEG` vào bộ đệm vòng trên STM32
       trước khi phát lệnh `GO`, và tiếp tục nạp bù khi có ô trống.

[DEBUG TIP / MẸO GỠ LỖI]:
     - `smooth_square()` in this file MUST match the exact integer bit-shift
       math in `Firmware/ScaraCartesian/src/cart_core.c` (`smooth_rate()`),
       because both use 20-bit fixed-point (`1048576 = 1 << 20`)!
     - Hàm `smooth_square()` trong file này PHẢI khớp tuyệt đối với phép dịch bit
       số nguyên trong `Firmware/ScaraCartesian/src/cart_core.c` (`smooth_rate()`),
       vì cả hai đều dùng số học dấu phẩy tĩnh 20-bit (`1048576 = 1 << 20`)!
================================================================================
"""

from dataclasses import dataclass
import math

# ==============================================================================
# 1. MOTION PROFILE & FIFO CONSTANTS / HẰNG SỐ QUY HOẠCH CHUYỂN ĐỘNG & BỘ ĐỆM
# ==============================================================================

# [EN] Firmware path segment ring buffer capacity (32 segments).
# [VI] Sức chứa bộ đệm vòng chứa các đoạn đường chạy trên firmware (32 đoạn).
CAPACITY = 32

# [EN] Nominal Cartesian acceleration limit (mm/s^2) and virtual junction tolerance (mm).
# [VI] Giới hạn gia tốc chuẩn trong không gian Đề-các (mm/s^2) và dung sai nối góc ảo (mm).
ACCEL_MM_S2 = 20.0
JUNCTION_MM = 0.03


# ==============================================================================
# 2. QUINTIC S-CURVE FIXED-POINT MATH / TOÁN HỌC ĐƯỜNG CONG S BẬC 5 DẤU PHẨY TĨNH
# ==============================================================================


def ramp_ticks(peak, end, accel):
    """
    [EN] Compute the number of DDA ticks required to accelerate/decelerate
         between `end` (pps) and `peak` (pps) under peak acceleration `accel` (pps/s)
         using a quintic v^2 profile where peak factor = 15/8 = 1.875.
         Physics derivation:
         - In constant-acceleration motion: v_peak^2 - v_end^2 = 2 * a_avg * L
         - For the quintic S-curve s(u) = 10u^3 - 15u^4 + 6u^5, the steepest slope
           occurs at the midpoint u = 0.5, where ds/du = 15/8 = 1.875.
         - To keep instantaneous acceleration <= `accel`, the average acceleration
           is scaled by 8/15, so 2 * a_avg = (16 / 15) * accel.
         - Solving for distance L (in ticks) with integer ceiling division:
           ticks = ceil(15 * (peak^2 - end^2) / (16 * accel)).
    [VI] Tính số nhịp xung DDA cần thiết để tăng/giảm tốc giữa `end` (xung/s) và
         `peak` (xung/s) với gia tốc cực đại `accel` (xung/s^2) theo biên dạng
         bậc 5 của v^2 (hệ số đỉnh = 15/8 = 1.875).
         Giải thích vật lý:
         - Chuyển động biến đổi đều cơ bản: v_peak^2 - v_end^2 = 2 * a_tb * L
         - Với đường cong S bậc 5 s(u) = 10u^3 - 15u^4 + 6u^5, độ dốc lớn nhất nằm
           ở điểm giữa u = 0.5 với đạo hàm ds/du = 15/8 = 1.875.
         - Để gia tốc tức thời không vượt quá `accel`, gia tốc trung bình bằng
           (8/15) * accel, nên hệ số 2 * a_tb được thay bằng (16 / 15) * accel.
         - Công thức làm tròn lên số nguyên (ceiling):
           ticks = ceil(15 * (peak^2 - end^2) / (16 * accel)).
    """
    # [EN] Step 1: Difference of squared pulse rates (v_peak^2 - v_end^2).
    # [VI] Bước 1: Hiệu bình phương tốc độ xung (v_peak^2 - v_end^2).
    delta_v_squared = peak * peak - end * end

    # [EN] Step 2: Numerator and denominator for 15 * (peak^2 - end^2) / (16 * accel).
    # [VI] Bước 2: Tử số và mẫu số của biểu thức 15 * (peak^2 - end^2) / (16 * accel).
    numerator = 15 * delta_v_squared
    denominator = 16 * accel

    # [EN] Step 3: Integer ceiling division: ceil(A / B) = (A + B - 1) // B.
    # [VI] Bước 3: Phép chia nguyên làm tròn lên: ceil(A / B) = (A + B - 1) // B.
    required_ticks = (numerator + denominator - 1) // denominator
    return required_ticks


def smooth_square(low, high, tick, length):
    """
    [EN] Evaluate the instantaneous squared pulse rate `v^2(tick)` between `low^2`
         and `high^2` at step `tick` out of `length` using 20-bit fixed-point
         de Casteljau evaluation of the quintic polynomial `s(u) = 10u^3 - 15u^4 + 6u^5`.
         Why quintic Bezier?
         - Control points [0, 0, 0, 1, 1, 1] give zero 1st and 2nd derivatives
           at u = 0 and u = 1, eliminating jerk spikes at ramp transitions.
         - Using Q20 fixed-point (`1048576 = 1 << 20`) avoids floating-point drift
           and matches the STM32 C firmware bit-for-bit.
    [VI] Tính bình phương tốc độ xung tức thời `v^2(tick)` nội suy giữa `low^2`
         và `high^2` tại nhịp `tick` trên tổng độ dài dốc `length` bằng thuật toán
         de Casteljau dấu phẩy tĩnh 20-bit cho đa thức bậc 5 `s(u) = 10u^3 - 15u^4 + 6u^5`.
         Tại sao dùng đường cong Bezier bậc 5?
         - Các điểm kiểm soát [0, 0, 0, 1, 1, 1] giúp đạo hàm bậc 1 và bậc 2 bằng 0
           tại hai đầu u = 0 và u = 1, triệt tiêu độ giật (jerk) khi bắt đầu/kết thúc dốc.
         - Số học dấu phẩy tĩnh Q20 (`1048576 = 1 << 20`) tránh sai số dấu phẩy động
           và khớp tuyệt đối từng bit với mã C trên vi điều khiển STM32.
    """
    # [EN] If the ramp has zero length or `tick` has passed the ramp, return `high^2`.
    # [VI] Nếu đoạn dốc có độ dài bằng 0 hoặc `tick` đã vượt hết dốc, trả về `high^2`.
    if not length or tick >= length:
        return high * high

    # [EN] Fixed-point scale: 1.0 in Q20 format is 2^20 = 1048576.
    # [VI] Hệ số tỷ lệ dấu phẩy tĩnh: 1.0 ở định dạng Q20 là 2^20 = 1048576.
    q20_one = 1048576

    # [EN] Normalized progress `u` in Q20 fixed-point [0, 1048576].
    # [VI] Tiến độ chuẩn hóa `u` ở dạng dấu phẩy tĩnh Q20 [0, 1048576].
    u = tick * q20_one // length

    def blend(a, b):
        """
        [EN] Linear interpolation in Q20 fixed-point: a + (b - a) * u / 2^20.
        [VI] Nội suy tuyến tính trong hệ Q20: a + (b - a) * u / 2^20.
        """
        return a + ((b - a) * u >> 20)

    # [EN] De Casteljau subdivision levels for control points [0, 0, 0, 1, 1, 1]:
    # [VI] Các tầng chia nhỏ de Casteljau cho các điểm kiểm soát [0, 0, 0, 1, 1, 1]:
    p = blend(0, u)
    q = blend(u, q20_one)

    a = blend(0, p)
    b = blend(p, q)
    d = blend(q, q20_one)

    ab = blend(a, b)
    bd = blend(b, d)
    s = blend(ab, bd)

    # [EN] Interpolate squared velocity between `low^2` and `high^2` using `s`.
    # [VI] Nội suy bình phương vận tốc giữa `low^2` và `high^2` theo hệ số `s`.
    low_sq = low * low
    high_sq = high * high
    v_squared = low_sq + ((high_sq - low_sq) * s >> 20)
    return v_squared


# ==============================================================================
# 3. SEGMENT & STROKE DATA STRUCTURES / CẤU TRÚC DỮ LIỆU ĐOẠN & NÉT VẼ
# ==============================================================================


@dataclass(frozen=True)
class PathSegment:
    """
    [EN] One retimed DDA path segment ready for `SEG` upload to the STM32.
         - `steps`: Target motor step tuple `(z_steps, j1_steps, j2_steps)`.
         - `peak`: Maximum pulse rate (pulses/s) reached during this segment.
         - `entry`: Initial pulse rate (pulses/s) entering this segment.
         - `exit`: Final pulse rate (pulses/s) leaving this segment.
         - `accel`: Acceleration limit (pulses/s^2) along the dominant axis.
         - `ticks`: Total DDA ticks (dominant-axis step count) for this segment.
    [VI] Một đoạn đường chạy DDA đã được tính quy luật vận tốc, sẵn sàng gửi lệnh `SEG`.
         - `steps`: Bộ tọa độ xung đích `(z_steps, j1_steps, j2_steps)`.
         - `peak`: Tốc độ xung cực đại (xung/s) đạt được trong đoạn này.
         - `entry`: Tốc độ xung đầu vào (xung/s) khi bắt đầu đoạn.
         - `exit`: Tốc độ xung đầu ra (xung/s) khi kết thúc đoạn.
         - `accel`: Gia tốc giới hạn (xung/s^2) theo trục có số bước lớn nhất.
         - `ticks`: Tổng số nhịp DDA (số xung của trục chính) trong đoạn này.
    """

    steps: tuple
    peak: int
    entry: int
    exit: int
    accel: int
    ticks: int

    @property
    def ramp_up(self):
        """
        [EN] Number of ticks spent accelerating from `entry` to `peak`.
        [VI] Số nhịp xung dùng để tăng tốc từ `entry` lên `peak`.
        """
        return ramp_ticks(self.peak, self.entry, self.accel)

    @property
    def ramp_down(self):
        """
        [EN] Number of ticks spent decelerating from `peak` to `exit`.
        [VI] Số nhịp xung dùng để giảm tốc từ `peak` xuống `exit`.
        """
        return ramp_ticks(self.peak, self.exit, self.accel)

    def command(self, session, job, index):
        """
        [EN] Format the ASCII `SEG` serial command string for this segment:
             `SEG <session> <job> <index> <z> <j1> <j2> <peak> <entry> <exit> <accel>`
        [VI] Định dạng chuỗi lệnh ASCII `SEG` truyền qua serial cho đoạn này:
             `SEG <session> <job> <index> <z> <j1> <j2> <peak> <entry> <exit> <accel>`
        """
        step_strings = []
        for step_val in self.steps:
            step_strings.append(str(step_val))
        steps_part = " ".join(step_strings)

        return (
            f"SEG {session} {job} {index} "
            + steps_part
            + f" {self.peak} {self.entry} {self.exit} {self.accel}"
        )

    def seconds(self):
        """
        [EN] Exact duration in seconds by summing `1 / rate(t)` over all DDA ticks.
             At each tick `t`, the active rate is bounded by both the acceleration
             ramp from `entry` and the deceleration ramp toward `exit`.
        [VI] Thời gian chạy chính xác (giây) bằng cách cộng tổng `1 / rate(t)` qua mọi nhịp DDA.
             Tại mỗi nhịp `t`, tốc độ xung thực tế bị giới hạn đồng thời bởi dốc
             tăng tốc từ `entry` và dốc giảm tốc về `exit`.
        """
        up = self.ramp_up
        down = self.ramp_down
        tick_durations = []

        for t in range(self.ticks):
            # [EN] Squared rate allowed by the entry acceleration ramp at tick `t`.
            # [VI] Bình phương tốc độ cho phép bởi dốc tăng tốc đầu vào tại nhịp `t`.
            v_sq_up = smooth_square(self.entry, self.peak, t, up)

            # [EN] Squared rate allowed by the exit deceleration ramp with `ticks - t` left.
            # [VI] Bình phương tốc độ cho phép bởi dốc giảm tốc đầu ra khi còn `ticks - t` nhịp.
            remaining_ticks = self.ticks - t
            v_sq_down = smooth_square(self.exit, self.peak, remaining_ticks, down)

            # [EN] Take the lower of the two ramp envelopes, integer square-root, and clamp >= 50 pps.
            # [VI] Chọn giá trị nhỏ hơn giữa hai nhánh dốc, lấy căn bậc hai nguyên và giữ >= 50 xung/s.
            bounded_v_sq = min(v_sq_up, v_sq_down)
            instantaneous_rate = max(50, math.isqrt(bounded_v_sq))

            # [EN] Time for a single tick at `instantaneous_rate` (pulses/s) is 1 / rate seconds.
            # [VI] Thời gian thực hiện 1 nhịp xung ở tốc độ `instantaneous_rate` là 1 / rate giây.
            tick_durations.append(1 / instantaneous_rate)

        return sum(tick_durations)


@dataclass(frozen=True)
class StrokeRun:
    """
    [EN] A contiguous sequence of pen-down segments executed as one continuous
         `PATH` job between setpoint index `first` and `end`.
    [VI] Một chuỗi các đoạn hạ bút liên tiếp được thực thi dưới dạng một công việc
         `PATH` liên tục từ chỉ số điểm `first` đến `end`.
    """

    first: int
    end: int
    start_steps: tuple
    segments: tuple
    seconds: float


# ==============================================================================
# 4. LOOKAHEAD VELOCITY PLANNER / BỘ QUY HOẠCH VẬN TỐC NHÌN TRƯỚC (LOOKAHEAD)
# ==============================================================================


def lookahead(plans, starts, points):
    """
    [EN] Perform forward/backward velocity lookahead across a contiguous stroke:
         - No corner cutting: preserves the exact validated DDA target steps.
         - Corner speeds are bounded by joint/Cartesian caps and centripetal
           junction tolerance (`JUNCTION_MM`), dropping to minimum rate at sharp
           vertices (> 35 deg turn angle).
         - Forward and backward passes guarantee every junction velocity is
           reachable within `ACCEL_MM_S2` (`v_exit^2 - v_entry^2 <= (16/15) * a * L`).
    [VI] Thực hiện quy hoạch vận tốc nhìn trước hai chiều (tiến/lùi) trên toàn nét vẽ:
         - Không cắt góc: giữ nguyên tuyệt đối tọa độ xung đích DDA đã kiểm duyệt.
         - Vận tốc tại đỉnh nối được giới hạn bởi tốc độ trần và dung sai hướng tâm
           (`JUNCTION_MM`), tự động giảm về tốc độ tối thiểu tại các góc ngoặt gấp (> 35 độ).
         - Hai lượt duyệt tiến và lùi đảm bảo mọi vận tốc chuyển tiếp đều đạt được
           trong giới hạn gia tốc `ACCEL_MM_S2` (`v_exit^2 - v_entry^2 <= (16/15) * a * L`).
    """
    lengths = []
    vectors = []
    ticks = []
    densities = []
    caps = []

    # --------------------------------------------------------------------------
    # [EN] STEP 1: Extract geometric length, unit direction vector, and pulse
    #              density (pulses per mm) for each segment in the stroke.
    # [VI] BƯỚC 1: Trích xuất chiều dài hình học, vector đơn vị chỉ phương và
    #              mật độ xung (số xung trên mỗi mm) cho từng đoạn trong nét vẽ.
    # --------------------------------------------------------------------------
    for plan, start, (point_a, point_b) in zip(plans, starts, points):
        length = math.dist(point_a, point_b)

        # [EN] Dominant-axis step count `n` = max(|delta_Z|, |delta_J1|, |delta_J2|).
        # [VI] Số xung của trục chính `n` = max(|delta_Z|, |delta_J1|, |delta_J2|).
        axis_deltas = []
        for target_step, start_step in zip(plan.steps, start):
            axis_deltas.append(abs(target_step - start_step))
        n = max(axis_deltas)

        if n == 0:
            continue
        if length <= 0:
            raise ValueError("Nét có đoạn không có chiều dài XY.")

        # [EN] Unit direction vector from `point_a` to `point_b`: (b - a) / length.
        # [VI] Vector đơn vị chỉ hướng từ `point_a` tới `point_b`: (b - a) / length.
        unit_vec = []
        for coord_a, coord_b in zip(point_a, point_b):
            unit_vec.append((coord_b - coord_a) / length)

        # [EN] `density` converts mm -> ticks; `cap` converts max pulse rate -> mm/s.
        # [VI] `density` quy đổi mm -> số xung; `cap` quy đổi tốc độ xung trần -> mm/s.
        pulse_density = n / length
        cartesian_speed_cap = plan.rate * length / n

        lengths.append(length)
        vectors.append(tuple(unit_vec))
        ticks.append(n)
        densities.append(pulse_density)
        caps.append(cartesian_speed_cap)

    if len(ticks) != len(plans):
        raise ValueError("Đoạn nét trùng xung; giảm mật độ điểm hoặc tăng cỡ chữ.")

    count = len(plans)
    # [EN] Junction velocities at the `count + 1` vertices (start and end stay 0.0).
    # [VI] Vận tốc tại `count + 1` đỉnh nối (điểm bắt đầu và kết thúc nét bằng 0.0).
    velocity = [0.0] * (count + 1)

    # --------------------------------------------------------------------------
    # [EN] STEP 2: Compute maximum allowable junction velocity at each interior
    #              corner using the cosine of the turn angle and `JUNCTION_MM`.
    # [VI] BƯỚC 2: Tính vận tốc chuyển tiếp tối đa cho phép tại mỗi góc nối bên
    #              trong dựa trên cosin góc lệch và dung sai hướng tâm `JUNCTION_MM`.
    # --------------------------------------------------------------------------
    sharp_corner_cos_limit = math.cos(math.radians(35))

    for i in range(1, count):
        prev_vector = vectors[i - 1]
        curr_vector = vectors[i]

        # [EN] Dot product of unit vectors gives cos(theta) of the turn angle.
        # [VI] Tích vô hướng của hai vector đơn vị cho giá trị cos(theta) của góc rẽ.
        dot_product = 0.0
        for comp_prev, comp_curr in zip(prev_vector, curr_vector):
            dot_product += comp_prev * comp_curr
        cosine = max(-1, min(1, dot_product))

        # [EN] If the turn angle is sharper than 35 degrees, force a full stop (minimum rate).
        # [VI] Nếu góc rẽ gắt hơn 35 độ, ép giảm tốc về mức tối thiểu tại đỉnh góc.
        if cosine < sharp_corner_cos_limit:
            continue

        # [EN] Half-angle cosine: cos(theta / 2) = sqrt((1 + cos(theta)) / 2).
        #      Virtual circular blend radius R = JUNCTION_MM * half / (1 - half),
        #      giving centripetal speed limit v = sqrt(ACCEL_MM_S2 * R).
        # [VI] Cosin nửa góc: cos(theta / 2) = sqrt((1 + cos(theta)) / 2).
        #      Bán kính cong ảo tại góc nối R = JUNCTION_MM * half / (1 - half),
        #      suy ra giới hạn vận tốc hướng tâm v = sqrt(ACCEL_MM_S2 * R).
        half = math.sqrt((1 + cosine) / 2)
        if 1 - half < 1e-9:
            junction = math.inf
        else:
            junction = math.sqrt(ACCEL_MM_S2 * JUNCTION_MM * half / (1 - half))

        velocity[i] = min(caps[i - 1], caps[i], junction)

    # --------------------------------------------------------------------------
    # [EN] STEP 3: Forward acceleration pass & backward deceleration pass.
    #              Physics constraint: v_next^2 <= v_curr^2 + (16/15) * a * L.
    #              (16/15 replaces 2 so peak quintic acceleration stays <= ACCEL_MM_S2).
    # [VI] BƯỚC 3: Lượt duyệt tăng tốc chiều thuận & lượt duyệt giảm tốc chiều ngược.
    #              Ràng buộc vật lý: v_next^2 <= v_curr^2 + (16/15) * a * L.
    #              (Hệ số 16/15 thay cho 2 để gia tốc đỉnh bậc 5 luôn <= ACCEL_MM_S2).
    # --------------------------------------------------------------------------
    for i in range(count):
        max_reachable_exit = math.sqrt(
            velocity[i] ** 2 + (16 / 15) * ACCEL_MM_S2 * lengths[i]
        )
        velocity[i + 1] = min(velocity[i + 1], max_reachable_exit)

    for i in reversed(range(count)):
        max_reachable_entry = math.sqrt(
            velocity[i + 1] ** 2 + (16 / 15) * ACCEL_MM_S2 * lengths[i]
        )
        velocity[i] = min(velocity[i], max_reachable_entry)

    # --------------------------------------------------------------------------
    # [EN] STEP 4: Convert Cartesian velocities (mm/s) into integer pulse rates
    #              (entry, peak, exit, accel) and binary-search the peak rate.
    # [VI] BƯỚC 4: Quy đổi vận tốc Đề-các (mm/s) sang tốc độ xung nguyên
    #              (entry, peak, exit, accel) và tìm kiếm nhị phân tốc độ đỉnh.
    # --------------------------------------------------------------------------
    segments = []
    for i, plan in enumerate(plans):
        peak = plan.rate
        density = densities[i]
        seg_ticks = ticks[i]

        # [EN] Clamp integer pulse rates to the firmware floor (50 pps) and segment cap.
        # [VI] Giới hạn tốc độ xung nguyên trong khoảng từ mức sàn (50 xung/s) đến mức trần.
        entry = max(50, min(peak, math.floor(velocity[i] * density)))
        exit = max(50, min(peak, math.floor(velocity[i + 1] * density)))
        accel = max(50, min(200000, math.ceil(ACCEL_MM_S2 * density)))

        # [EN] Ensure `entry` and `exit` rates are mutually reachable within `seg_ticks`.
        # [VI] Đảm bảo tốc độ `entry` và `exit` có thể biến thiên kịp trong số xung `seg_ticks`.
        if entry > exit:
            max_entry_sq = exit * exit + 16 * accel * seg_ticks // 15
            entry = min(entry, math.isqrt(max_entry_sq))
        else:
            max_exit_sq = entry * entry + 16 * accel * seg_ticks // 15
            exit = min(exit, math.isqrt(max_exit_sq))

        # [EN] Binary search for the highest `peak` rate whose up + down ramps fit in `seg_ticks`.
        # [VI] Tìm kiếm nhị phân tốc độ đỉnh `peak` lớn nhất sao cho tổng dốc tăng + giảm <= `seg_ticks`.
        lo = max(entry, exit)
        hi = peak
        while lo < hi:
            mid = (lo + hi + 1) // 2
            total_ramp_ticks = ramp_ticks(mid, entry, accel) + ramp_ticks(
                mid, exit, accel
            )
            if total_ramp_ticks <= seg_ticks:
                lo = mid
            else:
                hi = mid - 1

        peak = lo
        segments.append(PathSegment(plan.steps, peak, entry, exit, accel, seg_ticks))

    return tuple(segments)


# ==============================================================================
# 5. BOUNDED FIRMWARE FIFO UPLOADER / BỘ TẢI ĐƯỜNG CHẠY FIFO CÓ KIỂM SOÁT
# ==============================================================================


class PathUpload:
    """
    [EN] Bounded, credit-based firmware FIFO upload state machine:
         1. Wait for `PATH` acknowledgement.
         2. Upload the initial preload batch (up to `CAPACITY = 32` segments).
         3. Once all preloaded segments are acknowledged, transmit `GO` to start
            continuous hardware timer execution.
         4. As segments complete on the STM32 and free up ring-buffer slots
            (`self.free`), stream remaining `SEG` commands without overflowing
            the 32-slot hardware queue.
         Queue starvation is treated as a firmware fault, never a silent pause.
    [VI] Máy trạng thái tải đoạn đường chạy vào bộ đệm FIFO theo cơ chế tín dụng (credit):
         1. Chờ xác nhận lệnh khởi tạo `PATH`.
         2. Nạp lô đoạn đầu tiên (tối đa `CAPACITY = 32` đoạn) vào bộ đệm STM32.
         3. Khi toàn bộ lô đầu tiên đã được xác nhận (`ACK`), phát lệnh `GO` để
            bắt đầu chạy liên tục bằng ngắt phần cứng.
         4. Khi STM32 chạy xong từng đoạn và giải phóng ô nhớ (`self.free`), tiếp tục
            bơm các lệnh `SEG` còn lại mà không làm tràn bộ đệm 32 ô.
         Cạn bộ đệm giữa chừng được coi là lỗi phần cứng, không bao giờ tự dừng ngầm.
    """

    def __init__(self, session, job, segments, window=1):
        self.session = session
        self.job = job
        self.segments = segments
        self.window = max(1, min(CAPACITY, int(window)))
        self.sent = 0
        self.acked = 0
        self.in_flight = []
        self.go_sent = False
        self.started = False
        self.free = CAPACITY
        self.status_received = 0

    @property
    def pending(self):
        """
        [EN] Oldest unacknowledged command opcode (`"PATH"`, `"SEG"`, or `"GO"`).
        [VI] Mã lệnh cũ nhất đang chờ STM32 xác nhận (`"PATH"`, `"SEG"`, hoặc `"GO"`).
        """
        if self.in_flight:
            return self.in_flight[0]
        return None

    @pending.setter
    def pending(self, value):
        if value is None:
            self.in_flight.clear()
        elif self.in_flight:
            self.in_flight[0] = value
        else:
            self.in_flight.append(value)

    def status(self, state):
        """
        [EN] Update FIFO credit tracking from an incoming `NCStatus` telemetry frame.
             - `state.path[1]` is the number of segments received by the STM32 so far.
             - `state.path[3]` is the current free slot count in the 32-slot FIFO.
        [VI] Cập nhật số ô trống bộ đệm FIFO từ khung tin trạng thái `NCStatus` nhận về.
             - `state.path[1]` là tổng số đoạn `SEG` mà STM32 đã nhận được.
             - `state.path[3]` là số ô trống hiện có trong bộ đệm FIFO 32 ô.
        """
        if state.job != self.job:
            return
        self.status_received = state.path[1]
        self.free = state.path[3]

    def ack(self, ack):
        """
        [EN] Process an incoming `Ack` for `PATH`, `SEG`, or `GO`.
             Raises `ValueError(ack.reason)` if the firmware rejected the command.
        [VI] Xử lý gói `Ack` cho lệnh `PATH`, `SEG`, hoặc `GO`.
             Ném ngoại lệ `ValueError(ack.reason)` nếu firmware từ chối lệnh.
        """
        # [EN] Verify that this ACK belongs to our session, job, and oldest in-flight opcode.
        # [VI] Kiểm tra gói ACK có khớp phiên, mã công việc và lệnh đang chờ cũ nhất không.
        if ack.session != self.session or ack.job != self.job:
            return False
        if not self.in_flight or ack.op != self.in_flight[0]:
            return False

        # [EN] Remove the acknowledged command from the in-flight queue.
        # [VI] Lấy lệnh vừa được xác nhận ra khỏi hàng đợi đang bay.
        self.in_flight.pop(0)

        if not ack.ok:
            self.in_flight.clear()
            raise ValueError(ack.reason)

        if ack.op == "SEG":
            self.acked += 1
        if ack.op == "GO":
            self.started = True

        return True

    def next(self):
        """
        [EN] Return the next ASCII command (`SEG ...` or `GO ...`) to transmit if
             both the in-flight window and firmware FIFO credits allow, or `None`.
        [VI] Trả về dòng lệnh ASCII tiếp theo (`SEG ...` hoặc `GO ...`) cần gửi nếu
             cửa sổ lệnh đang bay và số ô trống FIFO cho phép, hoặc trả về `None`.
        """
        # [EN] Do not pipeline commands while waiting for `PATH` or `GO` acknowledgement.
        # [VI] Không gửi chồng lệnh khi đang chờ xác nhận lệnh `PATH` hoặc `GO`.
        if self.in_flight and self.in_flight[0] in ("PATH", "GO"):
            return None

        # [EN] Respect the maximum allowed in-flight command window.
        # [VI] Tuân thủ giới hạn số lệnh tối đa được phép gửi chờ xác nhận cùng lúc.
        if len(self.in_flight) >= self.window:
            return None

        # [EN] Issue `GO` once the initial batch (up to 32 segments) is fully acknowledged.
        # [VI] Phát lệnh `GO` ngay khi lô đoạn đầu tiên (tối đa 32 đoạn) đã được xác nhận đủ.
        initial_preload_target = min(CAPACITY, len(self.segments))
        if (
            not self.go_sent
            and not self.in_flight
            and self.acked >= initial_preload_target
        ):
            self.in_flight.append("GO")
            self.go_sent = True
            return f"GO {self.session} {self.job}"

        # [EN] Compute remaining guaranteed FIFO slots on STM32:
        #      `unreported_sent` accounts for SEG commands sent after the last status frame.
        # [VI] Tính số ô trống FIFO chắc chắn còn lại trên STM32:
        #      `unreported_sent` là số lệnh SEG đã gửi thêm kể từ bản tin status gần nhất.
        unreported_sent = max(0, self.sent - self.status_received)
        credits = self.free - unreported_sent

        if self.sent < len(self.segments) and credits > 0:
            segment = self.segments[self.sent]
            line = segment.command(self.session, self.job, self.sent)
            self.sent += 1
            self.in_flight.append("SEG")
            return line

        return None
