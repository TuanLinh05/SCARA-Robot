"""
============================================================================
FILE: cartesian_nc_upgrade.py
============================================================================
[EN] Configuration Schema Migration Helpers (R1 -> R14)
     Upgrades older `cartesian_nc_config.json` dictionaries across software
     revisions while preserving user-calibrated geometry and microstep settings.

[VI] Các Hàm Nâng cấp Phiên bản Cấu hình Tự động (R1 -> R14)
     Chuyển đổi dữ liệu cấu hình từ các phiên bản cũ sang cấu trúc mới nhất
     mà vẫn giữ nguyên thông số cơ khí và vi bước người dùng đã cài đặt.
============================================================================
"""

import json


def upgrade_r3(data):
    """
    [EN] Upgrade config to R3: compute `z_home_pps` for 1 mm/s Z profile and enable background home.
    [VI] Nâng cấp lên R3: tính tốc độ xung dò Z và bật chế độ chạy Home dưới nền.
    """
    result = dict(data)
    micro_z = result.get("microsteps", (16, 8, 8))[0]
    lead = result.get("lead_mm_rev", 2.0)
    factors = 200 * micro_z / lead

    result["z_home_pps"] = round(factors)
    result["max_z_speed_mm_s"] = 1.0
    current_master_rate = result.get("max_master_rate", 1600)
    result["max_master_rate"] = max(current_master_rate, round(factors))
    result.setdefault("z_latch_pps", max(50, round(factors * 0.25)))
    result.setdefault("home_in_background", True)
    return result


def upgrade_r4(data):
    """
    [EN] Upgrade config to R4: add default `z_latch_pps`, `j2_home_pps`, and `j2_latch_pps`.
    [VI] Nâng cấp lên R4: bổ sung tốc độ chốt công tắc Z và tốc độ dò/chốt trục J2.
    """
    result = dict(data)
    default_z_latch = min(400, result.get("z_home_pps", 1600))
    result.setdefault("z_latch_pps", default_z_latch)
    result.setdefault("home_in_background", True)
    result.setdefault("j2_home_pps", 800)
    result.setdefault("j2_latch_pps", 200)
    return result


def upgrade_r5(data):
    """
    [EN] Upgrade config to R5: set nominal belt coupling ratio `0.333333`.
    [VI] Nâng cấp lên R5: đặt hệ số bù truyền động đai danh định `0.333333`.
    """
    result = upgrade_r4(data)
    result["coupling"] = 0.333333
    return result


def upgrade_r6(data):
    """
    [EN] Upgrade config to R6: normalize J2 `positive_high` polarity opposite to J1.
    [VI] Nâng cấp lên R6: chuẩn hóa cực tính chân DIR của J2 ngược với J1.
    """
    result = upgrade_r5(data)
    polarity = list(result.get("positive_high", (True, True, True)))
    polarity[2] = not polarity[1]
    result["positive_high"] = polarity
    return result


def upgrade_r7(data):
    """
    [EN] Upgrade config to R7: enable automatic belt coupling probe (`auto_coupling = True`).
    [VI] Nâng cấp lên R7: bật tính năng tự động nhích đo tỷ số truyền đai (`auto_coupling`).
    """
    result = upgrade_r4(data)
    result.setdefault("auto_coupling", True)
    result.setdefault("coupling_probe_pulses", 128)
    return result


def upgrade_r10(data):
    """
    [EN] Upgrade config to R10: allow normal motion to continue in the background.
    [VI] Nâng cấp lên R10: cho phép giữ chuyển động chạy nền khi mất tiêu điểm cửa sổ.
    """
    result = upgrade_r7(data)
    result.setdefault("motion_in_background", True)
    return result


def upgrade_r12(data):
    """
    [EN] Upgrade config to R12: replace raw `z_home_pps` with physical `z_home_speed_mm_s = 2.0`.
    [VI] Nâng cấp lên R12: thay thông số xung `z_home_pps` bằng vận tốc vật lý `z_home_speed_mm_s = 2.0`.
    """
    result = upgrade_r10(data)
    result.pop("z_home_pps", None)
    result.setdefault("z_home_speed_mm_s", 2.0)
    return result


def upgrade_r14(data):
    """
    [EN] Upgrade config to R14: add 2D end-effector `tool_offset_mm` `[0.0, 0.0]`.
    [VI] Nâng cấp lên R14: bổ sung độ lệch đầu bút `tool_offset_mm` `[0.0, 0.0]`.
    """
    result = upgrade_r12(data)
    result.setdefault("tool_offset_mm", [0.0, 0.0])
    return result
