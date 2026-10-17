"""
================================================================================
FILE: cartesian_nc_audit.py
PROJECT: SCARA Cartesian Robot Control Software (Character & Drawing Studio)
================================================================================
[EN] PURPOSE:
     Offline, read-only audit tool for `.jsonl` session logs recorded by `NCApp`.
     Reconstructs calibration parameters, recompiles drawing strokes, verifies
     that closed strokes end at their exact starting step coordinates, checks
     total pulse counts (`state.total`) and ISR timer-late counters, and exports
     a JSON + Vietnamese Markdown diagnostic report. No serial ports are opened.

[VI] MỤC ĐÍCH:
     Công cụ kiểm toán (audit) ngoại tuyến, chỉ đọc cho các file nhật ký phiên
     `.jsonl` được ghi lại bởi `NCApp`.
     Tái lập thông số hiệu chuẩn, biên dịch lại các nét vẽ, đối chiếu tọa độ xung
     đầu/cuối của các nét khép kín, kiểm tra tổng số xung (`state.total`) và bộ đếm
     trễ định thời ISR (`timer_late`), sau đó xuất báo cáo chẩn đoán định dạng
     JSON và Markdown tiếng Việt. Không mở cổng COM và không điều khiển robot.

[DEBUG TIP / MẸO GỠ LỖI]:
     - Run `python Software/src/cartesian_nc_audit.py <path/to/session.jsonl> --out report`
       after a real drawing run to verify whether any stepper pulses were lost in
       software (`axis_pulse_error`) or if the STM32 timer ISR ran late (`timer_late_delta`).
     - Chạy lệnh `python Software/src/cartesian_nc_audit.py <đường_dẫn_log.jsonl> --out report`
       sau khi vẽ thật để kiểm tra xem phần mềm có bị lệch xung nào không
       (`axis_pulse_error`) hoặc ngắt timer trên STM32 có bị trễ nhịp nào không (`timer_late_delta`).
================================================================================
"""

from pathlib import Path
from dataclasses import asdict
import argparse
import json
import math
from cartesian_nc_model import NCConfig
from cartesian_nc_protocol import NCStatus
from cartesian_nc_drawing import BKSettings, compile_bk


# ==============================================================================
# 1. SESSION LOG PARSER & AUDITOR / BỘ PHÂN TÍCH & KIỂM TOÁN FILE LOG (.JSONL)
# ==============================================================================


def audit_log(path):
    """
    [EN] Parse a recorded `.jsonl` session log file and return a comprehensive
         audit dictionary summarizing calibration epochs, drawing strokes, pulse
         accuracy, and timer-late diagnostics.
    [VI] Đọc và phân tích file log phiên `.jsonl`, trả về từ điển kiểm toán chi tiết
         tóm tắt các lần lấy gốc hiệu chuẩn, các nét vẽ, độ chính xác số xung
         và chẩn đoán trễ bộ định thời.
    """
    config = None
    state = None
    current = None
    drawing_index = -1
    strokes = {}
    drawings = []
    homes = []
    warnings = []
    status_count = 0
    rejected = 0
    last_home_epoch = None

    with Path(path).open(encoding="utf-8") as stream:
        for line in stream:
            try:
                event = json.loads(line)
            except ValueError:
                rejected += 1
                continue

            kind = event.get("type")

            # ------------------------------------------------------------------
            # [EN] 1. Connection header event: load mechanical config snapshot.
            # [VI] 1. Sự kiện khởi tạo kết nối: nạp bản chụp cấu hình cơ khí.
            # ------------------------------------------------------------------
            if kind == "connection":
                if event.get("demo"):
                    return {"file": str(path), "demo": True, "drawings": []}
                config = NCConfig.from_dict(event["config"])

            # ------------------------------------------------------------------
            # [EN] 2. Periodic status telemetry snapshot: reconstruct `NCStatus`.
            # [VI] 2. Bản chụp trạng thái telemetry định kỳ: tái tạo `NCStatus`.
            # ------------------------------------------------------------------
            elif kind == "status":
                data = event["status"]
                status_fields = {}
                for key, val in data.items():
                    if isinstance(val, list):
                        status_fields[key] = tuple(val)
                    else:
                        status_fields[key] = val
                state = NCStatus(**status_fields)
                status_count += 1

                # [EN] Record each newly completed homing/calibration epoch (`stage == 8`).
                #      - `state.factor` is in Q10 fixed-point (`1024 = 2^10`).
                #      - `state.coupling_ppm` is in parts-per-million (`1000000`).
                #      - `state.beta_q` is in Q20 fixed-point (`1048576 = 2^20`).
                # [VI] Ghi nhận mỗi phiên lấy gốc hoàn tất mới (`stage == 8`).
                #      - `state.factor` lưu ở dạng dấu phẩy tĩnh Q10 (`1024 = 2^10`).
                #      - `state.coupling_ppm` lưu theo phần triệu (`1000000`).
                #      - `state.beta_q` lưu ở dạng dấu phẩy tĩnh Q20 (`1048576 = 2^20`).
                if (
                    state.referenced
                    and state.stage == 8
                    and state.epoch != last_home_epoch
                ):
                    last_home_epoch = state.epoch
                    factors_pp_unit = []
                    for factor_q10 in state.factor:
                        factors_pp_unit.append(factor_q10 / 1024)

                    homes.append(
                        dict(
                            epoch=state.epoch,
                            factors_pp_unit=factors_pp_unit,
                            range=list(state.range),
                            n1=list(state.n1),
                            n2=list(state.n2),
                            coupling=state.coupling_ppm / 1000000,
                            beta=state.beta_q / 1048576,
                            probe_da=state.probe_da,
                            probe_db=state.probe_db,
                            timer_late=state.timer_late,
                        )
                    )

            # ------------------------------------------------------------------
            # [EN] 3. Rejected serial line event.
            # [VI] 3. Sự kiện dòng dữ liệu serial bị từ chối.
            # ------------------------------------------------------------------
            elif kind == "rx_rejected":
                rejected += 1

            # ------------------------------------------------------------------
            # [EN] 4. Start of a Drawing Studio job: recompile to verify model error.
            # [VI] 4. Bắt đầu công việc vẽ hình: biên dịch lại để kiểm tra sai số mô hình.
            # ------------------------------------------------------------------
            elif kind == "drawing_start" and config and state:
                drawing_index += 1
                current = event["settings"]
                model = config.model(state)
                program = compile_bk(
                    model, state.pos, BKSettings(**current), event.get("dry", False)
                )
                drawings.append(
                    dict(
                        pattern=current["pattern"],
                        settings=current,
                        model_error_mm=program.max_model_error_mm,
                        strokes=[],
                        completed=False,
                        timer_late_at_start=state.timer_late,
                        measured_factors=list(model.factors),
                        coupling=model.coupling,
                    )
                )

            # ------------------------------------------------------------------
            # [EN] 5. Start of a continuous `PATH` stroke: tally expected pulses
            #         and direction reversals across all segments.
            # [VI] 5. Bắt đầu một nét vẽ liên tục `PATH`: cộng tổng số xung dự kiến
            #         và đếm số lần đảo chiều quay trên từng trục.
            # ------------------------------------------------------------------
            elif kind == "stroke_start" and state and current:
                profiles = event["profiles"]
                start = tuple(state.pos)
                end = tuple(profiles[-1]["steps"])
                position = start
                reversals = [0, 0, 0]
                previous = [0, 0, 0]
                expected = [0, 0, 0]
                is_constant_z = True

                for segment in profiles:
                    seg_steps = segment["steps"]
                    if seg_steps[0] != start[0]:
                        is_constant_z = False

                    for axis_idx in range(3):
                        d = seg_steps[axis_idx] - position[axis_idx]
                        expected[axis_idx] += abs(d)
                        if d > 0:
                            sign = 1
                        elif d < 0:
                            sign = -1
                        else:
                            sign = 0

                        if sign and previous[axis_idx] and sign != previous[axis_idx]:
                            reversals[axis_idx] += 1
                        if sign:
                            previous[axis_idx] = sign

                    position = tuple(seg_steps)

                record = dict(
                    job=event["job"],
                    start_steps=list(start),
                    target_steps=list(end),
                    commanded_closed=start == end,
                    segments=len(profiles),
                    expected_axis_pulses=expected,
                    direction_reversals=reversals,
                    constant_z=is_constant_z,
                    timer_late_at_start=state.timer_late,
                    start_total=list(state.total),
                )
                strokes[event["job"]] = record
                drawings[drawing_index]["strokes"].append(record)

            # ------------------------------------------------------------------
            # [EN] 6. Completion of an individual continuous `PATH` stroke:
            #         compare actual final steps and total pulses against expected.
            # [VI] 6. Hoàn thành một nét vẽ liên tục `PATH`:
            #         đối chiếu tọa độ xung cuối và tổng xung thực tế với dự kiến.
            # ------------------------------------------------------------------
            elif kind == "stroke_complete" and event["job"] in strokes:
                record = strokes[event["job"]]
                endpoint_error = []
                for actual_pos, target_pos in zip(event["pos"], record["target_steps"]):
                    endpoint_error.append(actual_pos - target_pos)

                record.update(
                    end_steps=event["pos"],
                    step_endpoint_error=endpoint_error,
                    timer_late_delta=event["timer_late"]
                    - record["timer_late_at_start"],
                    starved=event["starved"],
                    elapsed_s=event["elapsed_s"],
                    predicted_s=event["estimated_s"],
                )
                if state and state.job == event["job"]:
                    counted = []
                    for curr_total, init_total in zip(
                        state.total, record["start_total"]
                    ):
                        counted.append(curr_total - init_total)

                    pulse_error = []
                    for actual_cnt, exp_cnt in zip(
                        counted, record["expected_axis_pulses"]
                    ):
                        pulse_error.append(actual_cnt - exp_cnt)

                    record["reported_axis_pulses"] = counted
                    record["axis_pulse_error"] = pulse_error

            # ------------------------------------------------------------------
            # [EN] 7. Completion of the entire drawing job.
            # [VI] 7. Hoàn tất toàn bộ bản vẽ.
            # ------------------------------------------------------------------
            elif kind == "drawing_complete" and drawing_index >= 0:
                drawings[drawing_index].update(
                    completed=True,
                    elapsed_s=event["elapsed_s"],
                    timer_late_at_end=event["status"]["timer_late"],
                    starved=event["status"]["starved"],
                )

    if config:
        warnings.append(
            "Measured pulse/angle factors assume the configured switch endpoint angles; those angles were not measured by this log."
        )
        warnings.append(
            "Telemetry counts commanded pulses, not encoder position. Backlash, missed driver pulses/steps, pen flex/contact and actual dimensions are not measured."
        )

    return dict(
        file=str(Path(path).resolve()),
        demo=False,
        config=asdict(config) if config else None,
        status_frames=status_count,
        rejected_frames=rejected,
        home=homes,
        drawings=drawings,
        limitations=warnings,
    )


# ==============================================================================
# 2. MARKDOWN REPORT GENERATOR / HÀM XUẤT BÁO CÁO MARKDOWN
# ==============================================================================


def markdown_report(report):
    """
    [EN] Format an audit dictionary produced by `audit_log()` into a human-readable
         Vietnamese Markdown summary table.
    [VI] Định dạng từ điển kết quả từ `audit_log()` thành bảng báo cáo tóm tắt
         Markdown tiếng Việt dễ đọc.
    """
    lines = [
        "# Kiểm tra log nét vẽ",
        "",
        f"Nguồn: `{report['file']}`",
        "",
        "Báo cáo đọc lệnh và telemetry. Không mở COM và không di chuyển robot.",
        "",
        "| Mẫu | Cỡ ngang (mm) | Sai lệch mô hình tối đa (mm) | Hoàn tất | Trễ timer tăng lúc vẽ |",
        "|---|---:|---:|---|---:|",
    ]
    for d in report["drawings"]:
        end_late = d.get("timer_late_at_end", d["timer_late_at_start"])
        delay = end_late - d["timer_late_at_start"]
        completed_label = "Có" if d["completed"] else "Chưa"
        size_mm = d["settings"]["size_mm"]
        model_err = d["model_error_mm"]
        lines.append(
            f"| {d['pattern']} | {size_mm:g} | {model_err:.4f} | {completed_label} | {delay} |"
        )

    # [EN] Count how many closed strokes finished at their exact starting step coordinates.
    # [VI] Đếm số nét vẽ khép kín kết thúc đúng tại tọa độ xung điểm đầu.
    closed = []
    for d in report["drawings"]:
        for s in d["strokes"]:
            if s["commanded_closed"]:
                closed.append(s)

    verified = 0
    for s in closed:
        if "step_endpoint_error" in s and not any(s["step_endpoint_error"]):
            verified += 1

    lines.extend(
        (
            "",
            f"Nét đóng có telemetry xác nhận đúng tọa độ xung cuối: {verified}/{len(closed)}; xem JSON để đối chiếu từng nét.",
            "",
            "Đây không phải phép đo vị trí vật lý. Log không xác nhận motor/bút đã đi đúng các xung đó.",
            "",
            "Điểm cần đo tiếp: góc thật của hai đầu hành trình, khoảng cách tâm J1–J2 và tâm J2–đầu bút thực; độ lệch/độ rơ khi đảo chiều và lực tì bút.",
            "",
        )
    )
    return "\n".join(lines)


# ==============================================================================
# 3. CLI ENTRY POINT / HÀM KHỞI CHẠY DÒNG LỆNH
# ==============================================================================


def main():
    """
    [EN] CLI entry point to audit a `.jsonl` log file and write `.json` and `.md` reports.
    [VI] Điểm vào dòng lệnh để kiểm toán file log `.jsonl` và xuất báo cáo `.json` + `.md`.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("log")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    result = audit_log(args.log)
    target = Path(args.out)

    json_path = target.with_suffix(".json")
    md_path = target.with_suffix(".md")

    json_path.write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    md_path.write_text(markdown_report(result), encoding="utf-8")

    print(
        json.dumps(
            {
                "drawings": len(result["drawings"]),
                "demo": result["demo"],
                "report": str(md_path),
            }
        )
    )


if __name__ == "__main__":
    main()
