<div align="center">

# SCARA Drawing Robot

**A 2R + Z pen plotter based on X-SCARA mechanics · GUI R15 · Firmware R9**<br>
**Robot vẽ 2R + Z trên cơ khí X-SCARA · GUI R15 · Firmware R9**

Python/Tkinter · STM32F103C8 / Zephyr · MATLAB/Simulink · Webots

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)

<img src="media/drawing-demo.gif" alt="Real SCARA robot drawing a flower / Robot SCARA thực vẽ hoa" height="360">
&nbsp;
<img src="media/robot-wired.jpg" alt="Assembled and wired robot / Robot đã lắp ráp và đấu dây" height="360">

<sub>Real robot photos and video from earlier runs. [Original video](media/drawing-demo.mp4)<br>
Ảnh và video robot thực từ các lần chạy trước. [Video gốc](media/drawing-demo.mp4)</sub>

**[English](#english)** · **[Tiếng Việt](#tiếng-việt)**

</div>

# English

This **HPMR course project, semester HK261**, controls a SCARA robot that draws shapes
and text on paper. The frame and arm use the [madl3x/x-scara](https://github.com/madl3x/x-scara)
design; work developed in this project includes the firmware, GUI, simulation and Deli EG118 pen holder.

## Following this branch

Branch: **`codex/scara-r15-upgrade-20261017`**.
Initial upgrade: [`4a8317a`](https://github.com/TuanLinh05/SCARA-Robot/commit/4a8317ae81dedec2ba4cc48bac335332e0eb9d4c), dated **17 October 2026**.

| Component | Status on this branch |
| --- | --- |
| GUI | R15, runs from Python source; includes a demo mode |
| Firmware | Identifier `SCARA_CART_NC_HOME_V5_R9`, protocol 5; this branch includes further source changes |
| MATLAB/Simulink | Stepper model `scara_stepper.slx`, using a calibration snapshot from recorded logs |
| Webots | Separate physics research model; parameters are not yet aligned with the real robot's calibration |
| EXE packaging | Script/spec still use R14 names; R15 packaging has not been revalidated |

**Quick navigation:** [Changes](#main-changes) · [Run the GUI](#running-the-gui) ·
[Operation](#drawing-workflow) · [Simulation](#simulation) ·
[Testing](#testing) · [Source guide](#repository-layout-and-source-entry-points).

## Main changes

| Area | Changes | Source to follow |
| --- | --- | --- |
| R15 interface | Switch between 2D/3D views, separate 3D window, orbit/pan/zoom and SVG export; six metric cards and six switch indicators | [3D](Software/src/cartesian_nc_3d.py), [layout](Software/src/cartesian_nc_layout.py), [view](Software/src/cartesian_nc_view.py) |
| Motion control | Step-jog, quick poses, speed adjustment and quick HOME after calibration | [editor](Software/src/cartesian_nc_editor.py), [motion](Software/src/cartesian_nc_motion.py) |
| Drawing and file import | Single-stroke A–Z/0–9 text, SPIRAL/GRID/ROSE/POLYGON patterns, SVG/DXF/G-code, stroke ordering and background planning | [font](Software/src/cartesian_nc_font.py), [vector](Software/src/cartesian_nc_vector.py), [planning](Software/src/cartesian_nc_planning.py) |
| Calibration | Backlash estimation/compensation, two-point switch-angle calibration and three-point paper-plane fitting | [calibration](Software/src/cartesian_nc_calibration.py), [model](Software/src/cartesian_nc_model.py) |
| Data | Coordinate, joint-angle, speed/pulse-rate/FIFO plots; JSONL replay and CSV/SVG/Markdown export | [analytics](Software/src/cartesian_nc_analytics.py), [audit](Software/src/cartesian_nc_audit.py) |
| Firmware | Full/quick HOME, belt-coupling handling, backlash compensation and telemetry; segment upload governed by ACKs and FIFO capacity | [core](Firmware/ScaraCartesian/src/cart_core.c), [commands](Firmware/ScaraCartesian/src/cart_commands.c), [upload](Software/src/cartesian_nc_path.py) |
| Simulation and cleanup | MATLAB changes from PID/dynamics to replaying planner step counts; Webots controller updates; removal of `Firmware/Ref` and old MATLAB examples/models | [Simulation](Simulation/README.md), [MATLAB](Simulation/matlab/README.md) |

The GUI's 3D view is a simplified visualization with dimensions referenced from STL assets.
Telemetry and displayed positions are derived from step counts; the real robot has no encoders.

## Running the GUI

Run the following PowerShell commands from the **repository root**, where this README is located.
The tested environment uses **Python 3.12**, Tkinter and `pyserial`.

### Start with the demo

```powershell
python -m pip install -r Software/requirements.txt
python Software/src/cartesian_nc_control.py --demo
```

Demo mode uses a simulated robot in software and does not open a real COM port.

### Connect to the real robot

```powershell
python Software/src/cartesian_nc_control.py
```

On Windows, you can open [`Software/Run_Cartesian_GUI.bat`](Software/Run_Cartesian_GUI.bat)
after installing the dependencies. The launcher prefers running the Python source.

If an older EXE is present in `Software/dist/`, use the source command above to see the
R15 changes. The packaging entry point is [`Build_Cartesian_NC_GUI.ps1`](Software/Build_Cartesian_NC_GUI.ps1);
the script and spec still use the name `SCARA_Cartesian_NC_v5_R14`.

## Drawing workflow

1. Connect to COM and check **Cài đặt & Cơ khí** (Settings & Mechanics): link lengths,
   lead-screw lead, driver microstepping, transmission ratios, DIR polarity and switch angles.
2. Run **HOME / CALIB** to establish the reference and complete calibration. **HOME Nhanh**
   (Quick HOME) reuses calibrated travel ranges stored in firmware memory; if no calibration
   is available, the firmware performs a full HOME.
3. Use XYZ or **Jog & Tùy biến** (Jog & Customization) to position the pen. Capture and confirm
   **Z giấy** (paper Z). Use the wizard to calibrate switch angles or the paper plane when needed.
4. Open **Thư viện hình & chữ** (Shape & Text Library): select a pattern, enter text or import
   a vector file; set the size, center and rotation, then preview the path.
5. Dry-run above the paper before drawing. The planner checks the complete stroke,
   lifts the pen between strokes and lifts it again on completion.
6. Use **Đồ thị & Báo cáo** (Plots & Reports) to inspect telemetry or export data.
   Run HOME again when Stop/Esc, a connection/limit fault or releasing the arm invalidates the reference.

GUI guide: [Software/README.md](Software/README.md).
Analysis of recorded drawings: [drawing_audit_20261008.md](Software/reports/drawing_audit_20261008.md).

## Hardware and configuration

| Item | Reference configuration |
| --- | --- |
| Mechanism | Two revolute joints J1/J2 + a prismatic Z axis |
| Links and pen tip | L1 = L2 = 98 mm; default TCP offset is zero and must be measured for the assembled robot |
| Transmission | 2GT belts for the arm; Z lead screw with a configured lead of 2 mm/rev |
| Motors and drivers | 3 × NEMA 17, 3 × TB6600, STEP/DIR/ENA |
| Microcontroller | STM32F103C8, USB CDC, Zephyr 3.7 firmware |
| Switches | Six COM-NC switches, two ends per axis |
| Reference paper | 100 × 100 mm |

GUI configuration is stored in [Software/cartesian_nc_config.json](Software/cartesian_nc_config.json).
Axis arrays follow the order **[Z, J1, J2]**. The current configuration uses microstepping
`[16, 16, 8]`, J1/J2 transmission ratios `[3, 9]` and `coupling = 0.333333`;
check these against the actual drivers and mechanics.

Distinguish the **angular** coupling factor `k` from the **step-count** coupling factor `β`:

```text
A = q1
B = q2 + k*q1
β = k*F2/F1
```

`q2` is the relative elbow angle; `F1/F2` are the respective axis factors in pulses per degree.
The MATLAB snapshot contains data from a recorded run and can differ from the current GUI configuration.
Do not copy a coupling factor between configurations without checking its definition and values.

### Wiring and firmware

Wire the TB6600 inputs common-cathode: PUL−, DIR− and ENA− connect to a shared GND.

| Axis | PUL+ | DIR+ | ENA+ | DIR HIGH limit | DIR LOW limit |
| --- | --- | --- | --- | --- | --- |
| Z | PA0 | PA1 | PA2 | PA3, top | PA4, bottom |
| J1 | PA8 | PB15 | PB14 | PB0 | PB1 |
| J2 | PA6 | PA7 | PB13 | PB10 | PB11 |

Connect switch COM to GND and NC to a GPIO pulled up to 3.3 V. The normal state is LOW;
switch activation or an open wire reads HIGH. DIR polarity must match `positive_high`.

Prebuilt HEX/BIN files are available in [Firmware/ScaraCartesian/Hex](Firmware/ScaraCartesian/Hex);
flash at `0x08000000` using an ST-LINK. This branch retains the R9 identifier; the version
label alone does not establish that the binary matches every source change. The validation
run below did not rebuild MCU firmware or flash the robot.

Building from source requires a Zephyr workspace, SDK, CMake and Ninja.
The [build.ps1](Firmware/ScaraCartesian/tools/build.ps1) script accepts
`-ZephyrRoot`, `-SdkRoot`, `-Python`, `-BuildDirectory` and `-SkipExport`.
Use `-SkipExport` to build separately without overwriting the HEX/BIN files.
See the [firmware guide](Firmware/ScaraCartesian/README.md).

BOM/3D printing: [arm](Hardware/arm/BOM.md) · [frame](Hardware/frame/BOM.md) ·
[print list](Hardware/3D%20Print/README.md) · [EG118 pen holder](Hardware/Pen_Holder_EG118).

## Simulation

| Environment | Purpose and parameter source | Entry point |
| --- | --- | --- |
| MATLAB/Simulink | Replays DDA step counts from the GUI planner, converts kinematics and studies NC/watchdog behavior and assumed deviations; snapshot in `matlab/config/robot_calibration.json` | [scara_stepper.slx](Simulation/matlab/simulink/models/scara_stepper.slx) |
| Webots | Studies physics, PID and pen/paper contact using estimated values; uses `common/config/scara.json` | [scara_draw.wbt](Simulation/webots/worlds/scara_draw.wbt) |

The two environments currently use different parameter sets. Webots results and the older
comparison fixture do not validate the new stepper model or the accuracy of the real robot.

### MATLAB/Simulink

Set Current Folder to `Simulation/matlab`. Running the model requires MATLAB, Simulink,
and Python with access to the Software source in the same repository.
Robotics System Toolbox and Simscape Multibody are not required.

```matlab
run_scara                         % 3D interface and pattern selection
startup_scara;
out = scara_simulink('flower');    % Run the stepper model
validate_scara;                   % Validate MATLAB/Simulink
```

The model uses standard Simulink blocks. The old MATLAB PID/dynamics simulator
has been removed from this branch. See the [MATLAB guide](Simulation/matlab/README.md).

<div align="center">
<img src="Simulation/matlab/output/figures/assembly.png" alt="SCARA assembly model in MATLAB" height="280">
&nbsp;
<img src="Simulation/matlab/output/figures/tracking.png" alt="Simulated trajectory replay results" height="280">
<br><sub>Saved simulation result images included in the repository.</sub>
</div>

### Webots

Open `Simulation/webots/worlds/scara_draw.wbt` and press **Run**.
Edit [demo.json](Simulation/webots/config/demo.json), then Reset/Reload to change the pattern/mode.
The world is saved in R2025a format; the Webots runtime was not revalidated in this branch's
validation run. See the [Webots guide](Simulation/webots/README.md).

## Repository layout and source entry points

```text
.
├── Firmware/ScaraCartesian/
│   ├── src/                 Core, HOME, DDA, commands, telemetry, GPIO/timer
│   ├── tests/               C harness and artifact tests
│   ├── tools/               Build, verify, HEX/BIN checks
│   └── Hex/                 Prebuilt firmware and reports
├── Software/
│   ├── src/                 R15 GUI, model, planner, vector, 3D, calibration, logs
│   ├── tests/               Unit and GUI smoke tests using a simulated port
│   ├── packaging/           PyInstaller spec, still named R14
│   └── reports/             Test reports and log analysis
├── Simulation/
│   ├── matlab/              config/, src/, simulink/, tests/, tools/, output/
│   ├── webots/              World, PROTO, controller, tests
│   └── common/              Configuration/assets/CSV used by Webots
├── Hardware/                STL, BOM, assembly guides, EG118 pen holder
├── Doc/                     Firmware review and drawing deviations
├── media/                   Real robot photos and video
├── tools/                   Shared PowerShell tools
└── Verify_SCARA.ps1         Tests firmware first, then software
```

Main control flow:

```mermaid
flowchart LR
    GUI["R15 GUI: shapes, vector, XYZ"] --> PLAN["IK, preflight, background planner"]
    PLAN --> USB["USB CDC: PATH / SEG / GO"]
    USB --> FW["R9 firmware: 32-segment FIFO, DDA"]
    FW --> MOTOR["TB6600 → Z / J1 / J2"]
    NC["6 NC limit switches"] --> FW
    FW --> DATA["Telemetry → GUI, JSONL, plots"]
```

Start reading at [cartesian_nc_control.py](Software/src/cartesian_nc_control.py)
and [main.c](Firmware/ScaraCartesian/src/main.c).
Planning/USB run on the PC; firmware generates pulses using a timer and enforces protections.
Soft limits, NC switches, the 350 ms heartbeat, USB, GPIO and buffer underrun are handled in firmware.

## Testing

Run from the repository root; the firmware harness requires **Python and GCC**.
These commands use simulated ports/peripherals and do not control the real robot.

```powershell
# Firmware C + artifact tests + all software unit tests
powershell -NoProfile -ExecutionPolicy Bypass -File .\Verify_SCARA.ps1

# Include six GUI smoke scenarios
powershell -NoProfile -ExecutionPolicy Bypass -File .\Verify_SCARA.ps1 -Gui

# Offline Webots tests; Webots installation is not required
python -m unittest discover -s Simulation/webots/tests -v
```

Pass `-Python` and `-Gcc` to `Verify_SCARA.ps1` if the tools are not on PATH.
Build/cache/log output is generated in directories ignored by Git.

Validation results from **17 October 2026**, on source commit `4a8317a`:

| Scope | Result |
| --- | --- |
| C harness: core, protocol, GPIO/timer ISR | Three test programs passed |
| Firmware artifact-tool tests | 6/6 passed |
| Software unit tests | 78/78 passed |
| GUI smoke tests, including R15 and the calibration wizard | 6/6 scenarios passed |
| Offline Webots tests | 14/14 passed |
| MATLAB/Simulink | Not rerun; the [saved report](Simulation/matlab/output/validation/validation.json) is from an earlier validation |
| MCU/real robot and EXE packaging | Not revalidated in this run |

## Limits when interpreting results

- Emitted step counts do not establish that the motor/pen reached the intended position.
  Backlash, missed steps, link-length errors, switch angles and pen flex still require hardware measurements.
- Backlash compensation and two-/three-point calibration are supporting tools; software tests have
  not established the improvement in physical drawing quality on the R15 branch.
- MATLAB data is an offline replay based on a snapshot. Backlash/lag blocks are sensitivity
  scenarios rather than measured mechanical parameters; detailed driver electrical behavior is not modeled.
- SVG/DXF/G-code readers are implemented in [cartesian_nc_vector.py](Software/src/cartesian_nc_vector.py).
  Preview and preflight imported files; this is not a CAD/CAM system supporting every entity type.
- Some component READMEs retain descriptions from earlier versions. The change table, repository
  layout and validation results in this root README describe the branch state identified above.

Analysis documents: [firmware review](Doc/Danh_gia_Firmware_SCARA.md) ·
[simulation overview](Simulation/README.md).

## Credits and license

- Frame, arm, STL files, BOM and assembly images:
  [madl3x/x-scara](https://github.com/madl3x/x-scara), GPL-3.0.
  Source revision recorded in this project: `3c7c17e3dc0805b41a3cac6d4cd2778d585d7ed2`.
- Mesh provenance:
  [Simulation/common/assets/manifest.json](Simulation/common/assets/manifest.json).
- Pen-holder attribution:
  [Hardware/Pen_Holder_EG118/NOTICE.txt](Hardware/Pen_Holder_EG118/NOTICE.txt).

Repository license: **[GNU GPL v3.0](LICENSE)**.

<div align="center">
Author: <a href="https://github.com/TuanLinh05">@TuanLinh05</a> · HK261
</div>

---

# Tiếng Việt

Project môn **HPMR, học kỳ HK261**: điều khiển robot SCARA vẽ hình và chữ trên giấy.
Khung và tay dùng thiết kế [madl3x/x-scara](https://github.com/madl3x/x-scara);
phần phát triển trong project gồm firmware, GUI, mô phỏng và giá kẹp bút Deli EG118.

## Theo dõi branch này

Branch: **`codex/scara-r15-upgrade-20261017`**.
Bản nâng cấp ban đầu: [`4a8317a`](https://github.com/TuanLinh05/SCARA-Robot/commit/4a8317ae81dedec2ba4cc48bac335332e0eb9d4c), ngày **17/10/2026**.

| Thành phần | Trạng thái trên branch |
| --- | --- |
| GUI | R15, chạy từ mã nguồn Python; có chế độ demo |
| Firmware | Nhãn `SCARA_CART_NC_HOME_V5_R9`, protocol 5; có thêm thay đổi mã nguồn trên branch |
| MATLAB/Simulink | Mô hình stepper `scara_stepper.slx`, dùng snapshot hiệu chuẩn từ log |
| Webots | Mô hình vật lý nghiên cứu riêng; tham số chưa đồng bộ với hiệu chuẩn robot thực |
| Đóng gói EXE | Script/spec vẫn mang tên R14; chưa kiểm chứng lại bản đóng gói R15 |

**Đọc nhanh:** [Thay đổi](#các-thay-đổi-chính) · [Chạy GUI](#chạy-gui) ·
[Vận hành](#quy-trình-vẽ) · [Mô phỏng](#mô-phỏng) ·
[Kiểm thử](#kiểm-thử) · [Đọc mã](#cấu-trúc-và-điểm-vào-mã-nguồn).

## Các thay đổi chính

| Nhóm | Nội dung | Mã nguồn để theo dõi |
| --- | --- | --- |
| Giao diện R15 | Chuyển giữa khung 2D/3D, cửa sổ 3D riêng, xoay/dịch/zoom và xuất SVG; 6 thẻ thông số và 6 đèn công tắc | [3D](Software/src/cartesian_nc_3d.py), [layout](Software/src/cartesian_nc_layout.py), [view](Software/src/cartesian_nc_view.py) |
| Điều khiển | Step-jog, tư thế nhanh, điều chỉnh tốc độ và HOME nhanh sau khi đã hiệu chuẩn | [editor](Software/src/cartesian_nc_editor.py), [motion](Software/src/cartesian_nc_motion.py) |
| Vẽ và nhập file | Chữ nét đơn A–Z/0–9, mẫu SPIRAL/GRID/ROSE/POLYGON, SVG/DXF/G-code, sắp xếp nét và lập quỹ đạo nền | [font](Software/src/cartesian_nc_font.py), [vector](Software/src/cartesian_nc_vector.py), [planning](Software/src/cartesian_nc_planning.py) |
| Hiệu chuẩn | Ước lượng/bù độ rơ, hiệu chuẩn góc công tắc bằng 2 điểm, mặt phẳng giấy bằng 3 điểm | [calibration](Software/src/cartesian_nc_calibration.py), [model](Software/src/cartesian_nc_model.py) |
| Dữ liệu | Đồ thị tọa độ, góc khớp, tốc độ/xung/FIFO; đọc log JSONL và xuất CSV/SVG/Markdown | [analytics](Software/src/cartesian_nc_analytics.py), [audit](Software/src/cartesian_nc_audit.py) |
| Firmware | HOME đầy đủ/nhanh, xử lý ghép đai, bù độ rơ và telemetry; truyền đoạn theo ACK và dung lượng FIFO | [core](Firmware/ScaraCartesian/src/cart_core.c), [commands](Firmware/ScaraCartesian/src/cart_commands.c), [upload](Software/src/cartesian_nc_path.py) |
| Mô phỏng và dọn nguồn | MATLAB chuyển từ PID/động lực học sang tái hiện số xung của planner; cập nhật controller Webots; bỏ `Firmware/Ref` và các ví dụ/mô hình MATLAB cũ | [Simulation](Simulation/README.md), [MATLAB](Simulation/matlab/README.md) |

Khung 3D trong GUI là mô hình hiển thị giản lược, có tham chiếu kích thước STL.
Telemetry và vị trí hiển thị được suy ra từ số xung; robot thực chưa có encoder.

## Chạy GUI

Các lệnh PowerShell dưới đây chạy từ **thư mục gốc repository**, nơi có README này.
Môi trường đã kiểm thử: **Python 3.12**, Tkinter và `pyserial`.

### Chạy demo trước

```powershell
python -m pip install -r Software/requirements.txt
python Software/src/cartesian_nc_control.py --demo
```

Demo dùng robot giả lập trong phần mềm, không mở cổng COM thật.

### Kết nối robot thực

```powershell
python Software/src/cartesian_nc_control.py
```

Trên Windows có thể mở [`Software/Run_Cartesian_GUI.bat`](Software/Run_Cartesian_GUI.bat)
sau khi cài các phụ thuộc. Launcher ưu tiên chạy mã nguồn Python.

Nếu có EXE cũ trong `Software/dist/`, dùng lệnh chạy nguồn ở trên để xem các thay đổi
R15. Điểm vào đóng gói là [`Build_Cartesian_NC_GUI.ps1`](Software/Build_Cartesian_NC_GUI.ps1);
script và spec hiện vẫn dùng tên `SCARA_Cartesian_NC_v5_R14`.

## Quy trình vẽ

1. Kết nối COM, kiểm tra **Cài đặt & Cơ khí**: chiều dài khâu, bước vít me,
   vi bước driver, tỷ số truyền, chiều DIR và góc công tắc.
2. Chạy **HOME / CALIB** để lấy mốc và hiệu chuẩn đầy đủ. **HOME Nhanh** dùng lại
   hành trình đã hiệu chuẩn trong bộ nhớ firmware; khi chưa có dữ liệu, firmware
   thực hiện HOME đầy đủ.
3. Dùng XYZ hoặc **Jog & Tùy biến** để rà bút. Lấy và xác nhận **Z giấy**.
   Khi cần, dùng wizard để hiệu chuẩn góc công tắc hoặc mặt phẳng giấy.
4. Mở **Thư viện hình & chữ**: chọn mẫu, nhập chữ hoặc nạp vector; chỉnh kích thước,
   tâm, góc xoay và xem trước đường chạy.
5. Chạy thử trên không trước, sau đó vẽ thật. Bộ lập quỹ đạo kiểm tra toàn bộ nét,
   nâng bút khi chuyển nét và khi hoàn tất.
6. Dùng **Đồ thị & Báo cáo** để quan sát telemetry hoặc xuất dữ liệu.
   Sau Dừng/Esc, lỗi kết nối/biên hoặc nhả arm làm mất mốc, cần HOME lại.

Hướng dẫn giao diện: [Software/README.md](Software/README.md).
Phân tích nét vẽ đã ghi: [drawing_audit_20261008.md](Software/reports/drawing_audit_20261008.md).

## Phần cứng và cấu hình

| Mục | Cấu hình tham chiếu |
| --- | --- |
| Cơ cấu | 2 khớp quay J1/J2 + trục Z tịnh tiến |
| Khâu và đầu bút | L1 = L2 = 98 mm; lệch TCP mặc định bằng 0, cần đo theo cụm lắp |
| Truyền động | Đai 2GT cho tay; vít me Z, bước dẫn cấu hình 2 mm/vòng |
| Động cơ và driver | 3 × NEMA 17, 3 × TB6600, STEP/DIR/ENA |
| Vi điều khiển | STM32F103C8, USB CDC, firmware Zephyr 3.7 |
| Công tắc | 6 công tắc COM-NC, hai biên mỗi trục |
| Giấy tham chiếu | 100 × 100 mm |

Cấu hình GUI nằm ở [Software/cartesian_nc_config.json](Software/cartesian_nc_config.json).
Thứ tự các mảng trục là **[Z, J1, J2]**. Cấu hình hiện tại dùng vi bước
`[16, 16, 8]`, tỷ số truyền J1/J2 `[3, 9]` và `coupling = 0.333333`;
phải đối chiếu với driver và cơ khí thực tế.

Phân biệt hệ số ghép **góc** `k` và hệ số ghép **xung** `β`:

```text
A = q1
B = q2 + k*q1
β = k*F2/F1
```

`q2` là góc khuỷu tương đối; `F1/F2` là số xung trên độ của từng trục.
Giá trị snapshot MATLAB là dữ liệu của lần chạy đã ghi, có thể khác cấu hình GUI hiện tại.
Không chép mặc định một hệ số giữa các bộ cấu hình.

### Đấu dây và firmware

TB6600 nối chung âm: PUL−, DIR−, ENA− về GND chung.

| Trục | PUL+ | DIR+ | ENA+ | Biên DIR HIGH | Biên DIR LOW |
| --- | --- | --- | --- | --- | --- |
| Z | PA0 | PA1 | PA2 | PA3, trên | PA4, dưới |
| J1 | PA8 | PB15 | PB14 | PB0 | PB1 |
| J2 | PA6 | PA7 | PB13 | PB10 | PB11 |

COM công tắc về GND, NC về GPIO kéo lên 3,3 V. Bình thường LOW; chạm công tắc
hoặc hở dây là HIGH. Chiều DIR phải khớp cấu hình `positive_high`.

Bộ HEX/BIN có sẵn trong [Firmware/ScaraCartesian/Hex](Firmware/ScaraCartesian/Hex),
nạp tại `0x08000000` qua ST-LINK. Nhãn R9 được giữ lại trên branch; chỉ nhãn phiên bản
không xác nhận binary khớp mọi thay đổi mã nguồn. Lần kiểm thử bên dưới chưa build lại
firmware cho MCU hoặc nạp vào robot.

Build nguồn cần workspace Zephyr, SDK, CMake và Ninja.
Script [build.ps1](Firmware/ScaraCartesian/tools/build.ps1) nhận
`-ZephyrRoot`, `-SdkRoot`, `-Python`, `-BuildDirectory`, `-SkipExport`.
Dùng `-SkipExport` khi muốn build riêng mà chưa ghi đè bộ HEX/BIN.
Xem thêm [hướng dẫn firmware](Firmware/ScaraCartesian/README.md).

BOM/in 3D: [tay](Hardware/arm/BOM.md) · [khung](Hardware/frame/BOM.md) ·
[danh sách in](Hardware/3D%20Print/README.md) · [giá kẹp EG118](Hardware/Pen_Holder_EG118).

## Mô phỏng

| Môi trường | Vai trò và nguồn tham số | Điểm mở |
| --- | --- | --- |
| MATLAB/Simulink | Tái hiện số xung DDA từ planner GUI, quy đổi động học, khảo sát NC/watchdog và sai lệch giả định; snapshot ở `matlab/config/robot_calibration.json` | [scara_stepper.slx](Simulation/matlab/simulink/models/scara_stepper.slx) |
| Webots | Nghiên cứu vật lý, PID và tiếp xúc bút/giấy với các giá trị ước lượng; dùng `common/config/scara.json` | [scara_draw.wbt](Simulation/webots/worlds/scara_draw.wbt) |

Hai môi trường hiện có bộ tham số khác nhau. Kết quả Webots và fixture đối chiếu cũ
không xác nhận mô hình stepper mới hoặc độ chính xác robot thực.

### MATLAB/Simulink

Đặt Current Folder ở `Simulation/matlab`. Cần MATLAB, Simulink để chạy mô hình,
và Python có thể truy cập mã nguồn Software trong cùng repository.
Không cần Robotics System Toolbox hay Simscape Multibody.

```matlab
run_scara                         % Giao diện 3D và chọn mẫu
startup_scara;
out = scara_simulink('flower');    % Chạy mô hình stepper
validate_scara;                   % Kiểm thử MATLAB/Simulink
```

Mô hình dùng các khối Simulink chuẩn. Không còn bộ mô phỏng PID/động lực học MATLAB
cũ trong branch này. Xem [hướng dẫn MATLAB](Simulation/matlab/README.md).

<div align="center">
<img src="Simulation/matlab/output/figures/assembly.png" alt="Mô hình lắp ráp SCARA trong MATLAB" height="280">
&nbsp;
<img src="Simulation/matlab/output/figures/tracking.png" alt="Kết quả tái hiện quỹ đạo trong mô phỏng" height="280">
<br><sub>Ảnh kết quả mô phỏng đã lưu trong repository.</sub>
</div>

### Webots

Mở `Simulation/webots/worlds/scara_draw.wbt`, nhấn **Run**.
Sửa [demo.json](Simulation/webots/config/demo.json), rồi Reset/Reload để đổi mẫu/chế độ.
World lưu theo định dạng R2025a; runtime Webots chưa được kiểm chứng lại trong lần
kiểm thử branch này. Xem [hướng dẫn Webots](Simulation/webots/README.md).

## Cấu trúc và điểm vào mã nguồn

```text
.
├── Firmware/ScaraCartesian/
│   ├── src/                 Core, HOME, DDA, lệnh, telemetry, GPIO/timer
│   ├── tests/               Harness C và kiểm thử artifact
│   ├── tools/               Build, verify, kiểm tra HEX/BIN
│   └── Hex/                 Bộ firmware và báo cáo có sẵn
├── Software/
│   ├── src/                 GUI R15, mô hình, planner, vector, 3D, hiệu chuẩn, log
│   ├── tests/               Unit test và GUI smoke test bằng cổng giả lập
│   ├── packaging/           Spec PyInstaller, hiện còn tên R14
│   └── reports/             Báo cáo kiểm thử và phân tích log
├── Simulation/
│   ├── matlab/              config/, src/, simulink/, tests/, tools/, output/
│   ├── webots/              World, PROTO, controller, tests
│   └── common/              Cấu hình/asset/CSV phục vụ Webots
├── Hardware/                STL, BOM, hướng dẫn lắp, giá kẹp EG118
├── Doc/                     Đánh giá firmware và sai lệch nét vẽ
├── media/                   Ảnh và video robot thực
├── tools/                   Công cụ PowerShell dùng chung
└── Verify_SCARA.ps1         Kiểm thử firmware trước, rồi software
```

Luồng điều khiển chính:

```mermaid
flowchart LR
    GUI["GUI R15: hình, vector, XYZ"] --> PLAN["IK, preflight, planner nền"]
    PLAN --> USB["USB CDC: PATH / SEG / GO"]
    USB --> FW["Firmware R9: FIFO 32 đoạn, DDA"]
    FW --> MOTOR["TB6600 → Z / J1 / J2"]
    NC["6 công tắc NC"] --> FW
    FW --> DATA["Telemetry → GUI, JSONL, đồ thị"]
```

Bắt đầu đọc từ [cartesian_nc_control.py](Software/src/cartesian_nc_control.py)
và [main.c](Firmware/ScaraCartesian/src/main.c).
Planner/USB chạy phía PC; firmware phát xung bằng timer và kiểm tra bảo vệ.
Giới hạn mềm, NC, heartbeat 350 ms, USB, GPIO và cạn bộ đệm được xử lý trong firmware.

## Kiểm thử

Chạy từ thư mục gốc; cần **Python và GCC** cho harness firmware.
Các lệnh này dùng cổng/ngoại vi giả lập, không điều khiển robot thực.

```powershell
# Firmware C + artifact tests + toàn bộ unit test software
powershell -NoProfile -ExecutionPolicy Bypass -File .\Verify_SCARA.ps1

# Thêm 6 kịch bản GUI smoke test
powershell -NoProfile -ExecutionPolicy Bypass -File .\Verify_SCARA.ps1 -Gui

# Webots offline, không cần cài Webots
python -m unittest discover -s Simulation/webots/tests -v
```

Có thể truyền `-Python` và `-Gcc` cho `Verify_SCARA.ps1` khi công cụ không có trên PATH.
Build/cache/log được tạo trong các thư mục đã bỏ qua bằng Git.

Kết quả kiểm tra **17/10/2026**, trên mã nguồn commit `4a8317a`:

| Phạm vi | Kết quả |
| --- | --- |
| Harness C: core, protocol, GPIO/timer ISR | 3 chương trình kiểm thử đạt |
| Kiểm thử công cụ firmware artifact | 6/6 đạt |
| Unit test software | 78/78 đạt |
| GUI smoke test, gồm R15 và wizard hiệu chuẩn | 6/6 kịch bản đạt |
| Webots offline | 14/14 đạt |
| MATLAB/Simulink | Chưa chạy lại; [báo cáo lưu sẵn](Simulation/matlab/output/validation/validation.json) thuộc lần kiểm tra trước |
| MCU/robot thực và đóng gói EXE | Chưa kiểm chứng lại trong lần này |

## Giới hạn cần hiểu khi đọc kết quả

- Số xung phát ra không chứng minh motor/bút đã đến đúng vị trí. Độ rơ, trượt bước,
  sai chiều dài khâu, góc công tắc và độ võng bút vẫn cần đo trên phần cứng.
- Bù độ rơ và hiệu chuẩn 2/3 điểm là các công cụ hỗ trợ; kiểm thử phần mềm chưa xác nhận
  mức cải thiện nét vẽ thực tế của branch R15.
- Dữ liệu MATLAB là tái hiện offline theo snapshot. Khối độ rơ/độ trễ là kịch bản
  khảo sát, không phải thông số cơ khí đã đo; không mô phỏng chi tiết điện driver.
- Các bộ đọc SVG/DXF/G-code nằm trong [cartesian_nc_vector.py](Software/src/cartesian_nc_vector.py).
  Cần xem trước và preflight file nhập; không coi đây là bộ CAD/CAM hỗ trợ mọi thực thể.
- Một số README thành phần còn giữ mô tả phiên bản trước. Bảng thay đổi, cấu trúc
  và kết quả kiểm thử ở README gốc này mô tả trạng thái branch đã nêu.

Tài liệu phân tích: [đánh giá firmware](Doc/Danh_gia_Firmware_SCARA.md) ·
[tổng quan mô phỏng](Simulation/README.md).

## Nguồn và giấy phép

- Khung, tay, STL, BOM và ảnh lắp ráp:
  [madl3x/x-scara](https://github.com/madl3x/x-scara), GPL-3.0.
  Phiên bản nguồn được ghi trong project: `3c7c17e3dc0805b41a3cac6d4cd2778d585d7ed2`.
- Nguồn mesh:
  [Simulation/common/assets/manifest.json](Simulation/common/assets/manifest.json).
- Ghi nhận nguồn giá kẹp bút:
  [Hardware/Pen_Holder_EG118/NOTICE.txt](Hardware/Pen_Holder_EG118/NOTICE.txt).

Giấy phép repository: **[GNU GPL v3.0](LICENSE)**.

<div align="center">
Tác giả: <a href="https://github.com/TuanLinh05">@TuanLinh05</a> · HK261
</div>
