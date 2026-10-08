# Danh sách file in 3D — X-SCARA

## Tổng số lượng

- **Frame:** 14 chi tiết (8 loại STL).
- **Arm:** 23 chi tiết (17 loại STL).
- **Test part:** 1 chi tiết để in thử trước khi in bộ chính.
- **Tổng:** 38 bản in, gồm 37 chi tiết lắp ráp và 1 mẫu thử.

Các thư mục màu/số lượng giữ theo cách phân loại gốc. `QuantityN` nghĩa là in **N bản của mỗi file STL trong thư mục đó**; ví dụ `Orange_Quantity2` có 4 file, mỗi file in 2 bản (8 chi tiết tổng).

## Frame — 14 chi tiết

| STL | Số lượng |
| --- | ---: |
| Frame_Plate_TipConnector | 2 |
| Frame_Plate_Side_Left | 2 |
| Frame_Plate_Side_Right | 2 |
| Frame_Plate_Side_RodClamp | 4 |
| Frame_Z_BearingPlate | 1 |
| Frame_Z_BearingHolder | 1 |
| Frame_Z_MotorPlate | 1 |
| Frame_Z_MotorConnector | 1 |
| **Cộng** | **14** |

## Arm — 23 chi tiết

| STL | Số lượng |
| --- | ---: |
| Arm_Base_MotorPlate | 2 |
| Arm_Base_ZPlate_Mid_NutHolder | 1 |
| Arm_Base_ZHub_Top | 1 |
| Arm_Base_ZHub_Bottom | 1 |
| Arm_Shoulder_Plate_Top | 1 |
| Arm_Shoulder_Plate_Bottom | 1 |
| Arm_SideWall_Left (Arm) | 2 |
| Arm_SideWall_Right (Arm) | 2 |
| Arm_Plates_Cover | 2 |
| Arm_Shoulder_Connector | 2 |
| Arm_Elbow_Plate_Top | 1 |
| Arm_Elbow_Plate_Bottom | 1 |
| Arm_Elbow_RodHub | 1 |
| Arm_EMount_Base_ABL | 1 |
| Arm_EMount_Base_Left | 1 |
| Arm_EMount_Base_Right | 1 |
| Arm_Endstop | 2 |
| **Cộng** | **23** |

`Arm_SideWall_Left` và `Arm_SideWall_Right` cần 2 bản mỗi loại: mỗi bên một bản cho Shoulder và một bản cho Elbow.

## In thử

In **1** bản `Test_hardware_fit_1.0.4.stl` trước để kiểm tra độ khớp với thanh M8, vít/ê-cu M3 và vít M5; sau đó mới in Frame và Arm.

## Nguồn số lượng

- [X-SCARA Frame README trên GitHub](https://github.com/madl3x/x-scara/blob/master/hardware/frame/README.md)
- `../frame/BOM.md` và `../arm/BOM.md` trong workspace.
- `../Hướng dẫn in test part và số lượng.docx` trong workspace.

## README theo từng folder

Mỗi folder Frame, Arm, Test Part và từng folder số lượng đều có README.md riêng liệt kê chính xác file STL và số bản cần in.
