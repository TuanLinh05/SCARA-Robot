# Đánh giá firmware SCARA và nguyên nhân nét vẽ bị lệch/cong

> Ngày: 08/10/2026
> Firmware: `SCARA_CART_NC_HOME_V5_R9` (`Firmware/ScaraCartesian`) · GUI: `SCARA_GUI_V5_R13/R14`
> Dữ liệu: toàn bộ mã nguồn firmware, 7 file log phần cứng thật trong
> `Software/dist/SCARA_Cartesian_NC_v5_R14/logs` (6 lần HOME, 58 nét vẽ, 3701 đoạn),
> BOM cơ khí `Hardware/arm/BOM.md`.

---

## Tóm tắt

- Firmware **chạy ổn định** trên phần cứng: HOME lặp lại rất tốt (J1 ±2 xung, J2 ±7 xung), không lần nào cạn bộ đệm hay mất mốc khi đang vẽ, ngắt timer không bị trễ khi vẽ.
- **Nguyên nhân chính khiến hình bị lệch và méo nằm ở bước HOME.** Firmware giả định hai công tắc của mỗi khớp cách nhau đúng 180° rồi suy ra số xung/độ. Log thật cho thấy khoảng cách thật chỉ khoảng **177,5°** ở cả J1 và J2. Thang góc vì vậy sai khoảng 1,4%: đầu bút lệch **0,8–6 mm**, đường tròn méo **0,2–0,9 mm** (tùy phần 2,5° bị thiếu nằm ở đầu nào).
- **Hệ số ghép khuỷu k được đo lại ở mỗi lần HOME** chỉ dựa trên khoảng 64 xung, nên dao động ±1,6%. Hình dịch thêm khoảng 0,4 mm, và mỗi lần HOME dịch một kiểu.
- Cấu hình có **hai giá trị không khớp phần cứng** nhưng đang bị che vì firmware đo lại:
  - Vi bước J1: cấu hình ghi x8, thực tế là 3200 xung/vòng động cơ.
  - k: cấu hình ghi 4/3, thực tế là **1/3** theo cả BOM lẫn log.
- Với nét thẳng ngắn (46 mm), các sai số trên chỉ làm cong dưới 0,07 mm. Nếu nét thẳng **gãy hoặc cong thấy rõ bằng mắt**, hãy nghi **độ rơ** trước (đai, giá bút).
- Đề xuất cho firmware:
  1. Lấy thang góc từ tỷ số truyền; công tắc chỉ dùng làm gốc và để kiểm tra.
  2. Dùng k = 1/3 cố định; phép đo chỉ để bảo vệ.
  3. Bù độ rơ.
  4. Thêm telemetry phục vụ hiệu chuẩn.

  Song song đó cần đo góc thật của công tắc, L1/L2 và vị trí ngòi bút.

---

## 1. Firmware hoạt động thế nào

Phần này tóm tắt những gì cần biết để hiểu các mục sau.

### 1.1 Đường đi của một xung

```
USB CDC (lệnh dạng chữ) ──► main loop (mỗi 2 ms) ──► ct_command() / ct_service()
                                                         │ đặt đích, profile tốc độ
TIM2 1 MHz ──► pulse_tick() (ngắt) ──► đọc 6 công tắc ──► kiểm tra an toàn
                                  └──► DDA chọn trục phát xung ──► chân PUL / DIR
```

- Mỗi xung ứng với **2 lần ngắt**: lần ở cạnh lên phát xung, lần ở cạnh xuống hạ xung. Nếu sang segment mới, DIR được đổi ở cạnh xuống, sau đó còn ít nhất 100 µs trước xung kế tiếp.
- **DDA** (giống Bresenham): trục cần đi nhiều xung nhất ("trục trội") phát một xung mỗi tick, các trục khác phát theo tỷ lệ. Mọi trục bắt đầu và kết thúc cùng lúc, nên giữa hai điểm robot đi **thẳng trong không gian khớp**, không phải thẳng trên giấy.
- GUI tính động học ngược (IK) và chia mỗi nét thành các đoạn khoảng 0,75 mm, rồi gửi `PATH` / `SEG` / `GO`. Firmware **chỉ biết tọa độ động cơ** (số xung).

### 1.2 Các đại lượng quan trọng

| Ký hiệu trong mã | Ý nghĩa |
|---|---|
| `factor` (f) | số xung/độ × 1024 |
| `range` | số xung giữa hai công tắc của một khớp, đo khi HOME |
| `low_md`, `high_md` | góc của hai công tắc **do người dùng nhập** (lệnh `SPAN`, đơn vị mili-độ) |
| `origin` | số xung tại góc 0 |
| `coupling_ppm` (k) | ghép khuỷu: góc khuỷu = B − k·A (đơn vị độ) |
| `beta` (β) | k quy ra xung: β = k·f2/f1 = số xung B cần phát để khuỷu đứng yên khi A đi 1 xung |

**Vì sao có ghép khuỷu (X-SCARA, theo BOM):**
- Tất cả các tầng đều là đai GT2 với puly 20T→60T.
- J1: động cơ A (20T) → puly 60T ở vai, tỷ số 3:1.
- J2: động cơ B (20T) → puly 60T nằm trên trục vai (dính liền với một puly 20T) → puly 60T ở khuỷu, tỷ số 3 × 3 = 9:1.
- Khi B đứng yên mà vai quay A độ: puly 20T trên trục vai đứng yên so với đế, nên khuỷu quay tương đối −A·20/60 = **−A/3**. Vậy k = 1/3.

### 1.3 Trình tự HOME (rút gọn)

1. **Z**: quét hai biên, kết thúc ở biên trên.
2. **J2_PRE**: quét hai biên J2 (hai lần đo n1 và n2 phải lệch nhau ≤ 1%), rồi về giữa.
3. **Dò ghép**: J1 đi 128 xung, quét lại một biên J2, đo β = ΔB/ΔA.
4. **J1**: quét hai biên J1; J2 chạy bù theo β để khuỷu không đụng biên.
5. J1 về góc đỗ, **J2_FINAL**: quét J2 lần nữa để lấy gốc J2.
6. **Đỗ**: Z lùi cách biên trên 3 mm.

Thang góc được tính ở [cart_core.c](../Firmware/ScaraCartesian/src/cart_core.c), dòng 371–374:

```
factor = range / (high_md − low_md)    // ví dụ: 4733 xung / 180° = 26,29 xung/°
origin = vị trí công tắc thấp − low_md × factor
```

→ **Thang góc phụ thuộc hoàn toàn vào góc công tắc do người dùng nhập (±90°).** Cách này có từ R7: nó giúp robot không đâm biên khi cấu hình vi bước sai. Đổi lại, độ chính xác phụ thuộc vào một con số chưa ai đo.

---

## 2. Đã kiểm tra những gì

| Hạng mục | Cách làm | Kết quả |
|---|---|---|
| Mã nguồn | Đọc toàn bộ `src/`, `app.overlay`, `prj.conf` | Mục 8 |
| Test có sẵn | Build bằng gcc (MSYS2) như `tools/verify.ps1`, chạy `test_cart_core` và `test_motor_io` | Pass cả hai |
| Hành vi PATH/MOVE | Viết test riêng chạy trên lõi `cart_core.c` | Mục 8 |
| Log phần cứng thật | Trích 7 file log ngày 07–08/10 | Mục 3 |
| Cơ khí | Đối chiếu `Hardware/arm/BOM.md` | Tỷ số 3:1 và 9:1, k = 1/3 |
| Ảnh hưởng của từng sai số | Mô phỏng động học L1 = L2 = 98 mm tại đúng vị trí vẽ trong log | Mục 4 |

---

## 3. Kết quả từ log phần cứng thật

### 3.1 Sáu lần HOME

| Phiên | range Z | range J1 | range J2 | ΔB khi dò (xung) | k firmware tính |
|---|---:|---:|---:|---:|---:|
| 07/10 17:41 | 87419 | 4732 | 7104 | 64 | 0,3331 |
| 07/10 18:47 | 87364 | 4732 | 7102 | 64 | 0,3331 |
| 07/10 20:07 | 87360 | 4732 | 7107 | 64 | 0,3329 |
| 07/10 20:41 | 87365 | 4735 | 7103 | 65 | 0,3385 |
| 08/10 10:48 | 87277 | 4733 | 7094 | 65 | 0,3388 |
| 08/10 13:06 | 87230 | 4733 | 7106 | 63 | 0,3278 |

### 3.2 Diễn giải

1. **Công tắc và HOME lặp lại rất tốt.** J1 đo được 4732–4735 xung, J2 7094–7107. Trong cùng một lần HOME, n1 và n2 lệch nhau ≤ 1 xung. Không có dấu hiệu mất bước khi quét.
2. **Vi bước J1 trong cấu hình không khớp phần cứng.**
   - GUI gửi `GEOM … 13653 …`, tức 13,33 xung/° (x8, 3:1).
   - Đo được 4733 xung cho "180°", tức 26,3 xung/°, gần gấp đôi.
   - Tỷ số J1 là 3:1 cố định theo BOM, nên driver J1 thực tế đang chạy **3200 xung/vòng động cơ**: hoặc DIP x16 với động cơ 1,8°, hoặc x8 với động cơ 0,9°. **Cần kiểm tra lại.**
3. **Hai công tắc của mỗi khớp chỉ cách nhau khoảng 177,5°**, không phải 180°:
   - J1: 4733 / 26,667 = **177,5°**
   - J2: 7104 / 40 = **177,6°**

   Cả hai khớp đều thiếu khoảng 2,5°. Nguyên nhân có thể là cần gạt công tắc tác động sớm, hoặc công tắc không được lắp đúng ±90°. Chỉ đo trực tiếp trên robot mới biết phần thiếu nằm ở đầu nào.
4. **k thật = 1/3.** BOM và log khớp nhau: β danh định = (1/3) · 40 / 26,667 = 0,5, và ba trong sáu lần đo cho đúng 64/128 = 0,5. Cấu hình đang ghi 1,333333 nhưng không gây hại, vì firmware đo lại và GUI dùng giá trị đo.
5. **Phép dò ghép quá ngắn.** 128 xung J1 chỉ cho khoảng 64 xung J2, nên sai 1 xung đã là 1,6%. Vì vậy k dao động 0,328–0,339 giữa các lần HOME.
6. **Khi vẽ, phần phát xung không có vấn đề.**
   - 58 nét gồm 3701 đoạn; mỗi đoạn trung vị 17 xung; tốc độ đỉnh trung vị 213 xung/s, lớn nhất 276 xung/s.
   - Không lần nào cạn bộ đệm.
   - `timer_late` chỉ tăng 0–1 lần trong toàn bộ thời gian vẽ.
   - Sai lệch do GUI chia đoạn và làm tròn xung ≤ 0,14 mm.
7. **`timer_late` = 266 ở phiên chạy R9.** Toàn bộ 266 lần đều xảy ra ở bước HOME trục Z, chạy 4800 xung/s (chu kỳ 208 µs). Các xung này bị trễ chứ không mất, nên Z vẫn lặp lại tốt. Tuy vậy đây là dấu hiệu ngắt bị khóa quá lâu (mục 8, dòng 4).
8. **Mỗi lần kết nối có đúng một dòng JSON bị cắt**, ghi nhận là `rx_rejected` ở giây 0,2. Firmware gửi status liên tục cả khi GUI chưa mở cổng; bộ đệm USB đầy thì byte bị bỏ. GUI loại dòng hỏng nên không gây hại, nhưng nó xác nhận vấn đề chưa kiểm tra DTR (mục 8, dòng 5).
9. **Không có lệnh STOP nào cắt ngang lúc robot đang chạy.** Có 20 lệnh STOP (USER 5, FOCUS 8, DISCONNECT 6, CLOSE 1), tất cả gửi lúc robot đứng yên. Không có lệnh nào bị từ chối, và robot chưa lần nào mất mốc khi đang chạy.

---

## 4. Vì sao nét vẽ bị lệch và cong

### 4.1 Cơ chế

Firmware tin rằng công tắc nằm ở ±90°. Nếu thực tế chúng chỉ cách nhau 177,5°, thì mỗi "độ" firmware tính ra chỉ bằng 0,986° thật.

Ví dụ tại tâm vùng vẽ trong log (x = 0, y = 165 mm), IK cho góc vai −32,67° và góc khuỷu 65,33°:

| Giả thiết về công tắc | Góc thật (vai, khuỷu) | Đầu bút lệch |
|---|---|---|
| Thiếu đều hai đầu (công tắc ở ±88,75°) | −32,21°, 64,46° | 0,8 mm |
| Thiếu hết ở đầu cao (công tắc thấp đúng −90°) | −33,48°, 63,26° | 5,7 mm |

Sai số góc thay đổi theo tư thế, nên sai số đầu bút cũng thay đổi theo vị trí. Kết quả là hình **vừa bị dời chỗ, vừa bị méo**.

### 4.2 Bảng độ nhạy

Điều kiện mô phỏng:
- L1 = L2 = 98 mm, hình 46 mm đặt tại tâm (0, 165) giống log.
- Lệnh khớp được tính bằng IK lý tưởng; robot "thật" chỉ mang đúng một sai số trong mỗi trường hợp.

| Nguồn sai số | Nét tiếp tuyến cong | Nét hướng kính cong | Tròn méo (Rmax − Rmin) | Lệch vị trí lớn nhất |
|---|---:|---:|---:|---:|
| Khoảng 177,5° bị coi là 180°, thiếu đều hai đầu | 0,00 mm | 0,01 mm | 0,38 mm | 1,4 mm |
| như trên, thiếu hết ở đầu cao | 0,04 mm | 0,01 mm | 0,94 mm | 6,3 mm |
| như trên, thiếu hết ở đầu thấp | 0,03 mm | 0,01 mm | 0,24 mm | 6,3 mm |
| k dao động theo lần HOME (±1 xung) | 0,02 mm | 0,04 mm | 0,23 mm | 0,4 mm |
| Độ rơ J1 0,3° | 0 | 0 | 0,67 mm | 0,5 mm |
| Độ rơ J2 0,3° | **0,27 mm** (gãy giữa nét) | 0 | 0,44 mm | 0,3 mm |
| Gốc 0 của J1 lệch 1° | 0 | 0 | 0 | 3,3 mm (cả hình xoay) |
| Gốc 0 của J2 lệch 1° | 0,03 mm | 0 | 0,47 mm | 1,7 mm |
| L1 (hoặc L2) thật là 99 mm thay vì 98 | 0,00 mm | 0,06 mm | 0,27 mm | 1,0 mm |

Độ phân giải không phải nguyên nhân:
- 1 xung J1 = 0,0375°, tức 0,11 mm ở bán kính 165 mm.
- 1 xung J2 = 0,025°, tức 0,04 mm.

### 4.3 Cách đọc bảng

- **Cả hình bị dời chỗ:** chủ yếu do thang góc và gốc 0 của khớp, cỡ vài mm.
- **Đường tròn bị méo:** nhạy nhất với thang góc, gốc 0 của khuỷu, chiều dài khâu và độ rơ. Mỗi nguồn gây 0,2–0,9 mm; cộng dồn có thể lên 1–2 mm.
- **Nét thẳng ngắn:** các sai số tham số chỉ làm nét 46 mm cong dưới 0,07 mm. Độ cong tăng theo bình phương chiều dài, nên nét 90 mm cong khoảng gấp 4 lần.
  - Nếu nét 46 mm cong hoặc gãy thấy rõ bằng mắt, hãy nghi **độ rơ** trước. Khi vẽ nét tiếp tuyến, khuỷu đổi chiều ngay giữa nét.
  - Cũng cần kiểm tra **giá bút bị rơ** hoặc **cánh tay bị võng**.
- **Hai sai số chỉ làm hình to/nhỏ hoặc xoay, không làm méo:**
  - Gốc 0 của J1 lệch: cả hình xoay quanh trục vai.
  - L1 và L2 cùng sai một tỷ lệ: hình phóng to hoặc thu nhỏ.

---

## 5. Đề xuất sửa firmware

### Bước 0: sửa cấu hình cho khớp phần cứng (làm trước)

- Kiểm tra DIP vi bước của driver J1, và động cơ J1 là loại 1,8° hay 0,9°. Nếu là 3200 xung/vòng với động cơ 1,8° thì đặt `microsteps` = `[16, 16, 8]`.
- Đặt `coupling` = `0.333333`.

Hiện tại hai giá trị sai này không gây hại vì firmware đo lại. Nhưng mọi phép kiểm tra "so với danh định" ở dưới đều cần chúng đúng.

### Đề xuất 1: thang góc lấy từ tỷ số truyền, công tắc chỉ dùng làm gốc và để kiểm tra

Thay đoạn ở `cart_core.c`, dòng 369–374:

```c
unsigned a = c->selected;
c->range[a] = c->cal[a].pulses;
uint64_t f = c->nominal[a];                       /* vi bước × tỷ số răng: chính xác */
if (a == CT_Z && c->z_span_um)                    /* Z giữ như cũ */
    f = (uint64_t)c->range[a] * 1024000 / c->z_span_um;
if (a != CT_Z) {
    unsigned j = a - 1;
    int32_t typed = c->high_md[j] - c->low_md[j];
    int32_t span  = (int32_t)((uint64_t)c->range[a] * 1024000U / f);   /* m° thật */
    if (abs(span - typed) > typed / 20)           /* lệch > 5%: sai DIP / tỷ số */
        return init_fail(c, "scale_mismatch");
    c->span_md[j] = span;                         /* trường mới, đưa vào status */
    c->high_md[j] = c->low_md[j] + span;          /* giới hạn mềm theo công tắc thật */
}
c->factor[a] = (uint32_t)f;
```

Kết quả:
- **Thang góc đúng tuyệt đối.** Sai số do làm tròn ×1024 dưới 0,002%.
- **Sai DIP hoặc sai tỷ số bị phát hiện:** HOME dừng với `scale_mismatch`, thay vì âm thầm bỏ qua như J1 hiện nay.
- **Giá trị thấp trong `SPAN` trở thành góc thật của công tắc thấp**, cần đo (mục 6). Giá trị cao chỉ còn dùng để kiểm tra.
- **GUI không cần đổi IK** vì vẫn đọc `factor`. Tuy vậy GUI nên đọc `span_md` để đặt giới hạn khớp khớp với công tắc thật.

### Đề xuất 2: k cố định theo cơ khí, phép dò chỉ để bảo vệ

Ở bước `HS_COUPLE_SCAN` (`cart_core.c`, khoảng dòng 420–436), sau khi đã tính `measured`:

```c
int64_t nominal = rounded((int64_t)c->coupling_ppm * c->nominal[2] * CT_Q,
                          (int64_t)1000000 * c->nominal[1]);   /* β = k·f2/f1 */
c->beta_measured = measured;                                   /* trường mới, cho telemetry */
if (llabs(measured - nominal) > llabs(nominal) / 20)           /* lệch > 5% */
    return init_fail(c, "coupling_mismatch");
c->beta = nominal;                                             /* dùng hằng số cơ khí */
c->coupling_ready = true;
```

Đồng thời bỏ khối tính lại `coupling_ppm` từ phép dò ở dòng 375–380, để k giữ đúng giá trị cấu hình.

- **Vẫn giữ phép dò**, vì nó bảo vệ robot trước lượt quét J1 dài; đây là lý do R7 thêm nó vào. Nếu cấu hình sai thì β đo được sẽ lệch xa danh định, và HOME dừng trước khi J2 đâm biên.
- **Hiệu quả:** k không còn dao động theo từng lần HOME, nên mất đi khoảng 0,4 mm dịch hình ngẫu nhiên.
- **Nếu vẫn muốn đo k:** tính β từ hai lần quét J2 sẵn có (`HS_J2_PRE` và `HS_J2_FINAL`). Quãng J1 giữa hai lần này thường dài hàng nghìn xung, so với 128 xung hiện nay, nên chính xác hơn hàng chục lần. Nếu robot khởi động sát tư thế đỗ thì hai vị trí quá gần nhau; khi đó thêm một lần quét J2 tại J1 = tư thế đỗ ± 45°.

### Đề xuất 3: bù độ rơ (làm sau khi đo được độ rơ)

Thêm lệnh `BACKLASH <session> <xung A> <xung B>`. Firmware nhớ chiều quay gần nhất của từng động cơ. Khi một segment làm động cơ đó đổi chiều, firmware phát thêm N xung bù trước, **không cộng vào `pos`**:

```c
/* trạng thái mới */
uint16_t backlash[CT_AXES], comp_left[CT_AXES]; int last_dir[CT_AXES];

/* khi nạp đoạn mới (raw_move / ct_path_advance) */
for (unsigned a = CT_J1; a <= CT_J2; a++) {
    if (c->direction[a] && c->last_dir[a] && c->direction[a] != c->last_dir[a])
        c->comp_left[a] = c->backlash[a];
    if (c->direction[a]) c->last_dir[a] = c->direction[a];
}
/* pulse_tick(), cạnh lên: nếu còn comp_left thì chỉ phát xung bù
 * ở tốc độ thấp cố định (ví dụ 200 xung/s), KHÔNG gọi ct_count(),
 * xong mới chạy DDA của đoạn. */
```

- **Ở khuỷu, xét chiều của động cơ B** chứ không phải chiều góc khuỷu, vì độ rơ nằm trong truyền động của B.
- **Trong PATH, khớp thường đổi chiều đúng lúc vận tốc khớp gần bằng 0**, nên chèn vài xung chỉ gây khựng rất ngắn.
- **Sau HOME phải ghi nhận lại chiều quay**, vì đó là chiều của lần đi vào tư thế đỗ.

### Đề xuất 4: telemetry phục vụ hiệu chuẩn

Báo thêm trong status:
- `span_md`: góc thật giữa hai công tắc của mỗi khớp.
- `beta_measured` và `beta_nominal`.
- Số xung bù độ rơ đã phát.

Nhờ đó mỗi lần HOME tự để lại số liệu kiểm tra trong log.

---

## 6. Hiệu chuẩn hình học (ngoài firmware nhưng cần làm)

Firmware không thể tự biết ba thứ: **góc thật của công tắc**, **L1/L2 thật**, và **vị trí ngòi bút** (`tool_offset_mm` hiện đang là (0, 0)).

1. **Góc công tắc.**
   - Nhả J1/J2 bằng lệnh `HOLD … 0`, đẩy tay cho tới khi công tắc kêu.
   - Đo góc so với trục Y của robot bằng thước đo góc. Cách tiện hơn: in một tờ giấy có các vạch góc với tâm đặt tại trục vai, dán dưới robot.
   - Nhập kết quả vào `joint_limits_deg` thay cho ±90.
2. **L1, L2 và ngòi bút.**
   - Dùng thước kẹp đo từ tâm trục vai tới tâm trục khuỷu, và từ tâm khuỷu tới tâm ngòi bút.
   - Nếu ngòi bút không nằm trên đường tâm khâu 2, nhập độ lệch vào `tool_offset_mm`.
3. **Kiểm tra bằng hình vẽ.**
   - Vẽ một hình vuông 60 mm có hai đường chéo, rồi đo 4 cạnh và 2 đường chéo bằng thước kẹp.
   - Cạnh sai tỷ lệ là do L1/L2 hoặc thang góc. Hai đường chéo dài khác nhau nghĩa là hình bị méo (gốc khuỷu, thang góc).
   - Sáu số đo này đủ để GUI tìm (bằng bình phương tối thiểu) gốc khuỷu, L1, L2 và thang góc.
   - Gốc 0 của J1 chỉ làm hình xoay, không làm méo, nên chỉ cần khi muốn đặt hình đúng vị trí tuyệt đối trên giấy.

---

## 7. Bài vẽ thử để chẩn đoán (làm trước khi sửa)

| # | Cách làm | Nếu thấy | Nghĩa là |
|---|---|---|---|
| 1 | Vẽ cùng một đường tròn D46 ở 3 vị trí (gần đế, giữa, xa đế) | Độ méo thay đổi theo vị trí | Lỗi tham số hình học → đề xuất 1, mục 6 |
| | | Có 4 bậc nhỏ ở những chỗ giống nhau | Độ rơ → đề xuất 3 |
| 2 | Vẽ một đoạn thẳng 40 mm đi rồi vẽ về | Hai nét không trùng nhau | Khoảng cách giữa hai nét xấp xỉ độ rơ |
| 3 | HOME 2 lần, mỗi lần chấm cùng một dấu + | Hai dấu lệch nhau 0,3–0,5 mm | k dao động → đề xuất 2 |
| 4 | Vẽ hình vuông có đường chéo rồi đo | Xem mục 6 | |
| 5 | Lắc nhẹ giá bút khi bút đang chạm giấy | Có rơ | Cần sửa cơ khí giá bút |

---

## 8. Các vấn đề khác của firmware (đánh giá lại sau khi có log)

| # | Vấn đề | Bằng chứng | Mức độ | Hướng sửa |
|---|---|---|---|---|
| 1 | Không có E-stop phần cứng, không giám sát nguồn động cơ. Mất 24 V khi đang chạy thì firmware vẫn đếm xung và vẫn báo có mốc | Overlay không có chân nào cho việc này | Cao (an toàn) | Mạch E-stop NC cắt nguồn driver, cộng một GPIO báo MCU; cầu phân áp đo VMOT |
| 2 | Điều khiển vòng hở, không phát hiện mất bước | Do thiết kế | Trung bình (log chưa thấy mất bước) | Lệnh "verify home": đi về công tắc và so số xung; hoặc gắn AS5600 ở khớp (phụ lục A) |
| 3 | Thang góc và k không được đối chiếu với giá trị danh định | Mục 3 | **Cao đối với độ chính xác** | Đề xuất 1–2 |
| 4 | `command()` phân tích lệnh và định dạng chuỗi (`sscanf`, `snprintf`) ngay trong vùng khóa ngắt, kể cả với lệnh KEEP | `timer_late` = 266 khi HOME Z ở 4800 xung/s | Thấp–trung bình | Trong vùng khóa chỉ chép trạng thái; định dạng chuỗi ở ngoài |
| 5 | Status vẫn gửi khi GUI chưa mở cổng; console dùng chung cổng với giao thức | Một dòng JSON hỏng mỗi lần kết nối | Thấp | Kiểm tra DTR bằng `uart_line_ctrl_get`; tắt `CONFIG_UART_CONSOLE` hoặc dùng CDC riêng |
| 6 | Lệnh bị từ chối trong lúc đang chạy làm robot dừng và mất mốc | Test riêng: gửi MOVE khi đang chạy thì robot abort | Thấp (chưa xảy ra trong log) | Từ chối lệnh vô hại mà không abort |
| 7 | STOP và mất heartbeat (350 ms) đều dừng tức thì và luôn làm mất mốc | Do thiết kế | Thấp–trung bình | Dừng có giảm tốc; giữ mốc khi người dùng bấm STOP |
| 8 | Một giới hạn tốc độ chung 1600 xung/s cho mọi trục | Z chỉ đạt 1 mm/s, nên mỗi lần nhấc bút 2 mm mất khoảng 2 s | Trung bình (thời gian vẽ) | Giới hạn tốc độ riêng cho từng trục (Z có thể tới 6400 xung/s như lúc HOME) |
| 9 | PATH không kiểm tra bước nhảy vận tốc của từng trục tại điểm nối | Test riêng: firmware chấp nhận J1 nhảy 250→1000 xung/s, và J2 đảo chiều ở 1000 xung/s | Thấp (GUI đã lập profile tốt; tốc độ ≤ 276 xung/s) | Lớp bảo vệ cuối: giới hạn Δv của từng trục |
| 10 | Không có watchdog phần cứng (IWDG) | `prj.conf` | Thấp | Bật `CONFIG_WATCHDOG`, nạp watchdog trong main loop |
| 11 | Giả định ngầm: hai công tắc của một trục phải cùng port; TIM2 16-bit chỉ đếm tới khoảng 65 ms | `main.c` dòng 36, `app.overlay` | Thấp | Thêm `BUILD_ASSERT` |
| 12 | `main()` gặp lỗi khởi tạo thì chỉ `return 0`, không báo gì | `main.c` dòng 296–305 | Thấp | Đưa về trạng thái an toàn và nháy LED báo mã lỗi |
| 13 | Mã khó đọc; còn mã chết (`zj_command`, `zj_period_us`, `zj_reason_name`); nhiều số ma thuật | | Thấp | Dùng enum và hằng số có tên, xóa mã chết |
| 14 | Flash đã dùng 83%, RAM 87% | `verification_nc_v5_R9.json` | Lưu ý | Cân nhắc STM32F401/F411 nếu thêm tính năng |
| 15 | Không dùng git | | Quy trình | `git init`, đánh tag theo số R |

### Đính chính so với nhận xét trước

- Cờ `"real_hardware_tested": false` trong `verification_nc_v5_R9.json` là thông tin cũ. Log cho thấy firmware đã chạy thật nhiều giờ.
- Trước đây mình đề xuất bắt buộc `entry[k] == exit[k-1]` cho SEG. Điều đó **sai**: GUI cố ý giữ vận tốc **đầu bút** liên tục, nên tốc độ tick giữa hai đoạn khác nhau là đúng. Phép kiểm tra đúng phải là bước nhảy vận tốc **của từng trục** (dòng 9 ở bảng trên).

---

## 9. Thứ tự đề nghị

1. Kiểm tra DIP và loại động cơ J1; sửa cấu hình (`microsteps`, `coupling`).
2. Làm bài thử 1–3 (mục 7) để biết độ rơ lớn hay nhỏ.
3. Làm đề xuất 1 và 2, đo góc công tắc thật, rồi vẽ lại để so sánh.
4. Hiệu chuẩn L1, L2 và ngòi bút (mục 6).
5. Nếu bài thử 2 cho thấy độ rơ lớn hơn khoảng 0,1°, làm đề xuất 3.
6. Các mục an toàn và độ bền ở mục 8 (E-stop, VMOT, DTR, khóa ngắt).

---

## Phụ lục A: Gắn AS5600 để đo góc khớp

**Vị trí gắn:**
- Nên gắn trên **trục khớp**, không gắn ở đuôi động cơ.
  - J1: nam châm ở đầu trục puly 60T của vai, cảm biến bắt vào đế.
  - J2: nam châm ở trục khuỷu, cảm biến bắt trên khâu 1. Cảm biến này đo trực tiếp **góc khuỷu tương đối**, đúng đại lượng IK dùng, nên kiểm tra được cả k.
- Gắn ở khớp còn có lợi thế phát hiện được trượt đai và độ rơ, và biết ngay góc tuyệt đối khi bật nguồn (vì khớp chỉ quay khoảng ±90°).

**Độ phân giải:** 4096 điểm/vòng, tức 0,088°/điểm.
- J1: 1 bước đầy tương ứng 0,6° ở khớp, khoảng 7 điểm.
- J2: 1 bước đầy tương ứng 0,2° ở khớp, khoảng 2 điểm.
- Khi động cơ bước mất đồng bộ, nó thường trượt theo bội của 4 bước đầy, nên phát hiện được dễ dàng.

**Lắp cơ khí:**
- Nam châm Ø6×2,5 mm, **từ hóa ngang** (diametric). Loại từ hóa dọc trục không dùng được.
- Dán bằng epoxy, đặt đúng tâm trục.
- Khe hở giữa nam châm và chip 0,5–3 mm.
- Gá bằng nhựa hoặc nhôm; tránh ốc và vòng đệm thép ở gần cảm biến.
- Kiểm tra bằng thanh ghi `STATUS` (0x0B): MD = 1 là có nam châm, ML là quá xa, MH là quá gần. Thanh ghi `AGC` (0x1A) nên nằm khoảng giữa dải.

**Đấu dây:**
- **Chân DIR phải nối cứng** xuống GND hoặc lên VCC, không để hở.
- **Khi chạy 3,3 V phải nối VDD5V với VDD3V3.**
- **Địa chỉ I²C cố định 0x36**, nên hai cảm biến sẽ trùng địa chỉ. Dùng mux TCA9548A, hoặc dùng AS5600L (đổi được địa chỉ).
- **Dùng I2C1 ở PB6/PB7** (đang trống); PB10/PB11 đã dành cho công tắc J2.
- **Dây tới khuỷu dài:** chạy 100 kHz, pull-up ngoài 2,2 kΩ, dây xoắn đôi (SDA với GND, SCL với GND), đi xa dây pha động cơ.

**Firmware:**
- Đọc thanh ghi RAW ANGLE (0x0C–0x0D) trong một thread riêng ở 100–200 Hz. Không đọc I²C trong ngắt hay trong `irq_lock`.
- So góc đo được với góc tính từ `pos`. Nếu lệch quá khoảng 2–4 bước đầy trong vài lần đọc liên tiếp, dừng robot và hủy mốc HOME.

---

## Phụ lục B: Tự kiểm tra lại từ log

Chạy từ thư mục gốc dự án. Đoạn mã in ra góc thật giữa hai công tắc và k của mỗi lần HOME:

```python
import json, glob

J1_PULSES_PER_DEG = 3200 * 3 / 360   # sửa nếu DIP/động cơ J1 khác
J2_PULSES_PER_DEG = 1600 * 9 / 360

for f in sorted(glob.glob('Software/dist/SCARA_Cartesian_NC_v5_R14/logs/cartesian_*.jsonl')):
    seen = set()
    for line in open(f, encoding='utf-8'):
        if '"referenced":1' not in line:
            continue
        s = json.loads(line).get('status')
        if not s or not s['referenced'] or s['epoch'] in seen:
            continue
        seen.add(s['epoch'])
        r = s['range']
        print(f[-45:], f"J1 {r[1] / J1_PULSES_PER_DEG:.2f}°  J2 {r[2] / J2_PULSES_PER_DEG:.2f}°",
              f"k={s['coupling_ppm'] / 1e6:.4f}  dò={s['probe_da']}/{s['probe_db']}")
```

Sau khi làm đề xuất 1–2, góc in ra phải ổn định, còn k phải đúng bằng 0,3333 ở mọi lần HOME.

---

## Phụ lục C: Giả định của mô phỏng (mục 4)

- Mô hình hai khâu phẳng, L1 = L2 = 98 mm, không có độ lệch ngòi bút. Tâm hình (0, 165) theo khung tọa độ của GUI, là vị trí được dùng trong log.
- Hình mẫu gồm: nét thẳng 46 mm theo hướng tiếp tuyến và theo hướng kính, và đường tròn D46. "Cong" là độ lệch lớn nhất so với đường thẳng khớp nhất; "méo tròn" là Rmax − Rmin so với đường tròn khớp nhất.
- Mỗi trường hợp chỉ có một nguồn sai số. Trên robot thật các sai số cộng dồn với nhau.
- Độ rơ được mô hình hóa đơn giản: khớp trễ b/2 theo chiều đang chuyển động.
- Sai thang góc được mô phỏng là phép co giãn quanh công tắc thấp, công tắc cao, hoặc đối xứng, tùy giả thiết.
