/*
 * ============================================================================
 * FILE: command_stream.h
 * ============================================================================
 * [EN] USB/UART Byte-Stream Framing Interface
 *      Assembles raw incoming serial bytes one character at a time into
 *      null-terminated ASCII command lines. Detects buffer overflows and
 *      non-printable control characters immediately so corrupted packets
 *      never execute partially.
 *
 * [VI] Giao diện Đóng gói Luồng Byte USB/UART
 *      Ghép từng ký tự nhận được qua cổng nối tiếp thành một dòng lệnh ASCII
 *      hoàn chỉnh kết thúc bằng '\0'. Phát hiện ngay lập tức lỗi tràn bộ đệm
 *      hoặc ký tự lạ không in được để ngăn chặn việc thực thi lệnh bị hỏng.
 *
 * [DEBUG TIP / MẸO GỠ LỖI]:
 * - If commands from Python/PC seem ignored, inspect `stream->discard` and
 *   `stream->used` in debugger. A missing '\n' at the end of a sent string
 *   leaves the state in `COMMAND_PENDING`.
 * - Nếu lệnh gửi từ máy tính không phản hồi, hãy kiểm tra biến `stream->discard`
 *   và `stream->used`. Nếu quên gửi ký tự xuống dòng '\n', hàm sẽ luôn trả về
 *   `COMMAND_PENDING`.
 * ============================================================================
 */

#ifndef SCARA_COMMAND_STREAM_H
#define SCARA_COMMAND_STREAM_H

#include <stdbool.h>
#include <stddef.h>

/*
 * [EN] Maximum length of a single ASCII command line (including null terminator '\0').
 * [VI] Độ dài tối đa của một dòng lệnh ASCII (bao gồm ký tự kết thúc chuỗi '\0').
 */
#define COMMAND_LINE_CAPACITY 192U

/*
 * [EN] Return codes when feeding a single byte into the command stream:
 *      - COMMAND_PENDING : Waiting for more bytes or newline '\n'.
 *      - COMMAND_COMPLETE: A valid, complete line is ready in `stream->line`.
 *      - COMMAND_INVALID : Line overflowed or contained non-printable bytes.
 *
 * [VI] Các trạng thái trả về khi nạp 1 byte vào bộ nhận lệnh:
 *      - COMMAND_PENDING : Đang chờ thêm ký tự hoặc dấu xuống dòng '\n'.
 *      - COMMAND_COMPLETE: Đã nhận đủ 1 dòng lệnh hợp lệ trong `stream->line`.
 *      - COMMAND_INVALID : Dòng lệnh quá dài hoặc chứa ký tự lỗi không in được.
 */
enum command_stream_result {
    COMMAND_PENDING,
    COMMAND_COMPLETE,
    COMMAND_INVALID
};

/*
 * [EN] State structure holding the partial command line being received.
 * [VI] Cấu trúc lưu trạng thái của dòng lệnh đang được nhận dở dang.
 */
struct command_stream {
    char line[COMMAND_LINE_CAPACITY]; /* [EN] Character buffer / [VI] Bộ đệm lưu chuỗi lệnh */
    size_t used;                      /* [EN] Current byte count / [VI] Số ký tự đã nhận */
    bool discard;                     /* [EN] True if current line is tainted / [VI] Cờ bỏ qua dòng lỗi cho tới '\n' */
};

/*
 * [EN] Reset stream state to empty. Must be called on startup and on every
 *      USB reconnect/epoch change so partial commands never cross sessions.
 * [VI] Đặt lại bộ đệm nhận lệnh về trạng thái rỗng. Cần gọi khi khởi động
 *      hoặc khi cáp USB bị rút ra cắm lại để tránh ghép nhầm lệnh cũ.
 */
void command_stream_reset(struct command_stream *stream);

/*
 * [EN] Feed one raw byte into the framing state machine.
 *      - COMMAND_COMPLETE exposes `stream->line` until the next feed call.
 *      - COMMAND_INVALID fires once per rejected line; caller must halt motion
 *        immediately, while the stream drains remaining bytes until '\n'.
 * [VI] Nạp 1 byte thô vào máy trạng thái đóng gói lệnh.
 *      - Khi trả về COMMAND_COMPLETE, chuỗi `stream->line` sẵn sàng để xử lý.
 *      - Khi trả về COMMAND_INVALID, tầng gọi phải dừng robot ngay lập tức,
 *        bộ đệm sẽ tự động xả bỏ các ký tự còn lại cho đến khi gặp '\n'.
 */
enum command_stream_result command_stream_feed(struct command_stream *stream, unsigned char ch);

#endif /* SCARA_COMMAND_STREAM_H */
