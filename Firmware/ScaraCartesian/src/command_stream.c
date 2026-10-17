/*
 * ============================================================================
 * FILE: command_stream.c
 * ============================================================================
 * [EN] Implementation of the byte-by-byte USB/UART command line framer.
 *      Reads raw bytes from the serial port one by one and groups them into
 *      clean ASCII lines terminated by '\0'.
 *
 * [VI] Triển khai bộ ghép dòng lệnh từng byte cho giao tiếp USB/UART.
 *      Nhận từng byte thô từ cổng nối tiếp và ghép thành chuỗi lệnh ASCII
 *      hoàn chỉnh kết thúc bằng '\0'.
 * ============================================================================
 */

#include "command_stream.h"

/*
 * [EN] Clear byte counter and error-discard flag.
 * [VI] Xóa bộ đếm số byte và cờ báo lỗi để sẵn sàng nhận dòng lệnh mới.
 */
void command_stream_reset(struct command_stream *stream)
{
    stream->used = 0;
    stream->discard = false;
}

/*
 * [EN] Process a single incoming byte `ch` from UART/USB CDC-ACM:
 *      1. Ignore carriage return '\r' (supports both "\n" and "\r\n" endings).
 *      2. On newline '\n', finalize null-terminated string if valid and non-empty.
 *      3. Reject any byte outside printable ASCII [32..126] or exceeding buffer size.
 *
 * [VI] Xử lý từng ký tự `ch` đọc từ cổng nối tiếp USB/UART:
 *      1. Bỏ qua ký tự '\r' (hỗ trợ cả chuẩn xuống dòng Windows "\r\n" và Linux "\n").
 *      2. Khi gặp '\n', kết thúc chuỗi bằng '\0' nếu dòng lệnh không bị lỗi và không rỗng.
 *      3. Báo lỗi COMMAND_INVALID nếu tràn bộ đệm hoặc gặp ký tự ngoài vùng ASCII [32..126].
 */
enum command_stream_result command_stream_feed(struct command_stream *stream, unsigned char ch)
{
    /*
     * [EN] Step 1: Ignore CR ('\r') so CRLF and LF line endings behave identically.
     * [VI] Bước 1: Bỏ qua ký tự '\r' để tương thích cả Windows ("\r\n") và Linux ("\n").
     */
    if (ch == '\r') {
        return COMMAND_PENDING;
    }

    /*
     * [EN] Step 2: End of line ('\n') reached: check if the buffered line is valid and non-empty.
     * [VI] Bước 2: Đã gặp ký tự xuống dòng '\n': kiểm tra xem dòng lệnh có hợp lệ không.
     */
    if (ch == '\n') {
        bool complete = (!stream->discard) && (stream->used > 0);
        stream->line[stream->used] = '\0';
        command_stream_reset(stream);
        return complete ? COMMAND_COMPLETE : COMMAND_PENDING;
    }

    /*
     * [EN] Step 3: If current line was already marked invalid, keep discarding until '\n'.
     * [VI] Bước 3: Nếu dòng hiện tại đã bị đánh dấu lỗi, tiếp tục bỏ qua các ký tự cho tới '\n'.
     */
    if (stream->discard) {
        return COMMAND_PENDING;
    }

    /*
     * [EN] Step 4: Guard against buffer overflow and non-printable control characters.
     *      Printable ASCII characters have byte codes from 32 (' ') to 126 ('~').
     * [VI] Bước 4: Kiểm tra tràn bộ đệm (chừa 1 byte cho '\0') và lọc ký tự ASCII in được
     *      (mã ASCII từ 32 là dấu cách ' ' đến 126 là dấu '~').
     */
    size_t max_chars = sizeof(stream->line) - 1U;
    bool is_printable = (ch >= 32U) && (ch <= 126U);
    if (stream->used >= max_chars || !is_printable) {
        stream->used = 0;
        stream->discard = true;
        return COMMAND_INVALID;
    }

    /*
     * [EN] Step 5: Append valid character to buffer and wait for more bytes.
     * [VI] Bước 5: Lưu ký tự hợp lệ vào bộ đệm và chờ ký tự tiếp theo.
     */
    stream->line[stream->used] = (char)ch;
    stream->used++;
    return COMMAND_PENDING;
}
