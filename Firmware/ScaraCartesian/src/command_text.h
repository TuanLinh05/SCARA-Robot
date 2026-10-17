/*
 * ============================================================================
 * FILE: command_text.h
 * ============================================================================
 * [EN] Safe String Tokenization and Integer Parsing Utilities
 *      Provides re-entrant, allocation-free helpers to split command lines
 *      into space-separated tokens and parse bounded 64-bit / 32-bit integers.
 *
 * [VI] Bộ Công cụ Tách Từ (Tokenize) và Đọc Số Nguyên An toàn
 *      Cung cấp các hàm tách chuỗi theo khoảng trắng và chuyển đổi chuỗi sang
 *      số nguyên 64-bit / 32-bit có kiểm tra giới hạn, không dùng cấp phát động
 *      và không dùng biến toàn cục như `strtok`.
 * ============================================================================
 */

#ifndef SCARA_COMMAND_TEXT_H
#define SCARA_COMMAND_TEXT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * [EN] Split `line` on ASCII spaces (' ') into `words[]` using caller-supplied
 *      `storage` buffer. Unlike C's `strtok()`, all state lives in caller buffers.
 * [VI] Tách chuỗi `line` thành mảng các từ `words[]` dựa vào dấu cách (' ').
 *      Mọi dữ liệu được lưu trong bộ đệm `storage` của hàm gọi nên hoàn toàn an toàn.
 */
bool command_split(
    const char *line,
    char *storage,
    size_t capacity,
    char **words,
    unsigned word_capacity,
    unsigned *count,
    bool printable
);

/*
 * [EN] Parse a signed 64-bit integer from `word` and verify `low <= *value <= high`.
 * [VI] Đọc số nguyên có dấu 64-bit từ chuỗi `word` và kiểm tra nằm trong đoạn [low, high].
 */
bool command_number_i64(const char *word, int64_t low, int64_t high, int64_t *value);

/*
 * [EN] Parse an unsigned 32-bit integer from `word` with overflow protection.
 * [VI] Đọc số nguyên không dấu 32-bit từ chuỗi `word` có chống tràn số.
 */
bool command_number_u32(const char *word, uint32_t *value);

/*
 * [EN] Extract the uppercase verb (first token, e.g. "MOVE", "INIT", "STOP")
 *      for ACK responses, or write "INVALID" if malformed.
 * [VI] Trích xuất tên lệnh viết hoa ở đầu dòng (ví dụ "MOVE", "INIT", "STOP")
 *      để đóng gói bản tin phản hồi ACK, hoặc trả về "INVALID" nếu sai cú pháp.
 */
void command_operation(const char *line, char operation[16]);

#endif /* SCARA_COMMAND_TEXT_H */
