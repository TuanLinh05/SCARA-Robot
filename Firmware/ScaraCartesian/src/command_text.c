/*
 * ============================================================================
 * FILE: command_text.c
 * ============================================================================
 * [EN] Implementation of re-entrant command line splitting and integer parsing.
 * [VI] Triển khai các hàm tách từ và đọc số nguyên an toàn cho bộ giải mã lệnh.
 *
 * [DEBUG TIP / MẸO GỠ LỖI]:
 * - `command_number_i64` strictly rejects trailing letters or decimal points
 *   (e.g., "100.0" or "100ms" returns `false`). Firmware only accepts integers!
 * - Hàm `command_number_i64` chỉ chấp nhận chuỗi số nguyên thuần túy (ví dụ
 *   "100.0" hay "100ms" sẽ trả về `false`). Mọi phép tính số thực đã được quy
 *   đổi ra số nguyên xung/millidegree từ phía máy tính.
 * ============================================================================
 */

#include "command_text.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * [EN] Copy `line` into `storage` and replace spaces with '\0' to populate `words[]`.
 * [VI] Sao chép `line` vào `storage` và thay khoảng trắng bằng '\0' để trỏ `words[]` tới từng từ.
 */
bool command_split(
    const char *line,
    char *storage,
    size_t capacity,
    char **words,
    unsigned word_capacity,
    unsigned *count,
    bool printable
)
{
    size_t length = strlen(line);
    *count = 0;

    /*
     * [EN] Reject empty lines or lines that would overflow caller storage.
     * [VI] Từ chối dòng rỗng hoặc dòng vượt quá kích thước bộ đệm `storage`.
     */
    if (length == 0U || length >= capacity) {
        return false;
    }

    memcpy(storage, line, length + 1U);

    char *p = storage;
    while (*p != '\0') {
        /*
         * [EN] Skip leading spaces between tokens.
         * [VI] Bỏ qua các dấu cách thừa giữa các từ.
         */
        while (*p == ' ') {
            p++;
        }
        if (*p == '\0') {
            break;
        }

        /*
         * [EN] Reject if token count exceeds `word_capacity`.
         * [VI] Từ chối nếu số lượng từ vượt quá sức chứa mảng `words`.
         */
        if (*count == word_capacity) {
            return false;
        }
        words[*count] = p;
        (*count)++;

        /*
         * [EN] Advance to the end of the current token, validating printable ASCII if requested.
         * [VI] Duyệt tới cuối từ hiện tại, đồng thời kiểm tra ký tự in được nếu bật cờ `printable`.
         */
        while (*p != '\0' && *p != ' ') {
            unsigned char uch = (unsigned char)*p;
            if (printable && (uch < 33U || uch > 126U)) {
                return false;
            }
            p++;
        }

        /*
         * [EN] Null-terminate the current token in-place and move past it.
         * [VI] Đặt ký tự kết thúc chuỗi '\0' tại vị trí dấu cách và tiến tới từ tiếp theo.
         */
        if (*p != '\0') {
            *p = '\0';
            p++;
        }
    }

    return (*count != 0U);
}

/*
 * [EN] Strictly parse a signed decimal integer within [low, high].
 * [VI] Chuyển đổi chuỗi số nguyên có dấu hệ 10 và kiểm tra nằm trong khoảng [low, high].
 */
bool command_number_i64(const char *word, int64_t low, int64_t high, int64_t *value)
{
    if (*word == '\0') {
        return false;
    }
    if (*word != '-' && (*word < '0' || *word > '9')) {
        return false;
    }

    /*
     * [EN] Verify all remaining characters are digits '0'..'9'.
     * [VI] Đảm bảo tất cả các ký tự phía sau đều là chữ số từ '0' đến '9'.
     */
    const char *p = word;
    if (*p == '-') {
        p++;
    }
    if (*p == '\0') {
        return false;
    }
    for (; *p != '\0'; p++) {
        if (*p < '0' || *p > '9') {
            return false;
        }
    }

    char *end = NULL;
    errno = 0;
    long long parsed = strtoll(word, &end, 10);
    if (errno != 0 || *end != '\0' || parsed < low || parsed > high) {
        return false;
    }

    *value = (int64_t)parsed;
    return true;
}

/*
 * [EN] Strictly parse an unsigned 32-bit decimal integer.
 * [VI] Chuyển đổi chuỗi thành số nguyên không dấu 32-bit, kiểm tra tràn số từng chữ số.
 */
bool command_number_u32(const char *word, uint32_t *value)
{
    uint32_t parsed = 0;
    if (*word == '\0') {
        return false;
    }

    while (*word != '\0') {
        if (*word < '0' || *word > '9') {
            return false;
        }
        uint32_t digit = (uint32_t)(*word - '0');
        word++;

        /*
         * [EN] Check for 32-bit unsigned overflow before multiplying by 10.
         * [VI] Kiểm tra tràn số 32-bit trước khi nhân 10 và cộng chữ số mới.
         */
        if (parsed > (UINT32_MAX - digit) / 10U) {
            return false;
        }
        parsed = parsed * 10U + digit;
    }

    *value = parsed;
    return true;
}

/*
 * [EN] Extract the command keyword (uppercase letters only, up to 15 chars) for ACK messages.
 * [VI] Lấy từ khóa tên lệnh ở đầu dòng (chỉ gồm chữ cái in hoa A..Z, tối đa 15 ký tự).
 */
void command_operation(const char *line, char operation[16])
{
    while (*line != '\0' && (unsigned char)*line <= ' ') {
        line++;
    }
    if (*line == '\0') {
        strcpy(operation, "INVALID");
        return;
    }

    unsigned i = 0;
    while (line[i] != '\0' && (unsigned char)line[i] > ' ' && i < 15U) {
        if (line[i] < 'A' || line[i] > 'Z') {
            strcpy(operation, "INVALID");
            return;
        }
        operation[i] = line[i];
        i++;
    }
    operation[i] = '\0';
}
