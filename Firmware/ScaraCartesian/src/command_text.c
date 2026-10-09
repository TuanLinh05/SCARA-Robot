#include "command_text.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

bool command_split(const char *line, char *storage, size_t capacity,
    char **words, unsigned word_capacity, unsigned *count, bool printable)
{
    size_t length = strlen(line);
    *count = 0;
    if (!length || length >= capacity) return false;
    memcpy(storage, line, length + 1);
    for (char *p = storage; *p;) {
        while (*p == ' ') p++;
        if (!*p) break;
        if (*count == word_capacity) return false;
        words[(*count)++] = p;
        while (*p && *p != ' ') {
            if (printable && ((unsigned char)*p < 33 || (unsigned char)*p > 126)) return false;
            p++;
        }
        if (*p) *p++ = 0;
    }
    return *count != 0;
}

bool command_number_i64(const char *word, int64_t low, int64_t high, int64_t *value)
{
    if (!*word || (*word != '-' && (*word < '0' || *word > '9'))) return false;
    for (const char *p = word + (*word == '-'); *p; p++)
        if (*p < '0' || *p > '9') return false;
    char *end;
    errno = 0;
    long long parsed = strtoll(word, &end, 10);
    if (errno || *end || parsed < low || parsed > high) return false;
    *value = parsed;
    return true;
}

bool command_number_u32(const char *word, uint32_t *value)
{
    uint32_t parsed = 0;
    if (!*word) return false;
    while (*word) {
        if (*word < '0' || *word > '9') return false;
        uint32_t digit = (uint32_t)(*word++ - '0');
        if (parsed > (UINT32_MAX - digit) / 10) return false;
        parsed = parsed * 10 + digit;
    }
    *value = parsed;
    return true;
}

void command_operation(const char *line, char operation[16])
{
    strcpy(operation, "INVALID");
    sscanf(line, "%15s", operation);
    for (const char *p = operation; *p; p++) {
        if (*p < 'A' || *p > 'Z') {
            strcpy(operation, "INVALID");
            break;
        }
    }
}
