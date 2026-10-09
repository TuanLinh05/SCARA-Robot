#ifndef SCARA_COMMAND_TEXT_H
#define SCARA_COMMAND_TEXT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Split on ASCII spaces only. All storage belongs to the caller, so parsers
 * can be interleaved without strtok's shared cursor. Printable validation is
 * optional to preserve each existing command entry point's grammar. */
bool command_split(const char *line, char *storage, size_t capacity,
    char **words, unsigned word_capacity, unsigned *count, bool printable);
bool command_number_i64(const char *word, int64_t low, int64_t high, int64_t *value);
bool command_number_u32(const char *word, uint32_t *value);
void command_operation(const char *line, char operation[16]);

#endif
