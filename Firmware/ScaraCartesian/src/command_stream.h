#ifndef SCARA_COMMAND_STREAM_H
#define SCARA_COMMAND_STREAM_H

#include <stdbool.h>
#include <stddef.h>

#define COMMAND_LINE_CAPACITY 192U
enum command_stream_result { COMMAND_PENDING, COMMAND_COMPLETE, COMMAND_INVALID };
struct command_stream {
    char line[COMMAND_LINE_CAPACITY];
    size_t used;
    bool discard;
};

/* Reset on every USB epoch change; partial commands cannot cross sessions. */
void command_stream_reset(struct command_stream *stream);
/* COMPLETE exposes stream->line until the next feed. INVALID fires once per
 * rejected line; the caller must halt immediately, then drain to newline. */
enum command_stream_result command_stream_feed(struct command_stream *stream, unsigned char ch);

#endif
