#include "command_stream.h"

void command_stream_reset(struct command_stream *stream)
{
    stream->used = 0;
    stream->discard = false;
}

enum command_stream_result command_stream_feed(struct command_stream *stream, unsigned char ch)
{
    if (ch == '\r') return COMMAND_PENDING;
    if (ch == '\n') {
        bool complete = !stream->discard && stream->used;
        stream->line[stream->used] = 0;
        command_stream_reset(stream);
        return complete ? COMMAND_COMPLETE : COMMAND_PENDING;
    }
    if (stream->discard) return COMMAND_PENDING;
    if (stream->used >= sizeof(stream->line) - 1 || ch < 32 || ch > 126) {
        stream->used = 0;
        stream->discard = true;
        return COMMAND_INVALID;
    }
    stream->line[stream->used++] = (char)ch;
    return COMMAND_PENDING;
}
