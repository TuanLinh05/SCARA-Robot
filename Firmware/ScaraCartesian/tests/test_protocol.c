/* Host-only boundary coverage for the real firmware's protocol adapters. */
#include "cart_core.h"
#include "cart_telemetry.h"
#include "command_stream.h"
#include "command_text.h"
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

static void text_tests(void)
{
    char first[192], second[192], *words[13], *other[13];
    unsigned count;
    assert(command_split("  HELLO   7  ", first, sizeof(first), words, 13, &count, false));
    assert(count == 2 && !strcmp(words[0], "HELLO") && !strcmp(words[1], "7"));
    assert(command_split("STOP USER", second, sizeof(second), other, 13, &count, false));
    assert(count == 2 && !strcmp(words[1], "7")); /* independent parser storage */
    assert(!command_split("   ", first, sizeof(first), words, 13, &count, false));
    assert(command_split("A B C D E F G H I J K L M", first, sizeof(first), words, 13, &count, false));
    assert(count == 13);
    assert(!command_split("A B C D E F G H I J K L M N", first, sizeof(first), words, 13, &count, false));
    assert(command_split("HELLO\t7", first, sizeof(first), words, 13, &count, false));
    assert(count == 1); /* Cartesian direct API keeps its space-only grammar. */
    assert(!command_split("HELLO\t7", first, sizeof(first), words, 13, &count, true));
    char oversized[193]; memset(oversized, 'A', sizeof(oversized)); oversized[192] = 0;
    assert(!command_split(oversized, first, sizeof(first), words, 13, &count, false));

    int64_t signed_value = 99;
    assert(command_number_i64("-0", INT64_MIN, INT64_MAX, &signed_value) && signed_value == 0);
    assert(command_number_i64("9223372036854775807", INT64_MIN, INT64_MAX, &signed_value) && signed_value == INT64_MAX);
    assert(command_number_i64("-9223372036854775808", INT64_MIN, INT64_MAX, &signed_value) && signed_value == INT64_MIN);
    const char *invalid[] = {"", "-", "+1", "1.0", "1 ", "--1", "9223372036854775808", "-9223372036854775809"};
    for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); i++)
        assert(!command_number_i64(invalid[i], INT64_MIN, INT64_MAX, &signed_value));
    assert(!command_number_i64("11", 0, 10, &signed_value));
    errno = ERANGE;
    assert(command_number_i64("10", 0, 10, &signed_value) && signed_value == 10);
    uint32_t unsigned_value;
    assert(command_number_u32("4294967295", &unsigned_value) && unsigned_value == UINT32_MAX);
    assert(command_number_u32("000", &unsigned_value) && unsigned_value == 0);
    assert(!command_number_u32("4294967296", &unsigned_value));
    assert(!command_number_u32("-0", &unsigned_value));
    assert(!command_number_u32("+1", &unsigned_value));

    char operation[16];
    command_operation("  STOP USER", operation); assert(!strcmp(operation, "STOP"));
    command_operation("move 7", operation); assert(!strcmp(operation, "INVALID"));
    command_operation("", operation); assert(!strcmp(operation, "INVALID"));
    command_operation("ABCDEFGHIJKLMNOPQ 7", operation); assert(!strcmp(operation, "ABCDEFGHIJKLMNO"));
}

static enum command_stream_result feed(struct command_stream *stream, const char *text)
{
    enum command_stream_result result = COMMAND_PENDING;
    for (; *text; text++) result = command_stream_feed(stream, (unsigned char)*text);
    return result;
}

static void stream_tests(void)
{
    struct command_stream stream = {0};
    assert(feed(&stream, "HELLO 7\r\n") == COMMAND_COMPLETE && !strcmp(stream.line, "HELLO 7"));
    assert(feed(&stream, "\n\r\n") == COMMAND_PENDING);
    for (unsigned i = 0; i < COMMAND_LINE_CAPACITY - 1; i++)
        assert(command_stream_feed(&stream, 'A') == COMMAND_PENDING);
    assert(command_stream_feed(&stream, '\n') == COMMAND_COMPLETE);
    assert(strlen(stream.line) == COMMAND_LINE_CAPACITY - 1);
    for (unsigned i = 0; i < COMMAND_LINE_CAPACITY - 1; i++) command_stream_feed(&stream, 'B');
    assert(command_stream_feed(&stream, 'B') == COMMAND_INVALID);
    assert(feed(&stream, "STOP USER\r\n") == COMMAND_PENDING); /* overlong suffix discarded */
    assert(feed(&stream, "STATUS\n") == COMMAND_COMPLETE && !strcmp(stream.line, "STATUS"));
    const unsigned char invalid[] = {0, 9, 31, 127, 255};
    for (unsigned i = 0; i < sizeof(invalid); i++) {
        feed(&stream, "HELLO ");
        assert(command_stream_feed(&stream, invalid[i]) == COMMAND_INVALID);
        assert(command_stream_feed(&stream, invalid[i]) == COMMAND_PENDING);
        assert(feed(&stream, "7\n") == COMMAND_PENDING);
        assert(feed(&stream, "HELLO 8\n") == COMMAND_COMPLETE && !strcmp(stream.line, "HELLO 8"));
    }
    feed(&stream, "HELLO "); command_stream_reset(&stream);
    assert(feed(&stream, "9\n") == COMMAND_COMPLETE && !strcmp(stream.line, "9"));
    command_stream_feed(&stream, 0); command_stream_reset(&stream);
    assert(feed(&stream, "STOP\n") == COMMAND_COMPLETE && !strcmp(stream.line, "STOP"));
}

static void command_tests(void)
{
    struct ct_state state;
    ct_init(&state);
    assert(ct_command(&state, "STATUS", 0) == CT_OK);
    assert(ct_command(&state, "STOP FOCUS", 0) == CT_OK);
    assert(ct_command(&state, "STOP", 0) == CT_OK && !strcmp(state.reason, "stop_focus"));
    assert(ct_command(&state, "HELLO 4294967295", 0) == CT_OK && state.session == UINT32_MAX);
    assert(ct_command(&state, "HELLO 4294967296", 0) == CT_REJECTED && !strcmp(state.reply, "session"));
    assert(ct_command(&state, "HELLO +7", 0) == CT_REJECTED);
    assert(ct_command(&state, "  HELLO   7  ", 0) == CT_OK);
    state.running = true;
    assert(ct_command(&state, "A B C D E F G H I J K L M N", 0) == CT_REJECTED);
    assert(!state.running && !strcmp(state.reason, "invalid_command"));
    struct zj_state jog;
    zj_init(&jog);
    assert(zj_command(&jog, "  HELLO  7 ", 0) == ZJ_OK && jog.session == 7);
    assert(zj_command(&jog, "HELLO\t7", 0) == ZJ_REJECTED);
}

static void telemetry_tests(void)
{
    struct ct_state state;
    struct ct_status snapshot;
    ct_init(&state);
    state.session = 7; state.job = 2;
    state.pos[CT_Z] = 100; state.origin[CT_Z] = 10; state.goal[CT_Z] = 120;
    state.total[CT_J2] = UINT64_MAX;
    ct_status_capture(&snapshot, &state, true, 9, 20);
    assert(snapshot.busy && snapshot.late == 9 && snapshot.p[CT_Z] == 90 && snapshot.g[CT_Z] == 110);
    state.pos[CT_Z] = 900; state.session = 8; state.reason = "changed";
    char output[CT_STATUS_CAPACITY];
    int length = ct_status_format(output, sizeof(output), &snapshot, 30);
    assert(length > 0 && (size_t)length < sizeof(output));
    assert(strstr(output, "\"session\":7,\"job\":2,\"up_ms\":30"));
    assert(strstr(output, "\"pos\":[90,0,0],\"goal\":[110,0,0]"));
    assert(strstr(output, "18446744073709551615"));
    assert(!strstr(output, "changed"));
    char small[8];
    assert(ct_status_format(small, sizeof(small), &snapshot, 30) == length && small[sizeof(small) - 1] == 0);
    const char *ack = "{\"type\":\"ack\",\"protocol\":5,\"session\":7,\"job\":2,\"op\":\"STOP\",\"ok\":1,\"reason\":\"ok\"}\n";
    length = ct_ack_format(output, sizeof(output), 7, 2, "STOP", true, "ok");
    assert((size_t)length == strlen(ack) && !strcmp(output, ack));
}

int main(void)
{
    text_tests(); stream_tests(); command_tests(); telemetry_tests();
    puts("protocol: independent parsers, numeric bounds, USB framing/discard/reset, ACK bytes, stable telemetry passed");
    return 0;
}
