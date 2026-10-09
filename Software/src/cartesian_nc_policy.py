"""Pure motion gates and USB timing policy, independent of Tk and serial I/O."""

STATUS_TIMEOUT_S = 0.8
HANDSHAKE_TIMEOUT_S = 3.0
ACK_TIMEOUT_S = 1.0
KEEP_INTERVAL_S = 0.08
MAX_EVENTS_PER_POLL = 128
GUI_POLL_MS = 25
PATH_POLL_MS = 10


def status_fresh(link, state, received, now):
    return (
        link is not None
        and state is not None
        and state.session == link.session
        and now - received < STATUS_TIMEOUT_S
    )


def status_expired(state, opened, received, now):
    if state is None:
        return now - opened > HANDSHAKE_TIMEOUT_S
    return now - received > STATUS_TIMEOUT_S


def hardware_ready(state, config_error=None):
    return (
        state is not None
        and not config_error
        and not state.fault
        and state.ready == 7
        and not state.conflicts
        and not state.errors
    )


def background_allowed(task_kind, config):
    if task_kind == "INIT":
        return config.home_in_background
    return config.motion_in_background
