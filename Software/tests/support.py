"""Shared calibrated offline status for unit tests; never opens a COM port."""

from cartesian_nc_demo import DemoLink


def fixture():
    demo = DemoLink("DEMO")
    try:
        demo.referenced = True
        demo.stage = 8
        demo.pos = (35200, 0, 1800)
        demo.emit()
        events = []
        while not demo.events.empty():
            events.append(demo.events.get())
        return events[-1][2]
    finally:
        demo.close()
