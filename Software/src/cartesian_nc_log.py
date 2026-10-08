"""Per-connection JSONL trace; disk writes never run in the Tk/heartbeat loop."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import queue
import threading
import time
import uuid

class SessionLog:
    def __init__(self,directory):
        root=Path(directory); root.mkdir(parents=True,exist_ok=True)
        stamp=datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        self.path=root/f"cartesian_{stamp}_{os.getpid()}_{uuid.uuid4().hex[:8]}.jsonl"
        self.stream=self.path.open("x",encoding="utf-8",newline="\n")
        self.queue=queue.Queue(maxsize=4096); self.error=None; self.dropped=0
        self.started=time.monotonic(); self.closed=False
        self.thread=threading.Thread(target=self._write,daemon=True,name="SCARA debug log")
        self.thread.start()

    def record(self,kind,**fields):
        if self.closed or self.error: return
        item={"type":kind,"utc":datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
              "elapsed_s":round(time.monotonic()-self.started,6),**fields}
        try: self.queue.put_nowait(item)
        except queue.Full: self.dropped+=1

    def _write(self):
        last_flush=time.monotonic(); reported=0
        try:
            while True:
                try: item=self.queue.get(timeout=.2)
                except queue.Empty:
                    self.stream.flush(); last_flush=time.monotonic(); continue
                if item is None: break
                if self.dropped!=reported:
                    self.stream.write(json.dumps({"type":"log_dropped","count":self.dropped-reported})+"\n")
                    reported=self.dropped
                self.stream.write(json.dumps(item,ensure_ascii=False,separators=(",",":"))+"\n")
                if item["type"] not in ("status","tx") or time.monotonic()-last_flush>.2:
                    self.stream.flush(); last_flush=time.monotonic()
            self.stream.flush()
        except (OSError,TypeError,ValueError) as e:
            self.error=str(e)
        finally:
            try: self.stream.close()
            except OSError as e: self.error=str(e)

    def close(self):
        if self.closed: return
        self.record("log_end",dropped=self.dropped); self.closed=True
        # Closing occurs after STOP/COM close, never before the safety action.
        try: self.queue.put(None,timeout=.2)
        except queue.Full: self.error="Hàng đợi log đầy khi đóng."; return
        self.thread.join(timeout=1)
        if self.thread.is_alive(): self.error="Ghi log chưa hoàn tất khi đóng."
