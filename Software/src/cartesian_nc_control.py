"""SCARA XYZ with automatic NC homing/calibration. No boot motion; --demo is offline."""
import argparse
import ctypes
import json
from dataclasses import asdict
from pathlib import Path
import queue
import sys
import time
import threading
import tkinter as tk
from tkinter import ttk
from cartesian_nc_model import NCConfig
from cartesian_nc_protocol import NCLink, Ack, BUILD
from cartesian_nc_log import SessionLog
from cartesian_nc_editor import WorkspaceEditor
from cartesian_nc_workspace import bounded_pose, validate_program
from cartesian_nc_drawing_gui import DrawingUI
from cartesian_nc_path import PathUpload
from cartesian_nc_shapes import PATTERNS

BG,CARD,TEXT,MUTED="#0b1220","#142136","#ecf3fc","#9fb1ca"
BLUE,GREEN,RED="#5aaeff","#60d5ae","#ff8795"
CONFIG_PATH=Path(sys.executable).parent/"cartesian_nc_config.json" if getattr(sys,"frozen",False) else Path(__file__).resolve().parents[1]/"cartesian_nc_config.json"
STAGES=("Chưa lấy mốc","Z: đo hai biên","J2: đo sơ bộ","J2: về giữa","J1: đo và bù J2",
        "J1: về mốc góc","J2: lấy lại mốc","Về tư thế đỗ","Sẵn sàng XYZ","Khởi tạo thất bại",
        "Đo bù: dịch J1 một đoạn nhỏ","Đo bù: đo lại biên J2","Đo bù: trở về tư thế ban đầu")
PHASES=("Chờ","Tìm biên −","Lùi biên −","Tiếp cận chậm −","Quét đến biên +",
        "Lùi biên +","Tiếp cận chậm +","Quét lại biên −","Lùi biên −","Tiếp cận chậm −","Hoàn tất","Lỗi")
Z_PHASES=("Chờ","Tìm biên trên","Lùi khỏi biên trên","Tiếp cận chậm biên trên","Quét xuống biên dưới",
          "Lùi khỏi biên dưới","Tiếp cận chậm biên dưới","Quét lên biên trên","Lùi khỏi biên trên","Tiếp cận chậm biên trên","Hoàn tất","Lỗi")
ERRORS=("","Đã hủy","Mất heartbeat","GPIO/timer lỗi","Đầu vào công tắc lỗi","Vượt ngân sách tìm biên",
        "Chạm chưa được xác nhận ổn định","Hai lần đo không lặp lại","Lệnh calib không hợp lệ")
REASONS={"unreferenced":"Cần HOME + CALIB","initialized":"Đã lấy mốc, sẵn sàng XYZ",
         "initializing":"Đang khởi tạo","moving":"Đang di chuyển","complete":"Đã đến đích",
         "stopped":"Đã dừng; home lại nếu đang chạy","heartbeat_lost":"Mất heartbeat; đã dừng",
         "both_limits":"Hai biên cùng mở: kiểm tra NC/GND rồi reset nếu lỗi bị giữ",
         "both_open":"Đã đọc HIGH/HIGH; chuyển động đã bị hủy",
         "both_latched":"HIGH/HIGH đủ 20 ms: kiểm tra NC/GND, rồi reset STM32",
         "checking_inputs":"Đã dừng xung, đang xác nhận tín hiệu công tắc",
         "switch_unstable":"Tín hiệu không ổn định trở lại trong 200 ms; đã hủy",
         "calibration_failed":"Calib thất bại","j2_guard":"J2 chạm biên khi J1 calib; kiểm tra hệ số bù/chiều",
         "j2_limit":"Đã chạm biên J2","j1_limit":"Đã chạm biên J1","z_limit":"Đã chạm biên Z",
         "switch_stale":"Dữ liệu công tắc quá hạn trong firmware","switch_io":"Lỗi đọc GPIO công tắc",
         "switch_not_ready":"Đợi công tắc ổn định","motor_gpio_error":"Đầu ra motor lỗi; cần reset",
         "waiting_switch":"Đợi tín hiệu ổn định trước khi chạy trục tiếp theo",
         "switch_transition_unstable":"Tín hiệu không ổn định khi bắt đầu chạy trục",
         "timer_error":"Timer lỗi; cần reset","released":"Arm đang nhả lực",
         "coupling_sign":"Dấu bù đo được ngược cấu hình: kiểm tra chiều J2; xem log",
         "coupling_measurement":"Phép đo bù quá nhỏ hoặc vượt giới hạn; xem log",
         "coupling_geometry":"Hệ số góc suy ra ngoài phạm vi; kiểm tra góc tại công tắc",
         "coupling_repeat":"Hành trình J2 thay đổi khi đo bù; kiểm tra cơ khí/công tắc",
         "coupling_probe_limit":"Không đủ khoảng cho bước thử J1; xem log",
         "coupling_probe_short":"Bước thử J1 quá ngắn để đo bù",
         "soft_limit":"Vượt giới hạn góc/Z","usb_lost":"USB mất kết nối","z_clearance":"Khoảng lùi Z không phù hợp hành trình"}
STOP_REASONS={"USER":"stop_user","ESC":"stop_escape","FOCUS":"stop_focus","MINIMIZE":"stop_minimize",
              "DISCONNECT":"stop_disconnect","CLOSE":"stop_close","STALE":"stop_stale","ACK":"stop_ack_timeout","MODEL":"stop_model"}
REASONS.update({"stop_user":"Đã dừng bằng nút DỪNG","stop_escape":"Đã dừng bằng phím Esc",
                "stop_focus":"Đã dừng vì GUI mất focus","stop_minimize":"Đã dừng vì GUI bị thu nhỏ",
                "stop_disconnect":"Đã dừng khi ngắt COM","stop_close":"Đã dừng khi đóng GUI",
                "stop_stale":"Đã dừng vì dữ liệu USB quá hạn","stop_ack_timeout":"Đã dừng vì lệnh không được xác nhận",
                "stop_model":"Đã dừng vì mốc/mô hình không hợp lệ"})
REASONS.update({"path_loading":"Đang nạp bộ đệm nét","path_running":"Đang vẽ nét liên tục",
                "path_starved":"Hết bộ đệm đường vẽ; đã dừng. Gửi log để kiểm tra USB/GUI.",
                "path_sequence":"Sai thứ tự/bộ đệm đường vẽ","path_profile":"Thông số tăng tốc đường vẽ không hợp lệ"})

GUI_VERSION="SCARA_GUI_V5_R14"

class NCApp(WorkspaceEditor,DrawingUI,tk.Tk):
    def __init__(self,factory=NCLink,config_path=CONFIG_PATH,demo=False):
        if sys.platform=="win32":
            try: ctypes.windll.shcore.SetProcessDpiAwareness(1)
            except (OSError,AttributeError): pass
        super().__init__(); self.tk.call("tk","scaling",96/72)
        self.title("SCARA · MOTION & DRAWING STUDIO · GUI R14 · FW R9"+(" · MÔ PHỎNG" if demo else ""))
        self.geometry("1360x940"); self.minsize(1160,900); self.configure(bg=BG)
        self.factory,self.config_path,self.demo=factory,Path(config_path),demo
        self.config_error=None
        try: self.config=NCConfig.load(self.config_path)
        except FileNotFoundError: self.config=NCConfig().validate()
        except Exception as e: self.config=NCConfig().validate(); self.config_error=str(e)
        self.link=None; self.state=None; self.rx=self.opened=0; self.job=0; self.active_job=None
        self.pending=None; self.commands=[]; self.last_keep=self.started=0; self.expect_epoch=None
        self.task_kind=None; self.event_log_path=self.config_path.with_name("cartesian_events.jsonl")
        self.trace=None; self.last_log_path=None; self.log_text=tk.StringVar(value="Log tự động: tạo file mỗi lần kết nối")
        self.local_reference=False; self.model=None; self.preview=None; self.settings_window=None
        self.planning=False; self.plan_results=queue.Queue(); self.plan_token=0
        self.active_purpose=None; self.follow_target=None; self.follow_revision=0; self.follow_due=0
        self.program_points=(); self.program_index=0; self.program_wait=0
        self.path_upload=None; self.path_run=None; self.path_since=0
        self.port=tk.StringVar(value="DEMO" if demo else "")
        self.xyz=[tk.StringVar(value=v) for v in ("-69.296","167.296","20.0")]
        self.speed=tk.StringVar(value="5")
        self.message=tk.StringVar(value="Kiểm tra thông số cơ khí và sáu công tắc, rồi bấm HOME + CALIB.")
        self.metric=tk.StringVar(value="CHƯA LẤY MỐC"); self.details=tk.StringVar(value="")
        self.progress=tk.StringVar(value=STAGES[0]); self.switch_text=tk.StringVar(value="Chưa có dữ liệu công tắc")
        self.input_details=tk.StringVar(value="")
        self.init_editor()
        self.init_drawing()
        self._build(); self.refresh_ports(); self.render()
        self.bind_all("<Escape>",lambda e:self.stop("ESC"))
        self.bind("<FocusOut>",lambda e:self.after(10,self._focus_check))
        self.bind("<Unmap>",lambda e:self._unmap_check() if e.widget is self else None)
        self.protocol("WM_DELETE_WINDOW",self.close); self.after(25,self.poll)
        if self.config_error: self.message.set("Cấu hình lỗi; khóa chuyển động: "+self.config_error)

    def label(self,parent,text=None,size=11,color=TEXT,**kw):
        return tk.Label(parent,text=text,bg=parent.cget("bg"),fg=color,font=("Segoe UI",size),**kw)
    def button(self,parent,text,command,color="#2b3c55",**kw):
        return tk.Button(parent,text=text,command=command,bg=color,fg=TEXT,
                         activebackground="#3a5b7e",activeforeground=TEXT,relief="flat",
                         font=("Segoe UI",11),padx=12,pady=4,**kw)
    def card(self,parent): return tk.Frame(parent,bg=CARD,padx=16,pady=12)
    def entry(self,parent,var):
        return tk.Entry(parent,textvariable=var,width=10,bg="#243651",fg=TEXT,
                        insertbackground=TEXT,relief="flat",font=("Segoe UI",12))
    def _build(self):
        from cartesian_nc_layout import build_dashboard
        build_dashboard(self)

    def trace_event(self,kind,**fields):
        if self.trace: self.trace.record(kind,**fields)
    def open_logs(self):
        root=self.config_path.parent/"logs"; root.mkdir(exist_ok=True)
        if sys.platform=="win32":
            import os
            os.startfile(root)

    def fresh(self): return self.link is not None and self.state is not None and self.state.session==self.link.session and time.monotonic()-self.rx<.8
    def idle(self): return self.fresh() and not self.state.busy and self.active_job is None and self.pending is None and not self.commands and not self.planning
    def operable(self): return self.idle() and not self.config_error and not self.state.fault and self.state.ready==7 and not self.state.conflicts and not self.state.errors
    def refresh_ports(self):
        if self.demo: ports=["DEMO"]
        else:
            try:
                from serial.tools import list_ports
                ports=sorted(p.device for p in list_ports.comports())
            except Exception as e: self.message.set(str(e)); ports=[]
        self.ports.configure(values=ports)
        if self.port.get() not in ports: self.port.set(ports[0] if ports else "")
    def toggle_connection(self):
        if self.link: self.disconnect("Đã ngắt USB."); return
        try:
            if not self.port.get(): raise ValueError("Chọn cổng COM.")
            self.link=self.factory(self.port.get()); self.opened=time.monotonic()
            try:
                self.trace=SessionLog(self.config_path.parent/"logs")
                self.last_log_path=self.trace.path
                self.log_text.set("Log: "+self.trace.path.name)
                self.trace_event("connection",port=self.port.get(),session=self.link.session,build=BUILD,
                    gui_version=GUI_VERSION,config=asdict(self.config),demo=self.demo,axis_order=["Z","J1","J2"])
                self.trace_event("startup",commands=["STOP",f"HELLO {self.link.session}"])
            except OSError as e:
                self.trace=None; self.log_text.set("Không tạo được log: "+str(e))
            self.state=None; self.rx=0; self.job=0; self.local_reference=False
            self.model=None; self.expect_epoch=None
            self.message.set("Đang xác nhận firmware NC v5…")
        except Exception as e: self.message.set(str(e))
        self.render()
    def send(self,line):
        self.trace_event("tx",command=line)
        try: self.link.send(line); return True
        except Exception as e: self.disconnect("Lỗi USB: "+str(e)); return False
    def disconnect(self,message,source="DISCONNECT"):
        self.stop(source); link,self.link=self.link,None
        if link: link.close()
        self.trace_event("disconnect",source=source,message=message)
        trace,self.trace=self.trace,None
        if trace:
            trace.close()
            if trace.error: self.log_text.set("Lỗi ghi log: "+trace.error)
        self.state=None; self.local_reference=False; self.model=None; self.message.set(message); self.render()
    def sequence(self,lines):
        self.commands=list(lines); self.next_command()
    def next_command(self):
        if not self.commands or not self.link: return
        line=self.commands.pop(0); self.pending=(line.split()[0],time.monotonic())
        if self.pending[0] in ("INIT","MOVE","PATH"):
            self.task_kind=self.pending[0]
            self.active_job=self.job; self.started=time.monotonic(); self.last_keep=self.started
        self.send(line)
    def initialize(self):
        if not self.operable() or self.state.holding!=7: return
        self.clear_automation()
        self.local_reference=False; self.model=None; self.preview=None
        self.expect_epoch=self.state.epoch+1; self.job=max(self.job,self.state.job)+1
        self.task_kind="INIT"
        self.trace_event("home_start",config=asdict(self.config),job=self.job)
        self.sequence(self.config.commands(self.link.session)+[f"INIT {self.link.session} {self.job}"])
        self.message.set("Robot sẽ chạy đến các công tắc để đo; Esc/Dừng hủy ngay. "+
                         ("Home tiếp tục khi đổi cửa sổ; biên và mất kết nối vẫn được bảo vệ." if self.config.home_in_background else "Giữ cửa sổ này ở phía trước."))
        self.render()
    def get_plan(self):
        if not self.fresh() or not self.local_reference or not self.model: raise ValueError("Cần HOME + CALIB trước.")
        return self.model.plan(self.state.pos,tuple(float(v.get()) for v in self.xyz),float(self.speed.get()))
    def preview_target(self):
        self.start_plan(False)
    def move(self):
        self.start_plan(True)
    def start_plan(self,moving,target=None,speed=None,purpose="manual",revision=0):
        if not self.operable() or not self.local_reference or not self.model: return
        try:
            target=tuple(float(v.get()) for v in self.xyz) if target is None else tuple(target)
            speed=float(self.speed.get()) if speed is None else speed
        except ValueError as e: self.message.set(str(e)); return
        if purpose=="manual": self.clear_automation()
        self.planning=True; self.plan_token+=1; token=self.plan_token
        model=self.model; start=self.state.pos; epoch=self.state.epoch
        def calculate():
            try: result=model.plan(start,target,speed)
            except Exception as e: result=e
            self.plan_results.put((token,epoch,start,model,moving,result,purpose,revision))
        threading.Thread(target=calculate,daemon=True,name="SCARA path validation").start()
        self.message.set("Đang kiểm tra đường đi và từng bước DDA…")
        self.render()
    def clear_automation(self):
        self.path_upload=None; self.path_run=None
        self.bk_active=False; self.bk_pending=False
        self.follow_enabled.set(False); self.follow_target=None; self.follow_revision+=1
        self.program_points=(); self.program_index=0; self.program_wait=0
    def toggle_follow(self):
        if not self.follow_enabled.get():
            self.follow_target=None; self.follow_revision+=1
            if self.active_purpose=="follow": self.stop()
            return
        if not self.operable() or not self.local_reference or self.program_points:
            self.follow_enabled.set(False); self.message.set("Cần lấy mốc và dừng trước khi bật bám mô hình."); return
        self.message.set("Bám mô hình đã bật. Kéo đầu công tác, khuỷu hoặc thanh trượt để điều khiển.")
        self.trace_event("follow_enabled")
    def start_program(self):
        if not self.operable() or not self.local_reference or not self.points or self.program_points: return
        self.clear_automation(); self.planning=True; self.plan_token+=1; token=self.plan_token
        points=tuple(self.points); model=self.model; start=self.state.pos; epoch=self.state.epoch
        def calculate():
            try: validate_program(model,start,points,lambda:token!=self.plan_token); result=points
            except Exception as e: result=e
            self.plan_results.put((token,epoch,start,model,False,result,"program_check",0))
        threading.Thread(target=calculate,daemon=True,name="SCARA setpoint validation").start()
        self.message.set(f"Đang kiểm tra toàn bộ{len(points)} điểm và đường đi…")
    def finish_program_point(self,now):
        point=self.program_points[self.program_index]
        drawing=self.bk_active; mode=self.bk_active_plan.dry if drawing else False
        self.record_bk(self.state,True)
        self.trace_event("setpoint_complete",index=self.program_index,point=asdict(point),status=asdict(self.state))
        self.program_index+=1; self.program_wait=now+point.dwell_s
        if self.program_index==len(self.program_points):
            self.program_points=(); self.message.set("Đã hoàn thành chuỗi setpoint.")
            if drawing:
                self.bk_active=False
                pattern=PATTERNS[self.bk_active_plan.settings.pattern][0]
                message=f"Đã chạy khô {pattern}; bút ở Z nâng." if mode else f"Đã vẽ {pattern} theo xung; bút đã nâng."
                self.message.set(message); self.bk_note.set(message)
                elapsed=now-self.bk_started
                self.bk_note.set(message+f" Tổng thời gian {elapsed:.1f}s.")
                self.trace_event("drawing_complete",dry=mode,elapsed_s=elapsed,status=asdict(self.state))
        else: self.message.set(f"Đã đến{point.name}; nghỉ{point.dwell_s:g}s trước điểm tiếp theo.")
    def automation_tick(self,now):
        if not self.editor_ready():
            if self.follow_enabled.get() or self.program_points: self.clear_automation()
            return
        if not self.operable(): return
        if self.program_points:
            if now>=self.program_wait:
                if self.bk_active:
                    run=next((r for r in self.bk_active_plan.runs if r.first==self.program_index),None)
                    if run is not None:
                        self.start_stroke(run,now); return
                point=self.program_points[self.program_index]
                self.set_target_pose(self.model.ik(point.xyz,self.model.from_steps(self.state.pos)))
                self.start_plan(True,target=point.xyz,speed=point.speed_mm_s,purpose="program")
            return
        if self.follow_enabled.get() and self.follow_target is not None and now>=self.follow_due:
            if not self._foreground(): self.focus_lost("FOCUS"); return
            try:
                q0=self.model.from_steps(self.state.pos); q1=self.model.ik(self.follow_target,q0)
                if self.model.to_steps(q1)==self.state.pos:
                    self.follow_target=None; self.message.set("Đã bám đến đích mô hình."); return
                segment=bounded_pose(q0,q1)
                self.start_plan(True,target=self.model.fk(segment),purpose="follow",revision=self.follow_revision)
            except ValueError as e: self.follow_target=None; self.message.set(str(e))
    def start_stroke(self,run,now):
        if self.state.pos!=run.start_steps:
            self.clear_automation(); self.message.set("Vị trí đầu nét đã đổi; đã hủy đường vẽ."); return
        self.job=max(self.job,self.state.job)+1
        self.path_run=run; self.path_since=now
        self.path_upload=PathUpload(self.link.session,self.job,run.segments)
        self.path_upload.pending="PATH"; self.active_purpose="stroke"
        self.sequence([f"PATH {self.link.session} {self.job} {len(run.segments)}"])
        self.message.set(f"Vẽ nét liên tục: {len(run.segments)} đoạn; dự kiến {run.seconds:.1f}s theo xung.")
        self.trace_event("stroke_start",job=self.job,first=run.first,end=run.end,
            estimated_seconds=run.seconds,profiles=[asdict(s) for s in run.segments])
    def path_tick(self,now):
        if not self.path_upload or self.pending or not self.link: return
        line=self.path_upload.next()
        if line:
            self.pending=(line.split()[0],now); self.send(line)
    def finish_stroke(self,now):
        run=self.path_run
        self.trace_event("stroke_complete",job=self.state.job,elapsed_s=now-self.path_since,
            estimated_s=run.seconds,segments=len(run.segments),timer_late=self.state.timer_late,
            starved=self.state.starved,pos=self.state.pos)
        self.path_run=self.path_upload=None; self.pending=None
        self.program_index=run.end-1; self.finish_program_point(now)
    def current_target(self):
        if self.fresh() and self.local_reference and self.model:
            self.follow_target=None; self.follow_revision+=1
            self.sync_editor()
    def hold(self,value):
        if not self.idle() or self.state.fault: return
        self.clear_automation()
        self.local_reference=False; self.model=None; self.expect_epoch=None
        self.sequence([f"HOLD {self.link.session} J1 {int(value)}",f"HOLD {self.link.session} J2 {int(value)}"])
        self.message.set("Cần HOME + CALIB lại sau khi thay đổi lực giữ."); self.render()
    def stop(self,source="USER"):
        self.plan_token+=1; self.planning=False
        self.clear_automation(); self.active_purpose=None
        was_busy=self.active_job is not None or (self.state is not None and self.state.busy)
        context={"type":"stop_sent","source":source,"demo":self.demo,
                 "elapsed_s":round(time.monotonic()-self.opened,3) if self.opened else 0,
                 "task":self.task_kind,"job":self.active_job,
                 "firmware_reason":self.state.reason if self.state else None,
                 "input_axis":self.state.input_axis if self.state else None,
                 "input_bits":self.state.input_bits if self.state else None,
                 "data_age_s":round(time.monotonic()-self.rx,3) if self.rx else None}
        self.commands=[]; self.pending=None; self.active_job=None; self.expect_epoch=None
        self.task_kind=None
        if self.link:
            self.trace_event("stop_sent",context=context,status=asdict(self.state) if self.state else None)
            self.trace_event("tx",command="STOP "+source)
            try: self.link.send("STOP "+source)
            except Exception: pass
            if not self.demo:
                try:
                    with self.event_log_path.open("a",encoding="utf-8") as f: f.write(json.dumps(context,ensure_ascii=False)+"\n")
                except OSError: pass
        if was_busy: self.local_reference=False; self.model=None
        self.message.set(REASONS.get(STOP_REASONS.get(source,"stopped"),"Đã gửi DỪNG")+
                         ("; cần HOME + CALIB lại." if was_busy else ".")); self.render()
    def _foreground(self):
        if sys.platform=="win32":
            try:
                user32=ctypes.windll.user32
                ancestor=user32.GetAncestor; ancestor.argtypes=(ctypes.c_void_p,ctypes.c_uint); ancestor.restype=ctypes.c_void_p
                foreground=user32.GetForegroundWindow; foreground.restype=ctypes.c_void_p
                main=ancestor(self.winfo_id(),2); active=foreground()
                return bool(main and active and ancestor(main,3)==ancestor(active,3))
            except (OSError,AttributeError): pass
        return self.focus_displayof() is not None
    def _active_task(self):
        return self.active_job is not None or bool(self.state and self.state.busy) or bool(self.pending or self.commands) or self.planning or bool(self.program_points) or self.follow_enabled.get()
    def _home_background(self):
        return self.task_kind=="INIT" and self.config.home_in_background
    def background_allowed(self):
        if self.task_kind=="INIT": return self.config.home_in_background
        return self.config.motion_in_background
    def disarm_follow(self):
        if self.follow_enabled.get() or self.follow_target is not None:
            self.follow_enabled.set(False); self.follow_target=None; self.follow_revision+=1
            self.trace_event("follow_disarmed",reason="focus_lost",job=self.active_job)
            self.message.set("Đã tắt bám khi chuyển app; đoạn đang chạy sẽ hoàn thành, giữ mốc calib.")
    def focus_lost(self,source):
        if not self._active_task(): return
        if not self.background_allowed(): self.stop(source); return
        self.disarm_follow()
    def _focus_check(self):
        try:
            if not self._foreground(): self.focus_lost("FOCUS")
        except tk.TclError: pass
    def _unmap_check(self):
        self.focus_lost("MINIMIZE")

    def poll(self):
        now=time.monotonic()
        if self.link:
            for _ in range(128):
                try: event=self.link.events.get_nowait()
                except queue.Empty: break
                if event[0]=="error": self.disconnect("Lỗi USB: "+event[1]); break
                if event[0]=="rx_rejected":
                    self.trace_event("rx_rejected",received_monotonic=event[1],raw=event[2]); continue
                if event[0]=="ack":
                    ack=event[2]
                    self.trace_event("ack",received_monotonic=event[1],ack=asdict(ack))
                    if ack.session!=self.link.session: continue
                    if self.path_upload and ack.op in ("PATH","SEG","GO"):
                        try:
                            if self.path_upload.ack(ack): self.pending=None
                        except ValueError as e:
                            self.stop("MODEL"); self.message.set("Đường vẽ bị từ chối: "+str(e))
                        continue
                    if self.pending and ack.op==self.pending[0]:
                        self.pending=None
                        if not ack.ok:
                            self.commands=[]; self.active_job=None; self.local_reference=False; self.expect_epoch=None; self.task_kind=None
                            self.message.set("STM32 từ chối: "+REASONS.get(ack.reason,ack.reason))
                        else: self.next_command()
                    continue
                s=event[2]
                self.trace_event("status",received_monotonic=event[1],status=asdict(s))
                if s.session!=self.link.session: continue
                self.state=s; self.rx=event[1]; self.job=max(self.job,s.job)
                if self.path_upload: self.path_upload.status(s)
                self.record_bk(s)
                if s.fault or s.conflicts or not s.referenced:
                    self.local_reference=False
                if self.expect_epoch is not None and s.referenced and s.epoch==self.expect_epoch and not s.busy:
                    try:
                        self.model=self.config.model(s); self.local_reference=True; self.expect_epoch=None
                        self.current_target(); self.message.set("HOME + CALIB xong; có thể nhập XYZ.")
                    except Exception as e: self.stop("MODEL"); self.message.set(str(e))
                if self.local_reference and (not s.referenced or not self.model): self.local_reference=False
                if self.active_job is not None and s.job==self.active_job and not s.busy:
                    purpose=self.active_purpose; self.active_purpose=None
                    self.active_job=None
                    self.task_kind=None
                    if s.stage==9 or s.fault:
                        self.clear_automation()
                        self.expect_epoch=None; self.message.set(REASONS.get(s.reason,s.reason))
                    elif s.reason=="complete":
                        if purpose=="stroke" and s.referenced and self.path_run: self.finish_stroke(now)
                        elif purpose=="program" and s.referenced and self.program_points: self.finish_program_point(now)
                        else: self.message.set("Đã đến đích theo số xung; vị trí thực chưa có encoder xác nhận.")
                    else:
                        if self.bk_active: self.bk_note.set(REASONS.get(s.reason,s.reason))
                        self.clear_automation(); self.message.set(REASONS.get(s.reason,s.reason))
            if self.link:
                if (self.state is None and now-self.opened>3) or (self.state is not None and now-self.rx>.8):
                    self.disconnect("USB/status quá hạn; đã gửi DỪNG. Kiểm tra đúng firmware NC v5 R9.","STALE")
                elif self.pending and now-self.pending[1]>1:
                    self.stop("ACK"); self.message.set("Lệnh không được STM32 xác nhận; đã dừng.")
                elif self.active_job is not None and now-self.last_keep>=.08:
                    self.last_keep=now; self.send(f"KEEP {self.link.session} {self.active_job}")
        while not self.plan_results.empty():
            token,epoch,start,model,moving,result,purpose,revision=self.plan_results.get_nowait()
            if token!=self.plan_token: continue
            self.planning=False
            if purpose=="drawing_check": self.bk_pending=False
            if not self.operable() or not self.local_reference or self.model!=model or self.state.epoch!=epoch or self.state.pos!=start:
                self.message.set("Mốc hoặc vị trí đã đổi trong khi tính đường đi; hãy thử lại."); continue
            if purpose=="follow" and (not self.follow_enabled.get() or revision!=self.follow_revision): continue
            if purpose=="program" and not self.program_points: continue
            if isinstance(result,Exception):
                self.preview=None; self.message.set(str(result))
                if purpose in ("follow","program","program_check","drawing_check"): self.clear_automation()
                if purpose in ("drawing_check","drawing_fit"): self.bk_note.set(str(result))
                continue
            if purpose=="drawing_check": self.accept_bk(result); continue
            if purpose=="drawing_fit": self.accept_fit(result); continue
            if purpose=="program_check":
                self.program_points=result; self.program_index=0; self.program_wait=now
                self.trace_event("program_start",points=[asdict(p) for p in result]); continue
            self.preview=result
            if purpose=="manual": self.set_target_pose(result.joints)
            if moving:
                if not self._foreground():
                    if purpose=="follow": self.focus_lost("FOCUS"); continue
                    if not self.background_allowed(): self.stop("FOCUS"); continue
                if result.steps==self.state.pos:
                    if purpose=="program": self.finish_program_point(now)
                    elif purpose=="follow": self.follow_target=None
                    else: self.message.set("Đích đã trùng vị trí hiện tại.")
                    continue
                self.active_purpose=purpose
                self.job=max(self.job,self.state.job)+1
                self.sequence([f"MOVE {self.link.session} {self.job} "+" ".join(map(str,result.steps))+f" {result.rate}"])
                if purpose=="program":
                    point=self.program_points[self.program_index]
                    self.message.set(f"Đang tới {point.name} ({self.program_index+1}/{len(self.program_points)})…")
                    if not self.bk_active and self.points_window and self.points_window.winfo_exists() and self.points_tree.exists(str(self.program_index)):
                        self.points_tree.selection_set(str(self.program_index)); self.points_tree.see(str(self.program_index))
                else: self.message.set("Đang gửi chuyển động đã kiểm tra giới hạn.")
            else: self.message.set(f"Đích khớp: J1={result.joints[0]:.2f}°, J2={result.joints[1]:.2f}°; nhịp ≤{result.rate} xung/s. Đường đi có thể cong.")
        self.automation_tick(now)
        self.path_tick(now)
        self.render(); self.after(10 if self.path_upload else 25,self.poll)
        if self.trace and self.trace.error: self.log_text.set("Lỗi ghi log: "+self.trace.error)

    def render(self):
        if not hasattr(self,"home_btn"): return
        fresh=self.fresh(); idle=self.idle(); operate=self.operable()
        self.connect_btn.configure(text="Ngắt kết nối" if self.link else "Kết nối")
        if hasattr(self,"home_hint"):
            self.home_hint.configure(text=f"Z quét {self.config.z_home_speed_mm_s:g} mm/s ≈{self.config.z_home_pps:,} xung/s · chốt {self.config.z_latch_pps:,}\n"+
                f"J2 quét {self.config.j2_home_pps:,} · chốt {self.config.j2_latch_pps:,} xung/s")
        self.badge.configure(text="● ĐÃ KẾT NỐI" if fresh else "● CHƯA KẾT NỐI",fg=GREEN if fresh else MUTED)
        autonomous=bool(self.program_points) or self.follow_enabled.get()
        self.settings_btn.configure(state="normal" if not autonomous and self.active_job is None and not (self.state and self.state.busy) and not self.pending and not self.planning else "disabled")
        self.home_btn.configure(state="normal" if operate and not autonomous and self.state.holding==7 else "disabled")
        for b in (self.preview_btn,self.move_btn,self.current_btn): b.configure(state="normal" if operate and self.local_reference and not self.program_points else "disabled")
        for b in (self.release_btn,self.hold_btn): b.configure(state="normal" if idle and not autonomous and not self.state.fault else "disabled")
        self.render_editor()
        self.render_drawing()
        if self.state:
            s=self.state
            pins=("Z PA3/PA4","J1 PB0/PB1","J2 PB10/PB11")
            text=[]
            for a,p in enumerate(pins):
                bits=(s.switches>>(2*a))&3
                text.append(f"{p}: +{'CHẠM/HỞ' if bits&1 else 'đóng'} / −{'CHẠM/HỞ' if bits&2 else 'đóng'}")
                if s.conflicts & (1<<a): text[-1]+=" (lỗi đã giữ)"
                if s.errors & (1<<a): text[-1]+=" (GPIO lỗi)"
            self.switch_text.set("   ·   ".join(text))
            progress=STAGES[s.stage]
            self.progress_label.configure(height=1 if s.stage==8 else 3)
            if s.stage==8 and s.busy: progress="Đang di chuyển"
            if self.program_points:
                progress=f"Chuỗi {self.program_index+1}/{len(self.program_points)}: {self.program_points[self.program_index].name}"
            if self.path_upload and s.job==self.path_upload.job:
                total,received,done,free=s.path
                progress=f"Nét {self.bk_active_plan.operations[self.program_index].stroke+1}: {done}/{total} đoạn · {s.pps} xung/s"
                self.bk_note.set(f"{PATTERNS[self.bk_active_plan.settings.pattern][0]} · {time.monotonic()-self.bk_started:.1f}s\nNét {done}/{total} đoạn; bộ đệm {32-free}/32 · nạp {received}/{total}\nSai lệch mô hình ≤{self.bk_active_plan.max_model_error_mm:.3f}mm; vị trí theo xung.")
            if s.stage==8 and not s.referenced: progress="Mất mốc · cần HOME + CALIB"
            if s.busy and s.stage in (1,2,4,6,11):
                a=0 if s.stage==1 else 1 if s.stage==4 else 2
                progress+="\n"+(Z_PHASES if a==0 else PHASES)[s.phase[a]]
                progress+=f"\nĐo đi/về: {s.n1[a]} / {s.n2[a]} xung"
            if s.stage==9:
                errors=[f"{a}: {ERRORS[e] if e<len(ERRORS) else e} (pha {(Z_PHASES if i==0 else PHASES)[s.failed_phase[i]]})" for i,(a,e) in enumerate(zip(("Z","J1","J2"),s.cal_error)) if e]
                progress+="\n"+REASONS.get(s.reason,s.reason)+"\n"+"; ".join(errors)
            if s.input_wait: progress="TẠM DỪNG XUNG\nĐang xác nhận công tắc\n"+STAGES[s.stage]
            if s.gate_wait: progress=f"Đợi chân {('Z','J1','J2')[s.gate_axis]} ổn định\nTrước khi chạy trục tiếp theo"
            self.progress.set(progress)
            self.fw_label.configure(text=f"{s.build} · N[Z,J1,J2]={s.range} · timer trễ={s.timer_late} · GPIO lỗi={s.motor_error}\n"+
                f"Bù B/A={s.beta_q/1048576:+.5f} xung/xung ({'đã đo' if s.coupling_ready else 'chưa xác nhận'}) · thử ΔA/ΔB={s.probe_da}/{s.probe_db} · k={s.coupling_ppm/1000000:.6f} · DIR[Z,J1,J2]={s.dir_levels&1}/{(s.dir_levels>>1)&1}/{(s.dir_levels>>2)&1}")
            if s.input_axis>=0 and s.input_kind:
                pairs=(("Z","PA3","PA4"),("J1","PB0","PB1"),("J2","PB10","PB11"))
                axis,p,n=pairs[s.input_axis]; bits=(s.input_bits>>(2*s.input_axis))&3
                event={1:"đã từng đọc HIGH/HIGH",2:"HIGH/HIGH đã giữ lỗi",3:"tín hiệu dao động quá lâu",4:"lỗi đọc GPIO",5:"một biên mở nhưng không xác nhận ổn định",6:"xung thoáng qua ở trục đứng yên đã lọc",7:"tín hiệu chưa an toàn để chạy trục"}[s.input_kind]
                current=(s.raw>>(2*s.input_axis))&3
                self.input_details.set(f"Nguồn sự kiện: {axis} · {p}/{n}={bits&1}/{(bits>>1)&1} lúc {s.input_at} ms; hiện tại {current&1}/{(current>>1)&1}. "+
                    f"{event} · tạm dừng phục hồi: {s.recovered} · đã lọc khi đứng yên: {s.idle_ignored} · mở giả [Z,J1,J2]: {s.false_hits}")
                fault_stop=s.reason in ("both_open","both_latched","switch_unstable","calibration_failed","switch_io","j2_guard","edge_limit")
                self.input_label.configure(fg=RED if s.errors or s.conflicts or (s.stage==9 and fault_stop) or s.input_kind in (2,3,4,5,7) else GREEN)
            else: self.input_details.set("")
            if self.local_reference and self.model:
                q=self.model.from_steps(s.pos); xyz=self.model.fk(q)
                self.metric.set("X {:+.2f}    Y {:+.2f}    Z {:.2f} mm".format(*xyz))
                self.details.set(f"J1 {q[0]:+.2f}° · J2 tương đối {q[1]:+.2f}° · Xung theo mốc: {s.pos}\nTổng xung phát [Z,J1,J2]: {s.total}\n"+
                                 f"Z: 0–{s.range[0]/self.model.factors[0]:.2f} mm "+("đã nhập hành trình thực" if self.config.z_span_mm else "ước tính; hãy đo và nhập hành trình thực")+
                                 " · Vị trí theo xung, chưa có encoder.")
            else:
                self.metric.set("ĐANG LẤY MỐC" if s.busy and s.stage not in (8,9) else "CHƯA LẤY MỐC")
                self.details.set("Tọa độ XYZ chỉ có hiệu lực sau khi cả ba trục hoàn thành khởi tạo.\n"+
                                 f"Tổng xung phát [Z,J1,J2]: {s.total}\n"+REASONS.get(s.reason,s.reason))
        from cartesian_nc_layout import update_dashboard
        update_dashboard(self)
        self.draw()

    def draw(self):
        self.draw_workspace()

    def settings(self):
        if self.settings_window and self.settings_window.winfo_exists(): self.settings_window.lift(); return
        win=tk.Toplevel(self); self.settings_window=win; win.title("Thông số SCARA / NC"); win.geometry("790x650"); win.configure(bg=BG)
        self.label(win,"THÔNG SỐ CƠ KHÍ",17).pack(anchor="w",padx=18,pady=(12,4))
        self.label(win,"l1/l2 phải đo giữa các tâm khớp/điểm gá; tool_offset_mm là độ lệch đầu bút tại gá arm 2.\nOffset [+X sang phải, +Y dọc arm 2] khi hai arm thẳng hướng +Y; chưa đo giữ [0,0].\nGóc giữa công tắc phải đo thật; 180° là giả định. Z chưa đo giữ z_span_mm=0.",10,MUTED,justify="left").pack(anchor="w",padx=18,pady=(0,8))
        text=tk.Text(win,bg=CARD,fg=TEXT,insertbackground=TEXT,font=("Consolas",11),wrap="none",undo=True)
        text.pack(fill="both",expand=True,padx=18,pady=4); text.insert("1.0",json.dumps(asdict(self.config),indent=2,ensure_ascii=False))
        note=tk.StringVar(); self.label(win,textvariable=note,color=RED,wraplength=750).pack(fill="x",padx=18)
        def save():
            try:
                if not self.idle() and self.link: raise ValueError("Đang xử lý tác vụ; dừng trước khi đổi thông số.")
                cfg=NCConfig.from_dict(json.loads(text.get("1.0","end"))); cfg.save(self.config_path)
                self.config=cfg; self.config_error=None; self.local_reference=False; self.model=None; self.preview=None
                self.message.set("Đã lưu thông số. Bấm HOME + CALIB để áp dụng và lấy lại mốc.")
                win.destroy(); self.settings_window=None; self.render()
            except Exception as e: note.set(str(e))
        self.button(win,"Lưu thông số",save,"#1c6b58").pack(pady=(4,14))
    def close(self):
        self.disconnect("Đã đóng.","CLOSE"); self.destroy()

def main():
    p=argparse.ArgumentParser(); p.add_argument("--demo",action="store_true"); p.add_argument("--smoke-test",action="store_true")
    args=p.parse_args()
    if args.demo or args.smoke_test:
        from cartesian_nc_demo import DemoLink
        app=NCApp(factory=DemoLink,demo=True)
    else: app=NCApp()
    if args.smoke_test:
        app.after(100,app.toggle_connection); app.after(400,app.initialize); app.after(3000,app.close)
    app.mainloop()
if __name__=="__main__": main()



