"""Exercise model dragging, latest-target following and setpoint automation offline."""
from pathlib import Path
import sys
import time
import tempfile
import ctypes
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from cartesian_nc_control import NCApp
from cartesian_nc_demo import DemoLink
from cartesian_nc_model import NCConfig
from cartesian_nc_workspace import Setpoint

class RecordingDemo(DemoLink):
    def __init__(self,port):
        self.sent=[]; self.freeze_motion=False; self.silent=False
        super().__init__(port)
    def send(self,line):
        if line.startswith("MOVE "): assert not self.busy,"Overlapping MCU MOVE commands"
        self.sent.append((time.monotonic(),line)); super().send(line)
    def advance(self):
        if self.freeze_motion and self.busy and not self.initializing: self.ends=time.monotonic()+1
        super().advance()
    def emit(self):
        if not self.silent: super().emit()

base=Path(__file__).resolve().parents[2]/"Firmware/ScaraCartesian/build_host"
temp=tempfile.TemporaryDirectory(dir=base)
assert Path(temp.name).resolve().is_relative_to(base.resolve())
path=Path(temp.name)/"config.json"; NCConfig().save(path)
app=NCApp(factory=RecordingDemo,config_path=path,demo=True)
errors=[]; app.report_callback_exception=lambda *e:errors.append(e)
app._foreground=lambda:True

def pump(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        app.update(); time.sleep(.005)
    assert not errors,errors
def until(condition,seconds=5):
    end=time.monotonic()+seconds
    while not condition() and time.monotonic()<end: pump(.02)
    assert condition(),(app.message.get(),app.state)
def moves(): return [(t,line) for t,line in app.link.sent if line.startswith("MOVE ")]
def home():
    app.initialize(); until(lambda:app.local_reference and not app.planning,3)
    app.current_target(); pump(.05)

try:
    app.toggle_connection(); pump(.2); home()
    assert "0" in [app.canvas.itemcget(i,"text") for i in app.canvas.find_all() if app.canvas.type(i)=="text"]
    assert any("+X (mm)"==app.canvas.itemcget(i,"text") for i in app.canvas.find_all() if app.canvas.type(i)=="text")
    before=len(moves()); z=app.model.from_steps(app.state.pos)[2]
    px,py=app.view.point(0,170)
    app.canvas_press(SimpleNamespace(x=px,y=py)); app.canvas_release(SimpleNamespace(x=px,y=py)); pump(.15)
    assert abs(float(app.xyz[0].get()))<.001 and abs(float(app.xyz[1].get())-170)<.001
    assert len(moves())==before # editing with follow OFF cannot move the robot
    app.current_target(); pump(.05)
    px,py=app.view.point(0,0)
    app.canvas_press(SimpleNamespace(x=px,y=py)); app.canvas_release(SimpleNamespace(x=px,y=py)); pump(.1)
    assert len(moves())==before and app.local_reference
    # The latest target replaces older mouse positions while one MOVE drains.
    app.current_target(); app.follow_enabled.set(True); app.toggle_follow()
    initial=app.model.from_steps(app.state.pos); app.link.freeze_motion=True
    app.set_target_pose((4,49,z),True); app.release_pose(); until(lambda:app.state.busy,2)
    assert len(moves())==before+1
    app.set_target_pose((6,51,z),True); app.set_target_pose((-4,49,z),True); app.release_pose()
    pump(.25); assert len(moves())==before+1
    app.link.freeze_motion=False
    until(lambda:app.follow_target is None and not app.state.busy and not app.planning,6)
    q=app.model.from_steps(app.state.pos)
    assert abs(q[0]+4)<.1 and abs(q[1]-49)<.1
    previous=initial
    for _,line in moves()[before:]:
        next_q=app.model.from_steps(tuple(map(int,line.split()[3:6])))
        assert all(abs(b-a)<=limit for a,b,limit in zip(previous,next_q,(2.12,2.12,.501)))
        previous=next_q
    app.follow_enabled.set(False); app.toggle_follow(); assert app.local_reference
    # Reject the entire program before its first MOVE if a later point is bad.
    before=len(moves()); app.points=[Setpoint("valid",*app.model.fk((0,45,z)),10),Setpoint("bad",0,0,z,10)]
    app.start_program(); until(lambda:not app.planning,2); pump(.1)
    assert len(moves())==before and not app.program_points and app.local_reference
    # Run two checked targets in order, observing the dwell between them.
    app.points=[Setpoint("A",*app.model.fk((0,45,z)),10,.2),Setpoint("B",*app.model.fk((4,49,z)),10,.2)]
    app.start_program(); until(lambda:not app.program_points and not app.planning and not app.state.busy,5)
    program_moves=moves()[before:]; assert len(program_moves)==2,(program_moves,app.message.get(),app.program_index,app.state.pos)
    assert program_moves[1][0]-program_moves[0][0]>=.65
    assert "hoàn thành chuỗi" in app.message.get()
    # Cancellation during planning invalidates the result and issues no MOVE.
    before=len(moves()); app.start_program(); app.stop(); pump(.3)
    assert not app.program_points and len(moves())==before and app.local_reference
    # Default background mode continues a checked program across dwell.
    current=app.model.fk(app.model.from_steps(app.state.pos))
    app.points=[Setpoint("same",*current,5,1),Setpoint("next",*app.model.fk((8,50,z)),5,0)]
    app.start_program(); until(lambda:bool(app.program_points) and app.program_index==1,2)
    before=len(moves()); app._foreground=lambda:False; app._focus_check(); pump(.1)
    assert app.program_points and len(moves())==before and app.local_reference
    until(lambda:not app.program_points and not app.state.busy,3)
    assert len(moves())==before+1 and app.local_reference
    app._foreground=lambda:True
    # STOP during a running program cannot issue the remaining targets later.
    app.points=[Setpoint("first",*app.model.fk((8,50,z))),Setpoint("later",*app.model.fk((12,55,z)))]
    app.start_program(); until(lambda:app.state.busy,2); before=len(moves())
    app.stop(); pump(.8); assert not app.program_points and len(moves())==before and not app.local_reference
    home()
    # Numeric sliders and ghost controls must fit inside the notebook tab.
    app.geometry("1080x900"); app.workspace_tabs.select(1); pump(.1)
    parent=app.follow_checkbox.master
    assert app.follow_checkbox.winfo_rooty()+app.follow_checkbox.winfo_height()<=parent.winfo_rooty()+parent.winfo_height()
    assert app.points_btn.winfo_rooty()+app.points_btn.winfo_height()<=app.points_btn.master.winfo_rooty()+app.points_btn.master.winfo_height()
    app.points=[]; app.open_points(); pump(.1); assert len(app.points_tree.get_children())==9
    assert str(app.points_window.transient())==str(app) # owned dialog is allowed by native focus policy
    try:
        from PIL import ImageGrab
        hwnd=ctypes.windll.user32.GetAncestor(app.points_window.winfo_id(),2)
        ImageGrab.grab(window=hwnd).save(base/"cartesian_nc_R8_setpoints_preview.png")
        app.points_window.withdraw(); app.focus_force(); pump(.1)
        hwnd=ctypes.windll.user32.GetAncestor(app.winfo_id(),2)
        ImageGrab.grab(window=hwnd).save(base/"cartesian_nc_R8_model_preview.png")
    except (ImportError,OSError) as e: print("Optional own-window screenshot unavailable:",e)
    # Losing status while following stops/disconnects and clears automatic jobs.
    app.follow_enabled.set(True); app.toggle_follow(); app.link.freeze_motion=True
    app.set_target_pose((4,49,z),True); app.release_pose(); until(lambda:app.state.busy,2)
    app.link.silent=True; until(lambda:app.link is None,2)
    assert not app.program_points and not app.follow_enabled.get() and not app.local_reference
    print("R8 GUI: numeric axes, ghost drag, latest bounded follow, ordered/dwelled CSV setpoints, invalid-leg rejection, focus/STOP/stale cancellation passed")
finally:
    app.close(); temp.cleanup()
