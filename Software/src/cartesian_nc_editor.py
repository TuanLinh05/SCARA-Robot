"""Tk workspace editor. Motion acceptance stays in NCApp's checked planner."""
import math
import time
import tkinter as tk
from tkinter import ttk, filedialog
from cartesian_nc_workspace import Viewport, Setpoint, pose_ranges, test_points, load_points, save_points

class WorkspaceEditor:
    def init_editor(self):
        self.target_pose=None; self.view=None; self.grid_key=None; self.point_key=None; self.drag_kind=None; self.editor_sync=False
        self.pose_values=[tk.DoubleVar(value=0) for _ in range(3)]
        self.pose_entries=[tk.StringVar(value="0") for _ in range(3)]
        self.pose_note=tk.StringVar(value="HOME + CALIB để chỉnh mô hình")
        self.cursor_note=tk.StringVar(value="Kéo đầu công tác chọn XY · kéo khuỷu chọn góc J1")
        self.follow_enabled=tk.BooleanVar(value=False)
        self.points=[]; self.points_window=None; self.show_points=tk.BooleanVar(value=False)
        self.pose_scales=[]; self.pose_widgets=[]; self.point_buttons=[]

    def build_workspace_tabs(self,right):
        self.points_btn=self.button(right,"Điểm thử / Chuỗi setpoint",self.open_points,"#394771")
        self.points_btn.pack(side="bottom",fill="x",pady=(4,0))
        tabs=ttk.Notebook(right,style="Scara.TNotebook",width=300); tabs.pack(fill="both",expand=True,pady=(4,0))
        xyz=tk.Frame(tabs,bg=right.cget("bg")); pose=tk.Frame(tabs,bg=right.cget("bg"))
        tabs.add(xyz,text="Nhập XYZ"); tabs.add(pose,text="Chỉnh mô hình")
        self.label(pose,"J2 là góc giữa hai arm; góc dương về −X.",9,"#9cb0cb",wraplength=300).pack(anchor="w",pady=(6,4))
        for i,title in enumerate(("J1 (°)","Khuỷu J2 (°)","Z (mm)")):
            row=tk.Frame(pose,bg=pose.cget("bg")); row.pack(fill="x")
            self.label(row,title,size=10).pack(side="left")
            entry=self.entry(row,self.pose_entries[i]); entry.pack(side="right")
            entry.bind("<Return>",lambda event:self.commit_pose())
            scale=tk.Scale(pose,from_=-88,to=88,orient="horizontal",resolution=.01,showvalue=False,
                variable=self.pose_values[i],command=lambda value:self.slider_pose(),bg=pose.cget("bg"),
                fg="#edf3fc",troughcolor="#243651",highlightthickness=0,sliderlength=22)
            scale.pack(fill="x",pady=(0,6)); scale.bind("<ButtonRelease-1>",lambda event:self.release_pose())
            self.pose_scales.append(scale); self.pose_widgets.extend((entry,scale))
        row=tk.Frame(pose,bg=pose.cget("bg")); row.pack(fill="x",pady=2)
        self.label(row,"Tốc độ tối đa (mm/s)",size=10).pack(side="left")
        entry=self.entry(row,self.speed); entry.pack(side="right"); self.pose_widgets.append(entry)
        row=tk.Frame(pose,bg=pose.cget("bg")); row.pack(fill="x",pady=3)
        self.button(row,"Áp dụng",self.commit_pose).pack(side="left")
        self.button(row,"Gửi tư thế",self.move,"#245d9c").pack(side="right")
        self.follow_checkbox=tk.Checkbutton(pose,text="Robot bám theo mô hình",variable=self.follow_enabled,
            command=self.toggle_follow,bg=pose.cget("bg"),fg="#edf3fc",selectcolor="#243651",activebackground=pose.cget("bg"))
        self.follow_checkbox.pack(anchor="w",pady=4)
        self.workspace_tabs=tabs
        return xyz

    def bind_workspace(self):
        self.canvas.bind("<Motion>",self.canvas_hover)
        self.canvas.bind("<ButtonPress-1>",self.canvas_press)
        self.canvas.bind("<B1-Motion>",self.canvas_drag)
        self.canvas.bind("<ButtonRelease-1>",self.canvas_release)

    def editor_ready(self): return self.fresh() and self.local_reference and self.model is not None

    def set_target_pose(self,q,interactive=False):
        if not self.editor_ready(): return False
        try:
            self.model.check_pose(q)
            if q[1]*self.model.branch<=0: raise ValueError("Giữ nhánh khuỷu của lượt home hiện tại.")
            self.target_pose=tuple(q); xyz=self.model.fk(q)
            self.editor_sync=True
            for v,e,x in zip(self.pose_values,self.pose_entries,q): v.set(x); e.set(f"{x:.3f}")
            self.editor_sync=False
            for v,x in zip(self.xyz,xyz): v.set(f"{x:.3f}")
            self.pose_note.set(f"Đích X={xyz[0]:+.2f} · Y={xyz[1]:+.2f} · Z={xyz[2]:.3f} mm   |   J1={q[0]:+.2f}° · J2={q[1]:+.2f}°")
            if interactive and self.follow_enabled.get():
                self.follow_target=xyz; self.follow_revision+=1; self.follow_due=time.monotonic()+.2
                self.trace_event("model_target",xyz=xyz,joints=q,revision=self.follow_revision)
            self.draw(); return True
        except ValueError as e:
            self.editor_sync=False; self.pose_note.set(str(e)); self.message.set(str(e))
            if interactive: self.follow_target=None; self.follow_revision+=1
            return False

    def sync_editor(self):
        if self.editor_ready():
            self.editor_sync=True
            for scale,(lo,hi) in zip(self.pose_scales,pose_ranges(self.model)): scale.configure(from_=lo,to=hi)
            self.editor_sync=False
            self.set_target_pose(self.model.from_steps(self.state.pos))
            self.grid_key=None

    def slider_pose(self):
        if self.editor_sync or not self.editor_ready() or self.program_points: return
        self.set_target_pose(tuple(v.get() for v in self.pose_values),True)

    def commit_pose(self):
        if self.program_points: return
        try: q=tuple(float(v.get()) for v in self.pose_entries)
        except ValueError: self.pose_note.set("Góc và Z cần là số."); return
        self.set_target_pose(q,True)

    def release_pose(self):
        if self.follow_enabled.get() and self.follow_target: self.follow_due=time.monotonic()

    def canvas_hover(self,event):
        if self.view:
            x,y=self.view.world(event.x,event.y); self.cursor_note.set(f"Chuột X={x:+.1f} · Y={y:+.1f} mm   |   Kéo đầu công tác / khuỷu")

    def canvas_press(self,event):
        if not self.editor_ready() or self.program_points:
            self.message.set("HOME + CALIB để chỉnh mô hình."); return
        q=self.target_pose or self.model.from_steps(self.state.pos)
        elbows=[self.view.point(*self.model.elbow(p)) for p in (q,self.model.from_steps(self.state.pos))]
        self.drag_kind="j1" if any(math.hypot(event.x-ex,event.y-ey)<16 for ex,ey in elbows) else "xy"
        self.canvas_drag(event)

    def canvas_drag(self,event):
        self.canvas_hover(event)
        if not self.drag_kind or not self.editor_ready(): return
        x,y=self.view.world(event.x,event.y)
        old=self.target_pose or self.model.from_steps(self.state.pos)
        try:
            if self.drag_kind=="j1":
                if math.hypot(x,y)<1: return
                lo,hi=self.model.joint_limits_deg[0]
                q=(max(lo,min(hi,math.degrees(math.atan2(-x,y)))),old[1],old[2])
            else: q=self.model.ik((x,y,old[2]),old)
            self.set_target_pose(q,True)
        except ValueError as e:
            self.pose_note.set("Đích không hợp lệ: "+str(e))
            self.follow_target=None; self.follow_revision+=1

    def canvas_release(self,event):
        self.drag_kind=None; self.release_pose()

    def render_editor(self):
        enabled=self.editor_ready() and not self.program_points
        for widget in self.pose_widgets: widget.configure(state="normal" if enabled else "disabled")
        self.follow_checkbox.configure(state="normal" if enabled else "disabled")
        if not self.editor_ready(): self.target_pose=None
        if self.points_window and self.points_window.winfo_exists():
            editable=not self.program_points and not self.follow_enabled.get() and self.idle()
            for b in self.point_buttons: b.configure(state="normal" if editable else "disabled")

    def draw_workspace(self):
        if not hasattr(self,"canvas"): return
        c=self.canvas; w=max(c.winfo_width(),100); h=max(c.winfo_height(),100)
        reach=self.config.reach_mm; self.view=Viewport(w,h,reach); p=self.view.point
        bounds=pose_ranges(self.model)[:2] if self.editor_ready() else None
        key=(w,h,self.config.l1_mm,self.config.l2_mm,tuple(self.config.tool_offset_mm),bounds)
        if key!=self.grid_key:
            c.delete("all"); self.grid_key=key; self.point_key=None
            if bounds:
                (alo,ahi),(blo,bhi)=bounds
                outline=[]
                for a,b,da,db in ((alo,blo,ahi-alo,0),(ahi,blo,0,bhi-blo),(ahi,bhi,alo-ahi,0),(alo,bhi,0,blo-bhi)):
                    outline.extend(v for i in range(25) for v in p(*self.model.fk((a+da*i/24,b+db*i/24,1))[:2]))
                c.create_polygon(*outline,fill="#142d40",outline="#29526b",tags="grid")
            step=25 if self.view.scale>=.9 else 50
            limit=int(reach//step)*step
            for mm in range(-limit,limit+1,step):
                c.create_line(*p(mm,-reach),*p(mm,reach),fill="#26374d",tags="grid")
                c.create_line(*p(-reach,mm),*p(reach,mm),fill="#26374d",tags="grid")
                if mm:
                    x,y=p(mm,0); c.create_text(x,y+12,text=str(mm),fill="#9cb0cb",font=("Segoe UI",8),tags="grid")
                    x,y=p(0,mm); c.create_text(x-9,y,text=str(mm),fill="#9cb0cb",anchor="e",font=("Segoe UI",8),tags="grid")
            c.create_oval(*p(-reach,reach),*p(reach,-reach),outline="#44617c",dash=(3,4),tags="grid")
            c.create_line(*p(-reach,0),*p(reach,0),fill="#819ab7",arrow="last",width=2,tags="grid")
            c.create_line(*p(0,-reach),*p(0,reach),fill="#819ab7",arrow="last",width=2,tags="grid")
            x,y=p(reach,0); c.create_text(x,y-15,text="+X (mm)",fill="#c6d8ec",anchor="e",tags="grid")
            x,y=p(0,reach); c.create_text(x+9,y,text="+Y (mm)",fill="#c6d8ec",anchor="w",tags="grid")
            x,y=p(0,0); c.create_text(x+8,y+14,text="0",fill="#edf3fc",tags="grid")
        c.delete("dynamic")
        c.create_text(12,12,text="● Robot: xanh   ◇ Đích: vàng   Vùng với tới: nền xanh",anchor="nw",fill="#9cb0cb",font=("Segoe UI",9),tags="dynamic")
        if self.preview:
            coords=[v for x,y,z in self.preview.path for v in p(x,y)]
            if len(coords)>3: c.create_line(*coords,fill="#efc25a",width=2,dash=(3,3),tags="dynamic")
        self.draw_bk_overlay(c,p)
        point_key=(key,self.show_points.get(),tuple((point.x,point.y) for point in self.points))
        if point_key!=self.point_key:
            c.delete("waypoints"); self.point_key=point_key
        else: point_key=None
        if point_key is not None and self.show_points.get():
            for i,point in enumerate(self.points,1):
                x,y=p(point.x,point.y); c.create_oval(x-3,y-3,x+3,y+3,fill="#ad92ee",outline="",tags="waypoints")
                if self.view.scale>=1: c.create_text(x+5,y-8,text=str(i),anchor="w",fill="#ad92ee",font=("Segoe UI",8),tags="waypoints")
        if self.editor_ready():
            q=self.model.from_steps(self.state.pos)
            for pose,color,width,dash in ((self.target_pose,"#efc25a",3,(4,3)),(q,"#5aaeff",6,None)):
                if pose is None: continue
                p0=p(0,0); p1=p(*self.model.elbow(pose)); p2=p(*self.model.fk(pose)[:2])
                args={"fill":color,"width":width,"tags":"dynamic"}
                if dash: args["dash"]=dash
                c.create_line(*p0,*p1,**args); args["fill"]="#60d5ae" if pose is q else color
                c.create_line(*p1,*p2,**args)
                for pos in (p1,p2): c.create_oval(pos[0]-5,pos[1]-5,pos[0]+5,pos[1]+5,fill=args["fill"],outline="",tags="dynamic")
        else: c.create_text(w/2,h/2-24,text="HOME + CALIB để xác lập vị trí",fill="#9cb0cb",tags="dynamic")
        x,y=p(0,0); c.create_oval(x-4,y-4,x+4,y+4,fill="#edf3fc",outline="",tags="dynamic")

    def open_points(self):
        if self.points_window and self.points_window.winfo_exists(): self.points_window.lift(); return
        win=tk.Toplevel(self); self.points_window=win; win.transient(self)
        win.title("Điểm thử / Chuỗi setpoint · mm"); win.geometry("850x590")
        win.configure(bg="#172338"); self.point_buttons=[]
        self.label(win,"Chọn dòng để nhập đích. Chạy chuỗi sẽ kiểm tra tất cả đoạn trước khi bắt đầu.",10).pack(anchor="w",padx=12,pady=8)
        pane=tk.Frame(win,bg="#172338"); pane.pack(fill="both",expand=True,padx=12)
        tree=ttk.Treeview(pane,style="Scara.Treeview",columns=("name","x","y","z","speed","dwell"),show="headings",selectmode="browse")
        for column,title,width in zip(tree["columns"],("Tên","X (mm)","Y (mm)","Z (mm)","mm/s","Nghỉ (s)"),(130,100,100,100,90,85)):
            tree.heading(column,text=title); tree.column(column,width=width,anchor="center")
        scrollbar=ttk.Scrollbar(pane,orient="vertical",command=tree.yview)
        scrollbar.pack(side="right",fill="y"); tree.configure(yscrollcommand=scrollbar.set)
        tree.pack(side="left",fill="both",expand=True); self.points_tree=tree
        tree.bind("<<TreeviewSelect>>",lambda event:self.select_point())
        row=tk.Frame(win,bg="#172338"); row.pack(fill="x",padx=12,pady=8)
        for title,command in (("Tạo điểm thử",self.generate_points),("Thêm đích",self.add_point),("Sửa dòng",self.edit_point),
            ("Xóa",self.delete_point),("Nạp CSV",self.import_points),("Lưu CSV",self.export_points)):
            b=self.button(row,title,command); b.pack(side="left",padx=2); self.point_buttons.append(b)
        row=tk.Frame(win,bg="#172338"); row.pack(fill="x",padx=12,pady=(0,8))
        for title,command in (("Chạy điểm chọn",self.run_selected_point),("Chạy toàn bộ chuỗi",self.start_program)):
            b=self.button(row,title,command,"#245d9c"); b.pack(side="left",padx=2); self.point_buttons.append(b)
        self.button(row,"■ DỪNG",self.stop,"#90364c").pack(side="right")
        tk.Checkbutton(win,text="Hiện các điểm trên đồ thị",variable=self.show_points,command=self.draw,
            bg="#172338",fg="#edf3fc",selectcolor="#243651").pack(anchor="w",padx=12)
        self.label(win,"Điểm thử giữ Z hiện tại. Đường đi là nội suy khớp; quỹ đạo XY có thể cong.",9,"#9cb0cb").pack(anchor="w",padx=12,pady=(4,10))
        if not self.points and self.editor_ready(): self.generate_points()
        self.refresh_points(); self.render_editor()

    def refresh_points(self):
        if not self.points_window or not self.points_window.winfo_exists(): return
        self.points_tree.delete(*self.points_tree.get_children())
        for i,p in enumerate(self.points): self.points_tree.insert("", "end",iid=str(i),values=(p.name,*(f"{v:.3f}" for v in p.xyz),f"{p.speed_mm_s:g}",f"{p.dwell_s:g}"))
        self.draw()

    def selected_index(self):
        selection=self.points_tree.selection() if hasattr(self,"points_tree") else ()
        return int(selection[0]) if selection else None

    def select_point(self):
        i=self.selected_index()
        if i is None or self.program_points or self.planning or not self.editor_ready(): return
        self.follow_enabled.set(False); self.follow_target=None
        p=self.points[i]
        for v,x in zip(self.xyz,p.xyz): v.set(f"{x:.3f}")
        self.speed.set(str(p.speed_mm_s))
        try: self.set_target_pose(self.model.ik(p.xyz,self.model.from_steps(self.state.pos)))
        except ValueError as e: self.message.set(str(e))

    def generate_points(self):
        if not self.operable() or not self.editor_ready() or self.program_points: return
        try: self.points=test_points(self.model,self.model.from_steps(self.state.pos)[2],float(self.speed.get()))
        except (ValueError,AttributeError) as e: self.message.set(str(e)); return
        self.show_points.set(True); self.refresh_points()
        self.trace_event("test_points",points=[vars(p) for p in self.points])

    def add_point(self):
        if not self.operable() or self.program_points: return
        if len(self.points)>=500: self.message.set("Tối đa500 điểm."); return
        try: self.points.append(Setpoint(f"P{len(self.points)+1:02d}",*(float(v.get()) for v in self.xyz),float(self.speed.get())).validate())
        except ValueError as e: self.message.set(str(e)); return
        self.refresh_points()

    def edit_point(self):
        i=self.selected_index()
        if i is None or self.program_points: return
        p=self.points[i]; win=tk.Toplevel(self); win.title("Sửa setpoint"); win.configure(bg="#172338")
        win.transient(self.points_window); win.grab_set()
        fields=[]
        for name,value in zip(("Tên","X","Y","Z","mm/s","Nghỉ (s)"),(p.name,*p.xyz,p.speed_mm_s,p.dwell_s)):
            row=tk.Frame(win,bg="#172338"); row.pack(fill="x",padx=12,pady=4)
            self.label(row,name).pack(side="left"); v=tk.StringVar(value=str(value)); self.entry(row,v).pack(side="right"); fields.append(v)
        note=tk.StringVar(); self.label(win,textvariable=note,color="#ff8795",wraplength=300).pack()
        def save():
            try:
                if self.program_points: raise ValueError("Dừng chuỗi trước khi sửa.")
                self.points[i]=Setpoint(fields[0].get(),*(float(v.get()) for v in fields[1:])).validate()
                self.refresh_points(); win.destroy()
            except ValueError as e: note.set(str(e))
        self.button(win,"Lưu điểm",save).pack(pady=8)

    def delete_point(self):
        i=self.selected_index()
        if i is not None and not self.program_points: self.points.pop(i); self.refresh_points()

    def import_points(self):
        if self.program_points: return
        path=filedialog.askopenfilename(parent=self.points_window,initialdir=self.config_path.parent,filetypes=[("Setpoints CSV","*.csv")])
        if path:
            try: self.points=load_points(path); self.refresh_points()
            except (OSError,ValueError) as e: self.message.set(str(e))

    def export_points(self):
        path=filedialog.asksaveasfilename(parent=self.points_window,initialdir=self.config_path.parent,defaultextension=".csv",initialfile="scara_setpoints.csv",filetypes=[("Setpoints CSV","*.csv")])
        if path:
            try: save_points(path,self.points)
            except (OSError,ValueError) as e: self.message.set(str(e))

    def run_selected_point(self):
        i=self.selected_index()
        if i is not None: self.start_plan(True,target=self.points[i].xyz,speed=self.points[i].speed_mm_s)
