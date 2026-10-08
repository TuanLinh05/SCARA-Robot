"""Read-only audit of recorded commands/telemetry; no serial ports are opened."""
from pathlib import Path
from dataclasses import asdict
import argparse,json,math
from cartesian_nc_model import NCConfig
from cartesian_nc_protocol import NCStatus
from cartesian_nc_drawing import BKSettings,compile_bk

def audit_log(path):
    config=None; state=None; current=None; drawing_index=-1; strokes={}; drawings=[]; homes=[]; warnings=[]
    status_count=0; rejected=0; last_home_epoch=None
    with Path(path).open(encoding="utf-8") as stream:
        for line in stream:
            try: event=json.loads(line)
            except ValueError: rejected+=1; continue
            kind=event.get("type")
            if kind=="connection":
                if event.get("demo"): return {"file":str(path),"demo":True,"drawings":[]}
                config=NCConfig.from_dict(event["config"])
            elif kind=="status":
                data=event["status"]; state=NCStatus(**{k:tuple(v) if isinstance(v,list) else v for k,v in data.items()})
                status_count+=1
                if state.referenced and state.stage==8 and state.epoch!=last_home_epoch:
                    last_home_epoch=state.epoch
                    homes.append(dict(epoch=state.epoch,factors_pp_unit=[v/1024 for v in state.factor],
                        range=list(state.range),n1=list(state.n1),n2=list(state.n2),
                        coupling=state.coupling_ppm/1000000,beta=state.beta_q/1048576,
                        probe_da=state.probe_da,probe_db=state.probe_db,timer_late=state.timer_late))
            elif kind=="rx_rejected": rejected+=1
            elif kind=="drawing_start" and config and state:
                drawing_index+=1; current=event["settings"]; model=config.model(state)
                program=compile_bk(model,state.pos,BKSettings(**current),event.get("dry",False))
                drawings.append(dict(pattern=current["pattern"],settings=current,
                    model_error_mm=program.max_model_error_mm,strokes=[],completed=False,
                    timer_late_at_start=state.timer_late,measured_factors=list(model.factors),coupling=model.coupling))
            elif kind=="stroke_start" and state and current:
                profiles=event["profiles"]; start=tuple(state.pos); end=tuple(profiles[-1]["steps"])
                position=start; reversals=[0,0,0]; previous=[0,0,0]; expected=[0,0,0]
                for segment in profiles:
                    delta=[b-a for a,b in zip(position,segment["steps"])]
                    for i,d in enumerate(delta):
                        expected[i]+=abs(d); sign=(d>0)-(d<0)
                        if sign and previous[i] and sign!=previous[i]: reversals[i]+=1
                        if sign: previous[i]=sign
                    position=tuple(segment["steps"])
                record=dict(job=event["job"],start_steps=list(start),target_steps=list(end),
                    commanded_closed=start==end,segments=len(profiles),expected_axis_pulses=expected,
                    direction_reversals=reversals,constant_z=all(s["steps"][0]==start[0] for s in profiles),
                    timer_late_at_start=state.timer_late,start_total=list(state.total))
                strokes[event["job"]]=record; drawings[drawing_index]["strokes"].append(record)
            elif kind=="stroke_complete" and event["job"] in strokes:
                record=strokes[event["job"]]
                record.update(end_steps=event["pos"],step_endpoint_error=[a-b for a,b in zip(event["pos"],record["target_steps"])],
                    timer_late_delta=event["timer_late"]-record["timer_late_at_start"],starved=event["starved"],
                    elapsed_s=event["elapsed_s"],predicted_s=event["estimated_s"])
                if state and state.job==event["job"]:
                    counted=[a-b for a,b in zip(state.total,record["start_total"])]
                    record["reported_axis_pulses"]=counted
                    record["axis_pulse_error"]=[a-b for a,b in zip(counted,record["expected_axis_pulses"])]
            elif kind=="drawing_complete" and drawing_index>=0:
                drawings[drawing_index].update(completed=True,elapsed_s=event["elapsed_s"],
                    timer_late_at_end=event["status"]["timer_late"],starved=event["status"]["starved"])
    if config:
        warnings.append("Measured pulse/angle factors assume the configured switch endpoint angles; those angles were not measured by this log.")
        warnings.append("Telemetry counts commanded pulses, not encoder position. Backlash, missed driver pulses/steps, pen flex/contact and actual dimensions are not measured.")
    return dict(file=str(Path(path).resolve()),demo=False,config=asdict(config) if config else None,
        status_frames=status_count,rejected_frames=rejected,home=homes,drawings=drawings,limitations=warnings)

def markdown_report(report):
    lines=["# Kiểm tra log nét vẽ", "",f"Nguồn: `{report['file']}`", "",
        "Báo cáo đọc lệnh và telemetry. Không mở COM và không di chuyển robot.", "",
        "| Mẫu | Cỡ ngang (mm) | Sai lệch mô hình tối đa (mm) | Hoàn tất | Trễ timer tăng lúc vẽ |", "|---|---:|---:|---|---:|"]
    for d in report["drawings"]:
        delay=d.get("timer_late_at_end",d["timer_late_at_start"])-d["timer_late_at_start"]
        lines.append(f"| {d['pattern']} | {d['settings']['size_mm']:g} | {d['model_error_mm']:.4f} | {'Có' if d['completed'] else 'Chưa'} | {delay} |")
    closed=[s for d in report["drawings"] for s in d["strokes"] if s["commanded_closed"]]
    verified=sum("step_endpoint_error" in s and not any(s["step_endpoint_error"]) for s in closed)
    lines.extend(("", f"Nét đóng có telemetry xác nhận đúng tọa độ xung cuối: {verified}/{len(closed)}; xem JSON để đối chiếu từng nét.","",
        "Đây không phải phép đo vị trí vật lý. Log không xác nhận motor/bút đã đi đúng các xung đó.", "",
        "Điểm cần đo tiếp: góc thật của hai đầu hành trình, khoảng cách tâm J1–J2 và tâm J2–đầu bút thực; độ lệch/độ rơ khi đảo chiều và lực tì bút.", ""))
    return "\n".join(lines)

def main():
    parser=argparse.ArgumentParser(); parser.add_argument("log"); parser.add_argument("--out",required=True)
    args=parser.parse_args(); result=audit_log(args.log); target=Path(args.out)
    target.with_suffix(".json").write_text(json.dumps(result,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    target.with_suffix(".md").write_text(markdown_report(result),encoding="utf-8")
    print(json.dumps({"drawings":len(result["drawings"]),"demo":result["demo"],"report":str(target.with_suffix('.md'))}))

if __name__=="__main__": main()
