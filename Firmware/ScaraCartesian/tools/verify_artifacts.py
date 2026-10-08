"""Checks the distributable Intel HEX/BIN image without touching hardware."""
import hashlib
import json
from pathlib import Path
import struct
from elftools.elf.elffile import ELFFile
root=Path(__file__).resolve().parents[1]
name="SCARA_Cartesian_NC_Home_v5_R9"
hexpath=root/"Hex"/(name+".hex"); binpath=root/"Hex"/(name+".bin")
memory={}; base=0; eof=False
for line in hexpath.read_text(encoding="ascii").splitlines():
    if not line: continue
    assert line[0]==":"
    data=bytes.fromhex(line[1:]); count=data[0]; offset=int.from_bytes(data[1:3],"big"); kind=data[3]
    assert len(data)==count+5 and sum(data)%256==0
    payload=data[4:4+count]
    if kind==0:
        for i,b in enumerate(payload): memory[base+offset+i]=b
    elif kind==4: base=int.from_bytes(payload,"big")<<16
    elif kind==2: base=int.from_bytes(payload,"big")<<4
    elif kind==1: eof=True
    elif kind not in (3,5): raise AssertionError(kind)
binary=binpath.read_bytes()
assert eof and min(memory)==0x08000000 and max(memory)<0x08010000
rebuilt=bytes(memory.get(addr,255) for addr in range(min(memory),max(memory)+1))
assert rebuilt==binary and len(binary)<65536
sp,reset=struct.unpack_from("<II",binary)
assert 0x20000000<sp<=0x20005000 and reset&1 and 0x08000000<=(reset&~1)<0x08000000+len(binary)
assert b"SCARA_CART_NC_HOME_V5_R9" in binary
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
elfpath=root/"build_mcu/zephyr/zephyr.elf"
if elfpath.is_file():
    with elfpath.open("rb") as stream:
        elf=ELFFile(stream)
        symbols={s.name:s["st_value"] for s in elf.get_section_by_name(".symtab").iter_symbols()
                 if s.name in ("_image_ram_start","_image_ram_end")}
    ram_bytes=symbols["_image_ram_end"]-symbols["_image_ram_start"]
    assert symbols["_image_ram_start"]==0x20000000 and symbols["_image_ram_end"]<=0x20005000
    ram_source="linked ELF"
else:
    saved=json.loads((root/"Hex/verification_nc_v5_R9.json").read_text(encoding="utf-8"))
    assert saved["hex_sha256"]==sha(hexpath) and saved["bin_sha256"]==sha(binpath)
    ram_bytes=saved["ram_link_bytes"]; assert 0<ram_bytes<=20480
    ram_source="saved linked report matching image checksums"
report={"build":"SCARA_CART_NC_HOME_V5_R9","flash_bytes":len(binary),"flash_limit":65536,
        "ram_link_bytes":ram_bytes,"ram_limit":20480,"ram_check_source":ram_source,"base":"0x08000000","initial_sp":hex(sp),
        "reset_vector":hex(reset),"hex_sha256":sha(hexpath),"bin_sha256":sha(binpath),
        "configuration_sha256":{name:sha(root/name) for name in ("prj.conf","app.overlay")},
        "sources_sha256":{p.name:sha(p) for p in sorted((root/"src").glob("*")) if p.is_file()},
        "validation":["R9: pure-Z calibration up to 6400pps, default 4800pps=3mm/s at 2mm/rev and x16; latch stays400pps",
                      "R9: quintic v-squared distance profiles, cubic Bezier adaptive geometry and full-pulse ink preflight",
                      "R12 GUI: fit enlarged lettering at chosen center; >500 total vertices allowed with <=500 per stroke","R8: continuous 32-segment FIFO, acceleration lookahead, exact GPIO pulse counts and HIGH drain/DIR setup",
                      "R8: fixed Z ink, full-stroke preflight, STOP/USB/heartbeat/endstop and starvation abort",
                      "R8: actual ISR timing comparisons for BK/LINH/THOA; no hardware drawing by agent","R7: measured endpoint shift dB/dA replaces nominal compensation before long J1 home",
                      "R7: actual J1 pulse scale 2x/3x nominal holds elbow and completes home in real DIR/PUL harness",
                      "R7: opposite measured sign aborts before J1 sweep; near-endpoint probe retries and watchdog retained",
                      "R7: nominal beta on 3x J1 scale reproduces J2 guard after 20--40 degrees; measured beta completes home",
                      "R7: per-connection asynchronous JSONL config, commands, ACK, telemetry, rejected frames, STOP; effective measured k used by XYZ",
                      "R6: physical pose from actual PUL edges and DIR pads; wrong J2 sign reproduces guard failure",
                      "R6: opposite J1/J2 DIR holds elbow during J1 home; physical PB10/PB11 mapping and measured XYZ verified",
                      "R6: deterministic config migration keeps J1/Z, sets J2 opposite J1, retains k=4/3",
                      "coupled full home/calibration: core and real GPIO/timer harness",
                      "R5: passive absolute arm2 heading=-A/3; relative elbow=B-4A/3, both +/-90-degree observations",
                      "R5: old 2/3 coefficient reproduces J2 guard failure; 4/3 completes home from both shoulder extremes",
                      "R5: relative-elbow compensation ~=4 B pulses/A pulse; FK/IK use same coefficient",
                      "R4: inactive arm glitches do not interrupt Z; active first HIGH and persistent all-axis faults still stop",
                      "R4: false single hits on Z/J1/J2 resume without resetting budgets or DDA compensation",
                      "R4: delayed main loop confirms stable GPIO; stale filtered input gated before new axis",
                      "R4: independent J2 800/200pps; J1 unchanged",
                      "R3: Z search 1600pps, latch 400pps, pure-Z plan at 1mm/s under 2mm/rev estimate",
                      "R3: named STOP source retained through legacy serial-close STOP",
                      "R3: background HOME remains active; real foreground guard and XYZ focus STOP tested",
                      "R2: no pulses during stationary input check; brief double-open on all axes recovers",
                      "R2: coupled J1 and park accumulators retained; sustained/noisy inputs, STOP and lease still abort",
                      "R2: genuine target endpoint after double-open never receives another pulse into the stop",
                      "exact DDA pulses, cross-axis switches, glitches, heartbeat, USB, GPIO fault, clock wrap",
                      "Python FK/IK, measured scales, limits and firmware JSON decoder",
                      "GUI COM fixture workflow, asynchronous plan, STOP, release and stale status"],
        "real_hardware_tested":False}
(root/"Hex"/"verification_nc_v5_R9.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
print(json.dumps({k:v for k,v in report.items() if k not in ("sources_sha256","validation")},indent=2))





