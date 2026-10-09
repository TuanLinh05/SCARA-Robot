# Build from Software: python -m PyInstaller --noconfirm packaging/SCARA_Cartesian_NC_v5_R14.spec
from pathlib import Path
import sys
import json

software = Path(SPECPATH).resolve().parent
sys.path.insert(0, str(software / "src"))
from cartesian_nc_upgrade import upgrade_r14

current = Path(DISTPATH) / "SCARA_Cartesian_NC_v5_R14/cartesian_nc_config.json"
candidates = [current, software / "cartesian_nc_config.json"]
source = next(p for p in candidates if p.is_file())
saved_config = (
    source.read_bytes()
    if source == current
    else (
        json.dumps(
            upgrade_r14(json.loads(source.read_text(encoding="utf-8-sig"))),
            indent=2,
            ensure_ascii=False,
        )
        + "\n"
    ).encode("utf-8")
)
# COLLECT replaces its output directory. Keep diagnostic evidence when rebuilding.
saved_logs = {
    p.name: p.read_bytes()
    for p in (current.parent / "logs").glob("*.jsonl")
    if p.is_file()
}
events = current.parent / "cartesian_events.jsonl"
saved_events = events.read_bytes() if events.is_file() else None
saved_csv = {
    p.name: p.read_bytes() for p in current.parent.glob("*.csv") if p.is_file()
}

a = Analysis(
    [str(software / "src/cartesian_nc_control.py")],
    pathex=[str(software / "src")],
    binaries=[],
    datas=[],
    hiddenimports=["cartesian_nc_demo"],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)
exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name="SCARA_Cartesian_NC_v5_R14",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=False,
    disable_windowed_traceback=False,
)
coll = COLLECT(
    exe, a.binaries, a.datas, strip=False, upx=False, name="SCARA_Cartesian_NC_v5_R14"
)
(Path(DISTPATH) / "SCARA_Cartesian_NC_v5_R14/cartesian_nc_config.json").write_bytes(
    saved_config
)
log_dir = current.parent / "logs"
log_dir.mkdir(exist_ok=True)
for name, data in saved_logs.items():
    (log_dir / name).write_bytes(data)
if saved_events is not None:
    events.write_bytes(saved_events)

for name, data in saved_csv.items():
    (current.parent / name).write_bytes(data)
