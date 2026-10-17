from pathlib import Path
import shutil
import tempfile

ROOT = Path(SPECPATH).resolve().parent
DIST_APP = ROOT / "dist" / "SCARA_Cartesian_NC_v5_R14"
LOG_BACKUP_DIR = Path(tempfile.mkdtemp(prefix="scara_logs_backup_"))
if (DIST_APP / "logs").exists():
    for p in (DIST_APP / "logs").glob("*.jsonl"):
        if p.is_file():
            shutil.copy2(p, LOG_BACKUP_DIR / p.name)


a = Analysis(
    [str(ROOT / "src" / "cartesian_nc_control.py")],
    pathex=[str(ROOT / "src")],
    binaries=[],
    datas=[(str(ROOT / "cartesian_nc_config.json"), ".")],
    hiddenimports=[],
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
    upx=True,
    console=False,
    disable_windowed_traceback=False,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name="SCARA_Cartesian_NC_v5_R14",
)

shutil.copy2(ROOT / "cartesian_nc_config.json", DIST_APP / "cartesian_nc_config.json")
(DIST_APP / "logs").mkdir(parents=True, exist_ok=True)
for p in LOG_BACKUP_DIR.glob("*.jsonl"):
    target = DIST_APP / "logs" / p.name
    if not target.exists():
        shutil.copy2(p, target)
shutil.rmtree(LOG_BACKUP_DIR, ignore_errors=True)
