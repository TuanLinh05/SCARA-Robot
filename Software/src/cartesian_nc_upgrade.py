"""Upgrade R1/R2 parameters for the requested Z=1 mm/s profile, keep mechanics."""
import json
def upgrade_r3(data):
    result=dict(data)
    factors=200*result.get("microsteps",(16,8,8))[0]/result.get("lead_mm_rev",2.0)
    result["z_home_pps"]=round(factors)
    result["max_z_speed_mm_s"]=1.0
    result["max_master_rate"]=max(result.get("max_master_rate",1600),round(factors))
    result.setdefault("z_latch_pps",max(50,round(factors*.25)))
    result.setdefault("home_in_background",True)
    return result

def upgrade_r4(data):
    result=dict(data)
    result.setdefault("z_latch_pps",min(400,result.get("z_home_pps",1600)))
    result.setdefault("home_in_background",True)
    result.setdefault("j2_home_pps",800)
    result.setdefault("j2_latch_pps",200)
    return result

def upgrade_r5(data):
    result=upgrade_r4(data)
    # Correct the known observation error; do not alter motor polarity,
    # microsteps, geometry, rates or measured switch ranges during migration.
    result["coupling"]=1.333333
    return result

def upgrade_r6(data):
    result=upgrade_r5(data)
    # J2+ in the old jog GUI turns opposite to J1+. Normalize B to A's
    # geometric sign; do not toggle on repeated migration or change J1/Z.
    polarity=list(result.get("positive_high",(True,True,True)))
    polarity[2]=not polarity[1]
    result["positive_high"]=polarity
    return result

def upgrade_r7(data):
    # Preserve the user's current signs and seed geometry. R7 measures dB/dA
    # before the long J1 sweep instead of imposing another guessed coefficient.
    result=upgrade_r4(data)
    result.setdefault("auto_coupling",True)
    result.setdefault("coupling_probe_pulses",128)
    return result

def upgrade_r10(data):
    result=upgrade_r7(data)
    result.setdefault("motion_in_background",True)
    return result

def upgrade_r12(data):
    result=upgrade_r10(data)
    result.pop("z_home_pps",None)
    result.setdefault("z_home_speed_mm_s",3.0)
    return result

def upgrade_r14(data):
    result=upgrade_r12(data)
    result.setdefault("tool_offset_mm",[0.0,0.0])
    return result

