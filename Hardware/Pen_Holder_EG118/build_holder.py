"""Generate printable SCARA pen holder meshes and verify their interfaces.

Units: mm. Local +Y points out through the open end of the fork; +Z is up.
Install: python -m pip install -r requirements.txt
Run: python build_holder.py --pen-diameter 9
The STL files are already supplied; running this script is optional.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math
import sys

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent.parent
if (PROJECT / ".cad_tools").exists():
    sys.path.append(str(PROJECT / ".cad_tools"))
import numpy as np
import manifold3d as md
import trimesh

N = 96


def box(lo, hi):
    return md.Manifold.cube(np.subtract(hi, lo).tolist()).translate(lo)


def cylinder(radius, length, origin, axis="z", segments=N):
    solid = md.Manifold.cylinder(length, radius, circular_segments=segments)
    if axis == "x":
        solid = solid.rotate([0, 90, 0])
    elif axis == "y":
        solid = solid.rotate([-90, 0, 0])
    return solid.translate(origin)


def rounded_rect(x0, y0, x1, y1, radius=1.5):
    return md.CrossSection.square([x1-x0-2*radius, y1-y0-2*radius]).offset(
        radius, circular_segments=32).translate([x0+radius, y0+radius])


def hex_socket(x, y, z, af=5.8, depth=2.6):
    # One horizontal flat at the ceiling, for a short printable bridge.
    r = af / math.sqrt(3)
    points = [[r*math.cos(t), r*math.sin(t)] for t in np.linspace(0, 2*math.pi, 7)[:-1]]
    return md.CrossSection([points]).extrude(depth).rotate([-90, 0, 0]).translate([x, y, z])


def mesh(solid):
    raw = solid.to_mesh()
    return trimesh.Trimesh(vertices=raw.vert_properties[:, :3], faces=raw.tri_verts, process=True)


def as_solid(m):
    return md.Manifold(md.Mesh(np.asarray(m.vertices, dtype=np.float32),
                               np.asarray(m.faces, dtype=np.uint32)))


def build_parts(diameter, height=36, coupon=False):
    r = 12.4
    angles = np.linspace(math.pi, 2*math.pi, N//2+1)
    outline = np.vstack([np.c_[r*np.cos(angles), r*np.sin(angles)], [[r, 13.5], [-r, 13.5]]])
    profile = md.CrossSection([outline])
    # The original fork has a flat at the back of its nominal 25.5 mm arc.
    profile = profile ^ md.CrossSection.square([40, 40]).translate([-20, -11.4])
    profile = profile + rounded_rect(-17, 12, 17, 23.2)
    base = profile.extrude(height)
    # 90-degree saddle: contact on two faces, tolerant of 8.5-9.5 mm barrels.
    vee = md.CrossSection([[[0, 14], [22, 36], [-22, 36]]]).extrude(height+2).translate([0, 0, -1])
    base = base - vee
    # Through window cuts weight; wall remains >= 5.4 mm around the fork.
    window = rounded_rect(-7, -6, 7, 6, 2).extrude(height+2).translate([0, 0, -1])
    base = base - window

    mount_z = [4.0] if coupon else [5.5, 30.5]
    for z in mount_z:
        # 3.6 x 6.0 mm vertical capsule slot; no plastic threads.
        z0, z1 = z-1.2, z+1.2
        slot = cylinder(1.8, 50, [-25, 0, z0], "x") + cylinder(1.8, 50, [-25, 0, z1], "x")
        slot = slot + box([-25, -1.8, z0], [25, 1.8, z1])
        base = base - slot

    pen_y = 14 + diameter/math.sqrt(2)
    pressure_y = pen_y + diameter/2
    # Cap position is the theoretical contact position for the selected barrel.
    cap_profile = rounded_rect(-17, pressure_y, 17, pressure_y+4, 1.2)
    cap = cap_profile.extrude(height)
    clamp_z = [height/2] if coupon else [12, 24]
    for x in [-13, 13]:
        for z in clamp_z:
            hole = cylinder(1.7, 44, [x, -2, z], "y")
            base = base - hole - hex_socket(x, 11.99, z, depth=2.61)
            cap = cap - hole
    return base, cap, {"pen_axis_y_mm": pen_y, "pressure_face_y_mm": pressure_y,
                        "cap_gap_mm": pressure_y-23.2, "height_mm": height,
                        "mount_centers_z_mm": mount_z, "clamp_centers_z_mm": clamp_z}


def aligned_forks():
    source_dir = PROJECT / "Hardware/arm/STL/Black_Quantity1"
    parts = []
    records = []
    for name in ["Bottom", "Top"]:
        p = source_dir / f"Arm_Elbow_Plate_{name}_1.0.0.stl"
        m = trimesh.load(p, force="mesh")
        if name == "Bottom":
            # The bottom STL opens toward +Y.
            m.apply_translation([0, -54.672073, 0])
        else:
            # Rotate 180 deg about X: top recesses face the bottom recesses.
            m.apply_transform(np.array([[1, 0, 0, 0], [0, -1, 0, -54.672073],
                                       [0, 0, -1, 36], [0, 0, 0, 1]]))
        parts.append(m)
        records.append({"path": str(p.relative_to(PROJECT)).replace("\\", "/"),
                        "sha256": hashlib.sha256(p.read_bytes()).hexdigest()})
    return parts, records


def export_part(name, solid, orient_cap=False):
    m = mesh(solid)
    if orient_cap:
        # Outer flat face on the print bed. All screw holes print vertically.
        m.apply_transform(trimesh.transformations.rotation_matrix(math.pi/2, [1, 0, 0]))
    m.apply_translation(-m.bounds[0])
    m.export(ROOT / "STL" / name)
    if not m.is_watertight or not m.is_winding_consistent or m.volume <= 0:
        raise RuntimeError(f"Invalid printable mesh: {name}")
    if len(solid.decompose()) != 1:
        raise RuntimeError(f"Disconnected solid: {name}")
    return {"file": name, "watertight": bool(m.is_watertight),
            "winding_consistent": bool(m.is_winding_consistent),
            "connected_solids": len(solid.decompose()), "triangles": len(m.faces),
            "dimensions_mm": m.extents.round(4).tolist(),
            "volume_mm3": round(m.volume, 3)}


def validate(base, cap, forks, diameter, dims):
    result = []
    for name, part in [("base", base), ("cap", cap)]:
        for side, fork in zip(["Bottom", "Top"], forks):
            intersection = part ^ as_solid(fork)
            vol = intersection.volume()
            result.append({"part": name, "reference": side, "intersection_mm3": round(vol, 8)})
            if vol > 0.001:
                raise RuntimeError(f"Collision {name}/{side}: {vol}")
    clamp = base + cap
    for d in [8.5, diameter, 9.5]:
        py = 14 + d/math.sqrt(2)
        face_y = py+d/2
        moved_cap = cap.translate([0, face_y-dims["pressure_face_y_mm"], 0])
        pen = cylinder(d/2-0.005, 50, [0, py, -7])
        collision = (pen ^ (base+moved_cap)).volume()
        jaw_collision = (base ^ moved_cap).volume()
        result.append({"barrel_diameter_mm": d, "pen_axis_y_mm": round(py, 5),
                       "cap_gap_mm": round(face_y-23.2, 5),
                       "barrel_intersection_mm3": round(collision, 8),
                       "jaw_intersection_mm3": round(jaw_collision, 8)})
        if collision > 0.001 or jaw_collision > 0.001 or face_y <= 23.2:
            raise RuntimeError(f"Invalid clamp fit for {d} mm barrel")
    # Both retaining bolts must pass through the complete assembled plastic.
    for z in [5.5, 30.5]:
        bolt = cylinder(1.5, 52, [-26, 0, z], "x")
        collision = (bolt ^ (base + as_solid(forks[0]) + as_solid(forks[1]))).volume()
        result.append({"mount_bolt_z_mm": z, "M3_shaft_intersection_mm3": round(collision, 8)})
        if collision > 0.001:
            raise RuntimeError(f"Mount bolt obstructed at Z={z}")
    # Clamp bolt and external nut seats remain accessible between the plates.
    for x in [-13, 13]:
        for z in [12, 24]:
            # 20 mm under head length, with head on outer cap face.
            bolt = cylinder(1.5, 20, [x, dims["pressure_face_y_mm"]+4-20, z], "y")
            collision = (bolt ^ (clamp+as_solid(forks[0])+as_solid(forks[1]))).volume()
            result.append({"clamp_bolt_xz_mm": [x, z], "M3_shaft_intersection_mm3": round(collision, 8)})
            if collision > 0.001:
                raise RuntimeError(f"Clamp bolt obstructed at {x}, {z}")
    return result


def render_preview(base, cap, forks, diameter, dims):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    def add(ax, part, color, alpha=1):
        m = mesh(part) if isinstance(part, md.Manifold) else part
        tris = m.triangles
        rgb = np.array(matplotlib.colors.to_rgb(color))
        light = np.array([-.35, -.45, .82]); light /= np.linalg.norm(light)
        shade = .52+.48*np.clip(m.face_normals @ light, 0, 1)
        colors = np.c_[shade[:, None]*rgb, np.full(len(tris), alpha)]
        ax.add_collection3d(Poly3DCollection(tris, facecolors=colors, edgecolors="none"))

    # Actual source meshes, cropped only for the preview.
    crop = box([-27, -30, -1], [27, 12, 40])
    cropped = [as_solid(f) ^ crop for f in forks]
    pen_y = dims["pen_axis_y_mm"]
    barrel = cylinder(diameter/2, 87, [0, pen_y, -17])
    tip = md.Manifold.cylinder(11, .35, diameter/2, circular_segments=64).translate([0, pen_y, -28])

    fig = plt.figure(figsize=(15, 9), facecolor="#f5f7fa")
    for i in range(2):
        ax = fig.add_subplot(1, 2, i+1, projection="3d", facecolor="#f5f7fa")
        for f in cropped:
            add(ax, f, "#454f5d")
        add(ax, base, "#f39728")
        add(ax, cap.translate([0, 14 if i else 0, 0]), "#21a69b")
        if not i:
            add(ax, barrel, "#d4dce7")
            add(ax, tip, "#8797ab")
            for z in [5.5, 30.5]:
                add(ax, cylinder(1.5, 50, [-25, 0, z], "x", 24), "#bcc5d0")
                add(ax, cylinder(2.75, 3, [-25, 0, z], "x", 24), "#68798e")
        for x in [-13, 13]:
            for z in [12, 24]:
                start_y = dims["pressure_face_y_mm"]+4+(14 if i else 0)
                add(ax, cylinder(1.5, 20, [x, start_y-20, z], "y", 24), "#bcc5d0")
                add(ax, cylinder(2.75, 3, [x, start_y, z], "y", 24), "#68798e")
        ax.set_xlim(-28, 28); ax.set_ylim(-30, 49); ax.set_zlim(-29, 74)
        ax.set_box_aspect([56, 79, 103]); ax.view_init(23, 55)
        ax.set_axis_off()
        ax.set_title("ASSEMBLED" if i == 0 else "CAP REMOVED / V-SADDLE", fontsize=14, fontweight="bold", color="#1b2b40")
    fig.suptitle("SCARA / DELI EG118  |  9 mm adjustable screw clamp", fontsize=20, fontweight="bold", color="#1b2b40", y=.96)
    fig.text(.5, .075, "Orange: fork adapter + 90-degree saddle    |    Teal: pressure cap    |    Grey: original fork + barrel proxy",
             ha="center", fontsize=11, color="#3d4b5e")
    fig.text(.5, .035, f"2 x M3x50 mount bolts    /    4 x M3x20 clamp bolts    /    nominal pen offset: {pen_y:.2f} mm outwards",
             ha="center", fontsize=11, color="#3d4b5e")
    fig.subplots_adjust(left=0, right=1, bottom=.08, top=.87, wspace=0)
    fig.savefig(ROOT / "preview.png", dpi=150)
    plt.close(fig)

    # Dimensioned engineering views, in the assembled local coordinate frame.
    fig, axes = plt.subplots(1, 2, figsize=(14, 7), facecolor="white")
    for ax in axes:
        ax.set_aspect("equal"); ax.grid(alpha=.18); ax.set_axisbelow(True)
    ax = axes[0]
    for m, col in [(mesh(base), "#db871e"), (mesh(cap), "#008f85")]:
        section = m.section(plane_origin=[0, 0, 18], plane_normal=[0, 0, 1])
        for e in section.entities:
            pts = e.discrete(section.vertices)
            ax.plot(pts[:, 0], pts[:, 1], color=col, lw=1.4)
    theta = np.linspace(0, 2*math.pi, 100)
    ax.plot(diameter/2*np.cos(theta), pen_y+diameter/2*np.sin(theta), color="#6e7884")
    ax.plot([-24, 24], [0, 0], "--", color="#6e7884", lw=.9)
    ax.annotate("M3 mount axis", (-24, 0), (-23, -5), fontsize=9)
    ax.annotate("", (0, 0), (0, pen_y), arrowprops=dict(arrowstyle="<->", color="#334155"))
    ax.text(1, 8, f"{pen_y:.2f} mm", fontsize=10)
    ax.annotate("", (-17, 33), (17, 33), arrowprops=dict(arrowstyle="<->"))
    ax.text(0, 34, "34 mm", ha="center")
    ax.text(0, -16, "Tongue width 24.8 / rear depth 11.4", ha="center", fontsize=9)
    ax.set_xlim(-27, 27); ax.set_ylim(-20, 38)
    ax.set_title("TOP VIEW / XY (mm)"); ax.set_xlabel("X"); ax.set_ylabel("Y, outward from arm")
    ax = axes[1]
    ax.plot([-12.4, -12.4, 12.4, 12.4, -12.4], [0, 36, 36, 0, 0], color="#db871e", lw=1.5)
    for z in [5.5, 30.5]:
        ax.plot([-12.4, 12.4], [z, z], "--", color="#6e7884", lw=.8)
        ax.text(13.5, z, f"Z={z:g}", va="center", fontsize=9)
    ax.annotate("", (-17, 0), (-17, 36), arrowprops=dict(arrowstyle="<->"))
    ax.text(-19, 18, "36 mm", rotation=90, va="center")
    ax.text(0, -5, "Mount slots: 3.6 x 6.0 mm; center pitch 25 mm", ha="center", fontsize=9)
    ax.text(0, 41, "Clamp holes: X=+/-13; Z=12, 24; bore 3.4 mm", ha="center", fontsize=9)
    ax.set_xlim(-26, 28); ax.set_ylim(-8, 46)
    ax.set_title("MOUNT FRONT VIEW / XZ (mm)"); ax.set_xlabel("X"); ax.set_ylabel("Z")
    fig.tight_layout()
    fig.savefig(ROOT / "dimensions.png", dpi=150)
    plt.close(fig)


def export_reference_scene(base, cap, forks, diameter, dims):
    scene = trimesh.Scene()
    crop = box([-27, -30, -1], [27, 12, 40])
    reference_parts = [
        ("Fork_adapter", base, [245, 151, 40, 255]),
        ("Pressure_cap", cap, [33, 166, 155, 255]),
        ("Original_lower_fork", as_solid(forks[0]) ^ crop, [69, 79, 93, 255]),
        ("Original_upper_fork", as_solid(forks[1]) ^ crop, [69, 79, 93, 255]),
        ("Barrel_reference", cylinder(diameter/2, 87, [0, dims["pen_axis_y_mm"], -17]),
         [212, 220, 231, 255]),
    ]
    for name, solid, color in reference_parts:
        m = mesh(solid)
        m.visual.face_colors = color
        scene.add_geometry(m, node_name=name, geom_name=name)
    scene.export(str(ROOT / "assembly_reference.glb"))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pen-diameter", type=float, default=9)
    args = parser.parse_args()
    if not 8.5 <= args.pen_diameter <= 9.5:
        parser.error("This revision is verified for barrels 8.5-9.5 mm.")
    (ROOT / "STL").mkdir(exist_ok=True)
    base, cap, dims = build_parts(args.pen_diameter)
    forks, records = aligned_forks()
    checks = validate(base, cap, forks, args.pen_diameter, dims)
    cb, cc, _ = build_parts(args.pen_diameter, height=8, coupon=True)
    info = [export_part("01_Fork_Adapter_V_Saddle.stl", base),
            export_part("02_Pressure_Cap.stl", cap, orient_cap=True),
            export_part("03_Test_Adapter_8mm.stl", cb),
            export_part("04_Test_Cap_8mm.stl", cc, orient_cap=True)]
    # Assembly is a reference scene, deliberately not a print file.
    trimesh.util.concatenate([mesh(base), mesh(cap)]).export(ROOT / "assembly_reference.stl")
    report = {"units": "mm", "pen_diameter_input_mm": args.pen_diameter,
              "verified_barrel_range_mm": [8.5, 9.5], "dimensions": dims,
              "source_meshes": records, "fork_profile": {
                  "opening_width_mm": 25.5, "rear_flat_depth_mm": 11.75,
                  "adapter_width_mm": 24.8, "adapter_rear_depth_mm": 11.4,
                  "plate_thickness_mm": 9, "assumed_inner_plate_gap_mm": 18,
                  "assumed_outer_stack_mm": 36, "bolt_pitch_mm": 25,
                  "fork_tip_from_mount_axis_mm": 10.905854,
                  "wide_head_start_mm": 12},
              "print_meshes": info, "geometry_checks": checks,
              "limits": ["Physical fit, screw lengths and pen pressure require a real test.",
                         "Plate spacing inferred from 18 mm sidewalls; measure the built robot.",
                         "No dynamic load, fatigue or full robot travel validation.",
                         "Pen is fixed vertically; this model has no pressure spring."]}
    (ROOT / "validation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    render_preview(base, cap, forks, args.pen_diameter, dims)
    export_reference_scene(base, cap, forks, args.pen_diameter, dims)
    print(json.dumps({"dimensions": dims, "print_meshes": info, "geometry_checks_passed": len(checks)}, indent=2))


if __name__ == "__main__":
    main()
