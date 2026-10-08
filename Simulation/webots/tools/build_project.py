"""Rebuild metre-scale STL assets and Webots nodes from shared configuration.

No third-party packages required. Original Hardware files are read only.
Generated paths are relative; the complete Scara Robot folder can be moved.
"""
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import zlib

WEBOTS = Path(__file__).resolve().parents[1]
SIMULATION = WEBOTS.parent
sys.path.insert(0, str(WEBOTS / 'controllers' / 'scara_draw'))
from core.model import fingerprint, ik

HEADER = '#VRML_SIM R2025a utf8\n'


def read_stl(path):
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f'Invalid STL: {path}')
    count = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84 + count * 50:
        raise ValueError(f'Expected a binary STL: {path}')
    return [[list(struct.unpack_from('<3f', data, 84+i*50+12+j*12))
             for j in range(3)] for i in range(count)]


def write_stl(path, triangles):
    data = bytearray(b'X-SCARA metre-scale visual mesh; original design madl3x GPL-3.0'.ljust(80, b' '))
    data.extend(struct.pack('<I', len(triangles)))
    for triangle in triangles:
        a, b, c = triangle
        u, v = [b[j]-a[j] for j in range(3)], [c[j]-a[j] for j in range(3)]
        n = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]]
        length = math.sqrt(sum(x*x for x in n))
        n = [x/length for x in n] if length else [0., 0., 0.]
        data.extend(struct.pack('<12fH', *n, *a, *b, *c, 0))
    path.write_bytes(data)


def white_texture(path, size=1024):
    # Pure white paper: native Pen applies its paint layer without changing PNG.
    raw = (b'\x00' + b'\xff\xff\xff' * size) * size
    def chunk(kind, payload):
        return struct.pack('>I', len(payload)) + kind + payload + struct.pack('>I', zlib.crc32(kind+payload))
    path.write_bytes(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>2I5B', size, size, 8, 2, 0, 0, 0))
                     + chunk(b'IDAT', zlib.compress(raw)) + chunk(b'IEND', b''))


def vec(values):
    return ' '.join(f'{x:.12g}' for x in values)


def appearance(color):
    return f'PBRAppearance {{ baseColor {vec(color)} roughness 0.65 metalness 0 }}'


def shape(geometry, color):
    return f'Shape {{ appearance {appearance(color)} geometry {geometry} }}'


def box(center, dimensions, color, collision=False):
    geometry = f'Box {{ size {vec(dimensions)} }}'
    child = geometry if collision else shape(geometry, color)
    return f'Pose {{ translation {vec(center)} children [ {child} ] }}'


def cylinder(center, radius, height, color):
    return f'Pose {{ translation {vec(center)} children [ {shape(f"Cylinder {{ radius {radius} height {height} subdivision 24 }}", color)} ] }}'


def physics(mass, center, size, rotor=0):
    x, y, z = size
    # Isotropic transverse addition satisfies the inertia triangle inequalities.
    inertia = [mass*(y*y+z*z)/12+rotor/2, mass*(x*x+z*z)/12+rotor/2, mass*(x*x+y*y)/12+rotor]
    return f'Physics {{ density -1 mass {mass:.12g} centerOfMass [ {vec(center)} ] inertiaMatrix [ {vec(inertia)} 0 0 0 ] }}'


def camera_orientation(eye, target):
    # Webots camera: local +X forward, +Y left, +Z up (FLU).
    dx, dy, dz = [b-a for a,b in zip(eye,target)]
    yaw = math.atan2(dy,dx)
    pitch = math.atan2(-dz,math.hypot(dx,dy))
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    w, x, y, z = cy*cp, -sy*sp, cy*sp, sy*cp
    length = math.sqrt(x*x+y*y+z*z)
    return [x/length,y/length,z/length,2*math.atan2(length,w)]


def build():
    p = json.loads((SIMULATION / 'common/config/scara.json').read_text(encoding='utf-8'))
    if p['zMass'] <= p['mass1']+p['mass2']+p['massTool'] or p['massTool'] <= 0.01:
        raise ValueError('zMass must include carriage, arms, tool; massTool must exceed 0.01 kg.')
    if p['dt']*1000 < 1 or not (p['zLimits'][0] < 0 < p['zLimits'][1]):
        raise ValueError('World timestep needs >= 1 ms and Z limits must include zero.')
    hardware = SIMULATION.parent / 'Hardware/arm/STL'
    assets = SIMULATION / 'common/assets'
    mesh_folder = assets / 'meshes'
    texture_folder = assets / 'textures'
    for folder in (mesh_folder, texture_folder, WEBOTS/'protos', WEBOTS/'worlds'):
        folder.mkdir(parents=True, exist_ok=True)
    black, orange, metal = [0.22, 0.25, 0.28], [0.96, 0.47, 0.08], [0.65, 0.71, 0.75]
    parts = [
        ('Arm_Shoulder_Plate_Top', 'shoulder_black', [-1,-1,1], [0,49,97]),
        ('Arm_Shoulder_Plate_Bottom', 'shoulder_black', [-1,-1,1], [0,44.5,70]),
        ('Arm_Elbow_Plate_Top', 'elbow_black', [-1,-1,1], [0,43.327927,67]),
        ('Arm_Elbow_Plate_Bottom', 'elbow_black', [1,1,1], [0,43.327927,40]),
        ('Arm_Base_MotorPlate', 'carriage_black', [-1,-1,1], [36,-3,70]),
        ('Arm_Base_MotorPlate', 'carriage_black', [-1,-1,-1], [36,-3,69]),
        ('Arm_SideWall_Left', 'shoulder_orange', [1,1,1], [14,46,79]),
        ('Arm_SideWall_Right', 'shoulder_orange', [-1,1,1], [-14,46,79]),
        ('Arm_SideWall_Left', 'elbow_orange', [1,1,1], [14,46,49]),
        ('Arm_SideWall_Right', 'elbow_orange', [-1,1,1], [-14,46,49]),
        ('Arm_Elbow_RodHub', 'elbow_orange', [1,1,1], [0,p['L2']*1000,49])]
    merged, manifest = {}, []
    for stem, group, signs, offset in parts:
        matches = sorted(hardware.rglob(stem+'_*.stl'))
        if len(matches) != 1:
            raise ValueError(f'Expected one source STL: {stem}, found {len(matches)}')
        source = matches[0]
        triangles = read_stl(source)
        factor = p['L1']/0.098 if group.startswith('shoulder') else p['L2']/0.098 if group.startswith('elbow') and stem != 'Arm_Elbow_RodHub' else 1
        for triangle in triangles:
            for vertex in triangle:
                for j in range(3):
                    vertex[j] = (vertex[j]*signs[j]+offset[j])*0.001
                vertex[1] *= factor
            if math.prod(signs) < 0:  # mirrored mesh: restore outward winding
                triangle[1], triangle[2] = triangle[2], triangle[1]
        merged.setdefault(group, []).extend(triangles)
        manifest.append({'source': source.relative_to(SIMULATION.parent).as_posix(),
                         'sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
                         'output': group+'.stl', 'signs': signs, 'translation_mm': offset,
                         'y_scale': factor, 'triangles': len(triangles)})
    for group, triangles in merged.items():
        write_stl(mesh_folder/(group+'.stl'), triangles)
    white_texture(texture_folder/'paper.png')
    (assets/'manifest.json').write_text(json.dumps({'source_repository': 'https://github.com/madl3x/x-scara',
        'source_commit': '3c7c17e3dc0805b41a3cac6d4cd2778d585d7ed2', 'license': 'GPL-3.0',
        'units': 'm', 'assembly': 'estimated Z stack and sidewall placement',
        'physical_config_sha256': fingerprint(p), 'parts': manifest}, indent=2)+'\n', encoding='utf-8')
    def mesh(group):
        color = orange if group.endswith('orange') else black
        return shape(f'Mesh {{ url [ "../../common/assets/meshes/{group}.stl" ] }}', color)
    s0, e0, z0 = ik([0.034, 0, p['penLift']], p)
    l1, l2 = p['L1'], p['L2']
    tool_xy = [p['toolOffset'][0], l2+p['toolOffset'][1]]
    frame = '\n'.join([box([-0.042,-0.015,0.15],[0.02,0.04,0.30],metal),
                       box([0.114,-0.015,0.15],[0.02,0.04,0.30],metal),
                       box([0.036,-0.025,0.29],[0.176,0.02,0.02],metal),
                       box([0.036,-0.025,0.01],[0.176,0.02,0.02],metal)]
                      + [cylinder([x,y,0.16],0.004,0.30,metal) for x,y in [(-.038,0),(.110,0),(.036,-.039)]])
    carriage_mass = p['zMass'] - p['mass1'] - p['mass2'] - p['massTool']
    shoulder_limits, elbow_limits = p['jointLimits']
    proto = f'''# Generated by tools/build_project.py; edit builder or common config, then rebuild.
# Actual STL graphics; primitive collision volumes; estimated rigid-body properties.
PROTO ScaraBrush [
  field SFString controller "scara_draw"
  field MFString controllerArgs []
] {{
  Robot {{
    translation {vec(p['base']+[0])}
    name "X-SCARA brush robot"
    controller IS controller
    controllerArgs IS controllerArgs
    customData "{fingerprint(p)}"
    selfCollision FALSE
    children [
      {frame}
      DEF Z_JOINT SliderJoint {{
        jointParameters JointParameters {{
          axis 0 0 1 position {z0:.12g}
          minStop {p['zLimits'][0]} maxStop {p['zLimits'][1]}
          dampingConstant {p['viscous'][2]} staticFriction {p['coulomb'][2]}
        }}
        device [
          LinearMotor {{ name "z_motor" maxVelocity 0.05 maxForce {p['actuatorLimits'][2]}
            minPosition {p['zLimits'][0]} maxPosition {p['zLimits'][1]} controlPID 150 0 0 }}
          PositionSensor {{ name "z_sensor" }}
        ]
        endPoint DEF CARRIAGE Solid {{
          translation 0 0 {z0:.12g}
          name "Z carriage and motors"
          children [
            {mesh('carriage_black')}
            {box([0,0.025,0.039],[0.042,0.042,0.040],black)}
            {box([0.072,0.025,0.039],[0.042,0.042,0.040],black)}
            {cylinder([0,0,0.0835],0.013,0.049,metal)}
            DEF SHOULDER_JOINT HingeJoint {{
              jointParameters HingeJointParameters {{
                axis 0 0 1 anchor 0 0 0 position {s0:.12g}
                minStop {shoulder_limits[0]} maxStop {shoulder_limits[1]}
                dampingConstant {p['viscous'][0]} staticFriction {p['coulomb'][0]}
              }}
              device [
                RotationalMotor {{ name "shoulder_motor" maxVelocity 3 maxTorque {p['actuatorLimits'][0]}
                  minPosition {shoulder_limits[0]} maxPosition {shoulder_limits[1]} controlPID 30 0 0 }}
                PositionSensor {{ name "shoulder_sensor" }}
              ]
              endPoint DEF SHOULDER Solid {{
                rotation 0 0 1 {s0:.12g} name "Shoulder arm"
                children [
                  {mesh('shoulder_black')} {mesh('shoulder_orange')}
                  {cylinder([0,l1,.0685],.004,.075,metal)}
                  DEF ELBOW_JOINT HingeJoint {{
                    jointParameters HingeJointParameters {{
                      axis 0 0 1 anchor 0 {l1} 0 position {e0:.12g}
                      minStop {-math.pi} maxStop {math.pi}
                      dampingConstant {p['viscous'][1]} staticFriction {p['coulomb'][1]}
                    }}
                    device [
                      RotationalMotor {{ name "elbow_motor" maxVelocity 4 maxTorque {p['actuatorLimits'][1]}
                        minPosition {elbow_limits[0]} maxPosition {elbow_limits[1]} controlPID 30 0 0 }}
                      PositionSensor {{ name "elbow_sensor" }}
                    ]
                    endPoint DEF ELBOW Solid {{
                      translation 0 {l1} 0 rotation 0 0 1 {e0:.12g} name "Elbow arm"
                      children [
                        {mesh('elbow_black')} {mesh('elbow_orange')}
                        DEF BRUSH_BODY Solid {{
                          translation {vec(tool_xy+[0])} name "Brush holder"
                          children [
                            {cylinder([0,0,.0415],.011,.015,orange)}
                            {cylinder([0,0,.049],.0045,.082,[.31,.22,.12])}
                            DEF BRUSH_SPRING SliderJoint {{
                              jointParameters JointParameters {{ axis 0 0 1 minStop -0.002 maxStop 0.006
                                springConstant {p['brushK']} dampingConstant {p['brushC']} }}
                              device [ PositionSensor {{ name "brush_deflection_sensor" }} ]
                              endPoint DEF BRUSH_TIP TouchSensor {{
                                translation 0 0 0.001 name "brush_contact" type "force-3d" lookupTable []
                                contactMaterial "brush"
                                children [
                                  # Visual tip slightly smaller than collision sphere:
                                  # Pen ray starts below it, so the tip cannot occlude paper.
                                  {shape('Sphere { radius 0.0008 subdivision 2 }',[.08,.09,.12])}
                                  Pen {{ translation 0 0 -0.0009 name "drawing_pen" write FALSE
                                    inkColor 0.09 0.16 0.27 inkDensity 0.9 leadSize {p['lineWidth0']} maxDistance 0.0005 }}
                                  GPS {{ translation 0 0 -0.001 name "tool_gps" }}
                                ]
                                boundingObject Sphere {{ radius 0.001 }}
                                physics {physics(.01,[0,0,0],[.002,.002,.002])}
                              }}
                            }}
                          ]
                          boundingObject Pose {{ translation 0 0 .049 children [ Cylinder {{ radius .0045 height .082 }} ] }}
                          physics {physics(p['massTool']-.01,[0,0,.045],[.009,.009,.082])}
                        }}
                      ]
                      boundingObject {box([0,l2/2,.053],[.028,l2,.029],black,True)}
                      physics {physics(p['mass2'],[0,l2/2,.053],[.028,l2,.029],p['rotorInertia'][1])}
                    }}
                  }}
                ]
                boundingObject {box([0,l1/2,.083],[.028,l1,.029],black,True)}
                physics {physics(p['mass1'],[0,l1/2,.083],[.028,l1,.029],p['rotorInertia'][0])}
              }}
            }}
          ]
          boundingObject {box([.036,.020,.045],[.118,.042,.053],black,True)}
          physics {physics(carriage_mass,[.036,.020,.045],[.118,.042,.053])}
        }}
      }}
    ]
    # Fixed pedestal; moving endPoints above have their own Physics.
    boundingObject {box([.036,-.019,.15],[.176,.056,.30],metal,True)}
  }}
}}
'''
    (WEBOTS/'protos/ScaraBrush.proto').write_text(HEADER+proto, encoding='utf-8')
    hx, hy = p['paperHalfSize']
    paper = f'''# Paper top exactly Z=0; explicit UV coordinates for native Pen drawing.
PROTO DrawingPaper [] {{
  Solid {{
    name "Drawing paper" contactMaterial "paper"
    children [
      Shape {{
        appearance PBRAppearance {{ baseColor 1 1 1 metalness 0 roughness 1
          baseColorMap ImageTexture {{ url [ "../../common/assets/textures/paper.png" ] }} }}
        geometry IndexedFaceSet {{
          coord Coordinate {{ point [ {-hx} {-hy} 0, {hx} {-hy} 0, {hx} {hy} 0, {-hx} {hy} 0 ] }}
          coordIndex [ 0 1 2 3 -1 ]
          texCoord TextureCoordinate {{ point [ 0 0, 1 0, 1 1, 0 1 ] }}
          texCoordIndex [ 0 1 2 3 -1 ]
        }}
      }}
      {box([0,0,-.002],[2*hx,2*hy,.0039],[.83,.84,.86])}
    ]
    boundingObject {box([0,0,-.0015],[2*hx,2*hy,.003],black,True)}
  }}
}}
'''
    (WEBOTS/'protos/DrawingPaper.proto').write_text(HEADER+paper, encoding='utf-8')
    world = f'''EXTERNPROTO "../protos/ScaraBrush.proto"
EXTERNPROTO "../protos/DrawingPaper.proto"

WorldInfo {{
  title "X-SCARA brush drawing | 2R + Z"
  info [ "STL graphics from madl3x/x-scara GPL-3.0; estimated assembly and physical parameters." ]
  basicTimeStep {p['dt']*1000:.12g}
  coordinateSystem "ENU" gravity {p['gravity']} randomSeed 41
  CFM 0.000001 ERP 0.2 lineScale 0.02
  contactProperties [
    ContactProperties {{ material1 "brush" material2 "paper" coulombFriction [ {p['brushMu']} ]
      bounce 0 softERP 0.4 softCFM 0.000001 bumpSound "" rollSound "" slideSound "" }}
  ]
}}
Viewpoint {{ orientation {vec(camera_orientation([.35,.38,.32],[.01,-.06,.09]))} position 0.35 0.38 0.32 near 0.005 }}
Background {{ skyColor [ 0.91 0.94 0.97 ] }}
DirectionalLight {{ direction -0.3 0.5 -1 intensity 1 ambientIntensity 0.4 castShadows TRUE }}
DirectionalLight {{ direction 0.5 -0.4 -1 intensity 0.45 }}
Solid {{
  translation 0 -0.07 -0.021 name "Work table"
  children [ {shape('Box { size 0.45 0.40 0.035 }',[.55,.60,.64])} ]
  boundingObject Box {{ size 0.45 0.40 0.035 }}
}}
DrawingPaper {{}}
DEF SCARA ScaraBrush {{}}
'''
    (WEBOTS/'worlds/scara_draw.wbt').write_text(HEADER+world, encoding='utf-8')
    print(f'Built scara_draw.wbt, 2 PROTOs, {len(merged)} meshes from {len(parts)} STL instances.')
    print('Coordinates: m/rad, ENU; brush free tip Z=0 is paper top.')


if __name__ == '__main__':
    build()
