import hashlib
import json
import math
from pathlib import Path
import sys
import unittest

PROJECT = Path(__file__).resolve().parents[1]
SIMULATION = PROJECT.parent
sys.path.insert(0,str(PROJECT/'tools'))
sys.path.insert(0,str(PROJECT/'controllers/scara_draw'))
from build_project import read_stl
from vrml_check import Parser, load_schema
from core.model import fingerprint, fk


class ProjectTests(unittest.TestCase):
    def setUp(self):
        self.schema = load_schema(PROJECT)
        self.p = json.loads((SIMULATION/'common/config/scara.json').read_text(encoding='utf-8'))
        self.robot = Parser(PROJECT/'protos/ScaraBrush.proto',self.schema)
        self.robot.parse()

    def test_node_syntax_fields_and_relative_assets(self):
        for path in [PROJECT/'worlds/scara_draw.wbt', *sorted((PROJECT/'protos').glob('*.proto'))]:
            self.assertEqual(path.read_text(encoding='utf-8').splitlines()[0],'#VRML_SIM R2025a utf8')
            parsed = Parser(path,self.schema)
            parsed.parse()
            parsed.check_assets()
        self.assertEqual(self.robot.nodes[0].fields['customData'],fingerprint(self.p))

    def test_initial_joint_transform_consistency_and_devices(self):
        defs = self.robot.defs
        q = [defs[name].fields['jointParameters'].fields['position'] for name in ('SHOULDER_JOINT','ELBOW_JOINT','Z_JOINT')]
        self.assertAlmostEqual(defs['SHOULDER'].fields['rotation'][3],q[0])
        self.assertAlmostEqual(defs['ELBOW'].fields['rotation'][3],q[1])
        self.assertAlmostEqual(defs['CARRIAGE'].fields['translation'][2],q[2])
        self.assertLess(math.dist(fk(q,self.p),[.034,0,self.p['penLift']]),1e-11)
        names = {n.fields.get('name') for n in self.robot.nodes}
        self.assertTrue({'shoulder_motor','elbow_motor','z_motor','shoulder_sensor','elbow_sensor',
                         'z_sensor','brush_contact','brush_deflection_sensor','tool_gps','drawing_pen'} <= names)
        touch = defs['BRUSH_TIP']
        self.assertEqual(touch.fields['type'],'force-3d')
        self.assertEqual(touch.fields['lookupTable'],[])  # true newtons, not default x10
        pen = next(n for n in self.robot.nodes if n.kind == 'Pen')
        self.assertGreater(pen.fields['maxDistance'],0)
        self.assertLess(pen.fields['maxDistance'],self.p['penLift'])
        visual_tip = touch.fields['children'][0].fields['geometry']
        self.assertLess(pen.fields['translation'][2],-visual_tip.fields['radius'])

    def test_physics_mass_inertias_limits_and_paper(self):
        physical = [n for n in self.robot.nodes if n.kind == 'Physics']
        self.assertAlmostEqual(sum(n.fields['mass'] for n in physical),self.p['zMass'])
        for node in physical:
            inertia = node.fields['inertiaMatrix'][0]
            self.assertTrue(all(v > 0 for v in inertia))
            for j in range(3):
                self.assertLessEqual(inertia[j],sum(inertia)-inertia[j]+1e-12)
        for joint in [n for n in self.robot.nodes if n.kind in ('SliderJoint','HingeJoint')]:
            params = joint.fields['jointParameters'].fields
            position = params.get('position',0)
            self.assertLessEqual(params.get('minStop',0),position)
            self.assertGreaterEqual(params.get('maxStop',0),position)
            if joint.kind == 'HingeJoint':
                self.assertLessEqual(params['minStop'],0)
                self.assertGreaterEqual(params['maxStop'],0)
        paper = Parser(PROJECT/'protos/DrawingPaper.proto',self.schema)
        paper.parse()
        faces = next(n for n in paper.nodes if n.kind == 'IndexedFaceSet')
        self.assertEqual(faces.fields['coordIndex'],faces.fields['texCoordIndex'])
        for node in paper.nodes:
            if node.kind == 'Coordinate':
                self.assertTrue(all(v[2] == 0 for v in node.fields['point']))

    def test_source_hashes_scale_and_meshes(self):
        manifest = json.loads((SIMULATION/'common/assets/manifest.json').read_text(encoding='utf-8'))
        self.assertEqual(len(manifest['parts']),11)
        expected = {}
        for part in manifest['parts']:
            source = SIMULATION.parent / part['source']
            self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(),part['sha256'])
            expected[part['output']] = expected.get(part['output'],0) + part['triangles']
        for name, count in expected.items():
            triangles = read_stl(SIMULATION/'common/assets/meshes'/name)
            self.assertEqual(len(triangles),count)
            self.assertTrue(all(math.isfinite(v) and abs(v) < .5 for tri in triangles for point in tri for v in point))

    def test_camera_faces_the_robot(self):
        world = Parser(PROJECT/'worlds/scara_draw.wbt',self.schema)
        world.parse()
        view = next(n for n in world.nodes if n.kind == 'Viewpoint')
        x,y,z,angle = view.fields['orientation']
        # Rodrigues rotate local +X camera viewing axis into world coordinates.
        c,s = math.cos(angle),math.sin(angle)
        direction = [c+x*x*(1-c),z*s+x*y*(1-c),-y*s+x*z*(1-c)]
        delta = [.01-view.fields['position'][0],-.06-view.fields['position'][1],.09-view.fields['position'][2]]
        norm = math.sqrt(sum(v*v for v in delta))
        self.assertGreater(sum(a*b/norm for a,b in zip(direction,delta)),.999999999)


if __name__ == '__main__':
    unittest.main()
