"""Run offline tests and record honest validation scope, without Webots installed."""
import json
from pathlib import Path
import unittest

PROJECT = Path(__file__).resolve().parents[1]


if __name__ == '__main__':
    suite = unittest.defaultTestLoader.discover(str(PROJECT/'tests'))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    folder = PROJECT/'output/validation'
    folder.mkdir(parents=True, exist_ok=True)
    (folder/'offline_validation.json').write_text(json.dumps({
        'passed': result.wasSuccessful(), 'tests': result.testsRun,
        'failures': len(result.failures), 'errors': len(result.errors),
        'scope': 'Python algorithms, controller I/O mocks, R2025a field schema, transforms and asset references',
        'webots_launch_tested': False, 'webots_physics_tested': False,
        'next_step': 'Open worlds/scara_draw.wbt in Webots and test position and pid modes.'
    }, indent=2)+'\n',encoding='utf-8')
    raise SystemExit(0 if result.wasSuccessful() else 1)
