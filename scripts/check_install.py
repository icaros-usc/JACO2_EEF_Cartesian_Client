"""Assert every bundled package resolves inside this workspace, not an old overlay."""
from pathlib import Path
import json
from ament_index_python.packages import get_package_prefix

root = Path(__file__).resolve().parent.parent
expected = ('jaco_vla', 'ada_description', 'ada_hardware', 'ada_moveit',
            'ada_imu', 'ada_calibrate_camera')
origins = {}
for name in expected:
    prefix = Path(get_package_prefix(name)).resolve()
    if not prefix.is_relative_to((root/'install').resolve()):
        raise RuntimeError(f'{name} resolves outside this workspace: {prefix}')
    origins[name] = str(prefix)
print(json.dumps(origins, indent=2))
