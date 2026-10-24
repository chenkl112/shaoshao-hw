import importlib.util
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace, ModuleType
import unittest
from unittest.mock import patch
import yaml

SOURCE=Path(__file__).resolve().parents[1]

class MappingLaunchTests(unittest.TestCase):
    def launch(self,root):
        launch=ModuleType('launch');launch.LaunchDescription=list
        ros=ModuleType('launch_ros');actions=ModuleType('launch_ros.actions')
        actions.Node=lambda **kwargs:SimpleNamespace(**kwargs);ros.actions=actions
        spec=importlib.util.spec_from_file_location('mapping_launch_regression',SOURCE/'deployment/mapping_runtime/mapping.launch.py')
        module=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,{'launch':launch,'launch_ros':ros,'launch_ros.actions':actions}),patch.dict(os.environ,{'SHAOSHAO_HW_RUNTIME':str(root)}):
            spec.loader.exec_module(module)
            return module.generate_launch_description()

    def test_recorder_uses_lio_session_while_marker_still_points_to_preview(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'mapping_runtime').mkdir()
            preview=root/'preview';session=root/'new-map'
            (root/'current_mapping_dir').write_text(str(preview))
            config={'/**':{'ros__parameters':{'map_file_path':str(session/'site.pcd')}}}
            (root/'mapping_runtime/fast_lio.yaml').write_text(yaml.safe_dump(config))
            nodes=self.launch(root)
            recorder=next(n for n in nodes if n.executable=='index_recorder')
            self.assertEqual(Path(recorder.parameters[0]['maps_dir']),session)
            self.assertEqual((root/'current_mapping_dir').read_text(),str(preview))

    def test_legacy_config_without_map_filename_uses_session_marker(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'mapping_runtime').mkdir();session=root/'saved-map'
            (root/'current_mapping_dir').write_text(str(session))
            (root/'mapping_runtime/fast_lio.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':{}}}))
            nodes=self.launch(root)
            self.assertEqual(Path(nodes[0].parameters[0]['maps_dir']),session)

if __name__=='__main__':unittest.main()
