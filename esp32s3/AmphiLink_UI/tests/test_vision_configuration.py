"""Vision configuration stays within the project and reaches the real launcher."""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock,patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import vision_service


class VisionConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory()
        self.root=Path(self.directory.name)
        (self.root/'VISION/models').mkdir(parents=True)
        (self.root/'VISION/models/fish.pt').write_bytes(b'metadata fixture')
        self.workspace=patch.object(vision_service,'WORKSPACE',self.root);self.workspace.start()
        self.vision=vision_service.VisionService()

    def tearDown(self):
        self.workspace.stop();self.directory.cleanup()

    def test_available_models_and_calibration_state_are_real_files(self):
        (self.root/'VISION/configs').mkdir()
        (self.root/'VISION/configs/marker_profile.local.json').write_text('{"hue_center":8}')
        info=self.vision.configuration()
        self.assertEqual(info['models'][0]['path'],'VISION/models/fish.pt')
        profiles={item['key']:item for item in info['calibration']}
        self.assertTrue(profiles['tail']['saved']);self.assertEqual(profiles['tail']['values']['hue_center'],8)
        self.assertFalse(profiles['pool']['saved'])

    def test_model_selection_rejects_outside_or_non_pt_files(self):
        (self.root/'private.pt').write_bytes(b'outside models')
        for value in ['private.pt','VISION/models/../../private.pt','VISION/models/fish.onnx',12]:
            with self.assertRaises(ValueError):self.vision.model_path(value)
        self.assertEqual(self.vision.model_path(),self.root/'VISION/models/fish.pt')

    def test_selected_model_reaches_process_and_running_parameters_are_locked(self):
        process=Mock();process.poll.return_value=None;process.stdout=[]
        with patch.dict('os.environ',{'ROBOFISH_VISION_PYTHON':sys.executable}),patch.object(vision_service.subprocess,'Popen',return_value=process) as launch,patch.object(self.vision.ready,'wait'):
            state=self.vision.start(2,'test-parent',18767,'VISION/models/fish.pt')
            command=launch.call_args.args[0]
            self.assertEqual(command[command.index('--source')+1],'2')
            self.assertEqual(command[command.index('--model')+1],str(self.root/'VISION/models/fish.pt'))
            self.assertEqual(state['model'],str(self.root/'VISION/models/fish.pt'))
            with self.assertRaisesRegex(RuntimeError,'请先停止'):self.vision.start(3,'test-parent',18767)

    def test_output_link_opens_only_project_capture_directory(self):
        with patch.object(vision_service.os,'startfile',create=True) as open_folder:
            result=self.vision.open_output()
        self.assertTrue((self.root/'build/vision').is_dir())
        open_folder.assert_called_once_with(str(self.root/'build/vision'))
        self.assertTrue(result['opened'])


if __name__=='__main__':unittest.main()
