"""Source freshness checks never load or replace the running service's DLL."""
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import Mock,patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import motion_model
from motion_model import _needs_rebuild


def timestamp(value):
    path=Mock(spec=Path)
    path.stat.return_value=SimpleNamespace(st_mtime_ns=value)
    return path


class MotionModelFreshnessTests(unittest.TestCase):
    def test_missing_library_requires_build(self):
        library=Mock(spec=Path)
        library.stat.side_effect=FileNotFoundError
        self.assertTrue(_needs_rebuild(library,[]))

    def test_any_newer_dependency_requires_build(self):
        self.assertTrue(_needs_rebuild(timestamp(100),[timestamp(90),timestamp(101)]))

    def test_equal_or_older_sources_keep_existing_library(self):
        self.assertFalse(_needs_rebuild(timestamp(100),[timestamp(90),timestamp(100)]))

    def test_loaded_function_is_never_rebuilt_or_hot_reloaded(self):
        loaded=object()
        with patch.object(motion_model,'_loaded_function',loaded), \
                patch.object(motion_model.subprocess,'run') as build, \
                patch.object(motion_model.ctypes,'CDLL') as load:
            self.assertIs(motion_model.allocator(),loaded)
        build.assert_not_called()
        load.assert_not_called()


if __name__=='__main__':
    unittest.main()
