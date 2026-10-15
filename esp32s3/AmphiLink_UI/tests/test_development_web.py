"""Browser source/import boundaries and the native document bootstrap."""
import base64
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import development


class DevelopmentWebTests(unittest.TestCase):
    def test_host_launch_clears_the_parent_electron_node_mode(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(development,'ROOT',Path(folder)), patch.dict('os.environ',{'ELECTRON_RUN_AS_NODE':'1'}), patch.object(development.subprocess,'Popen') as launch:
            development.start_host()
            workspace=Path(folder)/'build/workstation/ember-host.code-workspace'
            self.assertTrue(workspace.is_file())
            saved=json.loads(workspace.read_text(encoding='utf8'));saved['settings']={'emberprobe.maxSamples':1234}
            workspace.write_text(json.dumps(saved),encoding='utf8')
            development.start_host()
            self.assertEqual(json.loads(workspace.read_text(encoding='utf8'))['settings']['emberprobe.maxSamples'],1234)
        self.assertNotIn('ELECTRON_RUN_AS_NODE',launch.call_args.kwargs['env'])
        self.assertIn('--new-window',launch.call_args.args[0])

    def test_source_allowlist_and_matching_content_hash(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(development,'ROOT',Path(directory)):
            root=Path(directory);(root/'common').mkdir()
            source=root/'common/protocol.h';source.write_bytes(b'int counter;\n')
            value=development.source_file('common/protocol.h')
            self.assertEqual(value['sha256'],hashlib.sha256(value['text'].encode()).hexdigest())
            (root/'private.h').write_text('private')
            with self.assertRaises(development.DevelopmentError):development.source_file('private.h')
            with self.assertRaises(development.DevelopmentError):development.source_file('../private.h')

    def test_import_rejects_non_arm_elf_and_svd_entities(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(development,'ROOT',Path(directory)):
            for name,data in [('x.elf',b'not an ELF'),('x.svd',b'<!DOCTYPE device><device/>'),('x.exe',b'executable')]:
                with self.assertRaises(development.DevelopmentError):development.import_file(name,base64.b64encode(data).decode())
            saved=development.import_file('device.svd',base64.b64encode(b'<device><name>STM32F405</name></device>').decode())
            self.assertTrue(Path(saved['absolute_path']).is_relative_to(Path(directory)/'build/workstation/imports'))

    def test_native_document_replaces_vscode_bootstrap_without_exposing_token(self):
        native={'id':'sidebar','generation':'test-generation','html':'<html><head><meta http-equiv="Content-Security-Policy" content="none"></head><body>native</body></html>'}
        with patch.object(development,'request',return_value=native) as call:
            html,view=development.web_document('sidebar',None,18767)
        self.assertEqual(view,'sidebar')
        self.assertIn(b'/ember-webview.js',html)
        self.assertIn(b'test-generation',html)
        self.assertNotIn(b'Content-Security-Policy',html)
        call.assert_called_once_with('/web/create',{'kind':'sidebar','id':None,'origin':'http://127.0.0.1:18767/'},timeout=12)


if __name__=='__main__':unittest.main()
