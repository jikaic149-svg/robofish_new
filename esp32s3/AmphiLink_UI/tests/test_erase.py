import base64
import unittest
from test_targets import bank
import server


class EraseTests(unittest.TestCase):
    def fixture(self, mode='all', fail=None):
        d=server.Debugger();d.identity={'flash_kib':1024}
        d.flash_ranges=[(0x08000000,0x08080000),(0x08100000,0x08180000)]
        calls=[]
        def command(cmd,timeout=5):
            calls.append(cmd)
            if fail and fail in cmd:raise RuntimeError('injected failure')
            if cmd=='llength [flash list]':return '3'
            for i,base in enumerate((0x08000000,0x1FFF7000,0x08100000)):
                if cmd==f'dict get [lindex [flash list] {i}] base':return hex(base)
                if cmd==f'flash info {i} sectors':return bank(base,0x80000,i)
            return ''
        d.command=command
        d.status=lambda:dict(connected=True,target_state='running')
        body=dict(name='image.bin',data=base64.b64encode(b'abcd').decode(),address='0x08000000',
                  erase=mode,verify=True,run=True)
        return d,body,calls

    def test_all_banks_without_otp_or_second_erase(self):
        d,b,c=self.fixture();d.flash(b)
        self.assertEqual([x for x in c if x.startswith('flash erase_sector')],
                         ['flash erase_sector 0 0 last','flash erase_sector 2 0 last'])
        self.assertNotIn('flash info 1 sectors',c)
        self.assertFalse(any(x.startswith('flash write_image erase') for x in c))
        self.assertTrue(c[0].startswith('test_image'))
        self.assertLess(c.index('flash info 2 sectors'),c.index('flash erase_sector 0 0 last'))

    def test_covered_unchanged(self):
        d,b,c=self.fixture('covered');d.flash(b)
        self.assertFalse(any(x.startswith('flash erase_sector') for x in c))
        self.assertTrue(any(x.startswith('flash write_image erase') for x in c))

    def test_invalid_image_has_no_erase(self):
        d,b,c=self.fixture(fail='test_image')
        with self.assertRaises(RuntimeError):d.flash(b)
        self.assertFalse(any(x.startswith('flash erase_sector') for x in c))

    def test_bank_preflight_failure_has_no_erase(self):
        d,b,c=self.fixture(fail='flash info 2')
        with self.assertRaises(RuntimeError):d.flash(b)
        self.assertFalse(any(x.startswith('flash erase_sector') for x in c))

    def test_erase_failure_does_not_write_or_run(self):
        d,b,c=self.fixture(fail='flash erase_sector 0')
        with self.assertRaises(RuntimeError):d.flash(b)
        self.assertFalse(any(x.startswith('flash write_image') for x in c))
        self.assertNotIn('reset run',c)

    def test_unknown_mode_has_no_target_operations(self):
        d,b,c=self.fixture('bad')
        with self.assertRaises(ValueError):d.flash(b)
        self.assertEqual(c,[])

    def test_standalone_needs_no_file_and_does_not_resume(self):
        d,b,c=self.fixture()
        d.status=lambda:dict(connected=True,target_state='halted')
        result=d.action('erase',{})
        self.assertEqual(result['target_state'],'halted')
        self.assertEqual([x for x in c if x.startswith('flash erase_sector')],
                         ['flash erase_sector 0 0 last','flash erase_sector 2 0 last'])
        self.assertFalse(any('write_image' in x or 'test_image' in x for x in c))
        self.assertNotIn('reset run',c)
        self.assertNotIn('resume',c)


if __name__=='__main__':unittest.main()
