"""Offline regressions; never connect to or program a physical target."""
import base64
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import targets
import server


def memory_for(device_id, size=512):
    p = targets.TARGETS[device_id]
    return {0xE000ED00: [0x410F0000 | (p.part << 4)],
            0x5C001000 if p.part == 0xC27 else 0xE0042000: [0x10000000 | device_id],
            p.size_address & ~3: [size << ((p.size_address & 3)*8)],
            p.uid_address: [0x12345678, 0x23456789, 0x34567890]}


def bank(base, length, index=0):
    return f'#{index} : driver at 0x{base:08x}, size 0x{length:x}, buswidth 0, chipwidth 0\n\t#  0: 0x00000000 (0x{length:x} 128kB) not protected\n'


class DetectionTests(unittest.TestCase):
    def test_all_registered_dies(self):
        for device_id, expected in targets.TARGETS.items():
            with self.subTest(id=hex(device_id)):
                data = memory_for(device_id, 1024 if expected.family == 'H7' else 128)
                profile, info = targets.identify(lambda address, *_: data[address])
                self.assertEqual(profile.family, expected.family)
                self.assertEqual(profile.cpu, expected.cpu)
                self.assertEqual(info['uid'][0], '12345678')

    def test_unknown_does_not_guess_g4(self):
        data = memory_for(0x469)
        data[0xE0042000] = [0x12345ABC]
        with self.assertRaises(ValueError): targets.identify(lambda a, *_: data[a])

    def test_core_and_die_must_match(self):
        data = memory_for(0x469)
        data[0xE000ED00] = [0x410FC230]
        with self.assertRaises(ValueError): targets.identify(lambda a, *_: data[a])

    def test_invalid_capacity(self):
        for size in (0, 0xFFFF, 513):
            data = memory_for(0x469, size)
            with self.assertRaises(ValueError): targets.identify(lambda a, *_: data[a])

    def test_h7_config_tracks_physical_banks(self):
        for dev, size, dual in ((0x450,128,False),(0x450,1024,True),(0x450,2048,True),
                                 (0x480,1024,True),(0x483,1024,False)):
            data = memory_for(dev, size)
            profile, _ = targets.identify(lambda a, *_: data[a])
            self.assertEqual('dual_bank' in profile.config, dual)

    def test_uid_unavailable_is_explicit(self):
        data = memory_for(0x469)
        del data[targets.G4.uid_address]
        _, info = targets.identify(lambda a, *_: data[a])
        self.assertEqual(info['uid'], [])
        self.assertTrue(info['warning'])

    def test_h7_hole_and_bank2(self):
        ranges = targets.flash_ranges([bank(0x08000000,0x80000),bank(0x08100000,0x80000,1)],1024)
        self.assertTrue(targets.contains_image(ranges,0x08100000,0x80000))
        self.assertFalse(targets.contains_image(ranges,0x0807FFFF,2))
        self.assertFalse(targets.contains_image(ranges,0x08080000,1))

    def test_g4_reduced_density_bank_hole(self):
        info = bank(0x08000000,0x10000) + '\t#  1: 0x00040000 (0x10000 64kB) not protected\n'
        ranges = targets.flash_ranges([info],128)
        self.assertEqual(ranges,[(0x08000000,0x08010000),(0x08040000,0x08050000)])

    def test_geometry_cannot_invent_capacity(self):
        with self.assertRaises(ValueError): targets.flash_ranges([bank(0x08000000,0x80000)],256)
        with self.assertRaises(ValueError): targets.flash_ranges(['unavailable'],512)

    def test_bin_rejects_hole_before_program_command(self):
        debugger = server.Debugger()
        debugger.identity = {'flash_kib':1024}
        debugger.flash_ranges = [(0x08000000,0x08080000),(0x08100000,0x08180000)]
        debugger.command = Mock()
        with self.assertRaises(ValueError):
            debugger.flash(dict(name='app.bin',data=base64.b64encode(b'abc').decode(),
                                address='0x08080000',erase='covered'))
        debugger.command.assert_not_called()

    def test_bundled_configurations_parse_without_hardware(self):
        scripts = server.OCD/'share/openocd/scripts'
        configs = {p.config for p in targets.TARGETS.values()} | {'stm32h7x_dual_bank.cfg'}
        for cfg in configs:
            with self.subTest(config=cfg):
                result = subprocess.run([str(server.OCD/'bin/openocd.exe'),'-s',str(scripts),
                    '-c','adapter driver cmsis-dap','-c','transport select swd',
                    '-c',f'source [find target/{cfg}]','-c','shutdown'],
                    capture_output=True, timeout=10)
                self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))


class SessionTests(unittest.TestCase):
    def test_auto_connection_then_actions_reuse_session(self):
        for device_id in (0x410,0x413,0x469,0x450):
            with self.subTest(id=hex(device_id)):
                debugger = server.Debugger()
                memory = memory_for(device_id,128)
                state = ['running']
                configs = []
                commands = []
                def start(config):
                    configs.append(config)
                    debugger.process = Mock()
                    debugger.process.poll.return_value = None
                def command(cmd, timeout=5):
                    commands.append(cmd)
                    if 'read_memory' in cmd:
                        return ' '.join(str(v) for v in memory[int(cmd.split()[2])])
                    if cmd.endswith('curstate'): return state[0]
                    if cmd == 'halt 2000': state[0] = 'halted'
                    if cmd == 'resume': state[0] = 'running'
                    if cmd.startswith('get_reg'): return 'pc 0x08000100 sp 0x20001000'
                    if cmd == 'llength [flash list]': return '2'
                    if cmd == 'dict get [lindex [flash list] 0] base': return '0x08000000'
                    if cmd == 'dict get [lindex [flash list] 1] base': return '0x1fff7000'
                    if cmd == 'flash info 0 sectors': return bank(0x08000000,128*1024)
                    return ''
                debugger.start_session = start
                debugger.command = command
                result = debugger.connect({'transport':'usb','speed':100})
                self.assertTrue(result['connected'])
                self.assertEqual(len(configs),2)
                self.assertEqual(state[0],'running')
                debugger.action('halt',{})
                debugger.action('registers',{})
                debugger.action('step',{})
                debugger.action('resume',{})
                self.assertEqual(len(configs),2)
                self.assertTrue(any(c == debugger.target+' arp_poll' for c in commands))
                self.assertFalse(any('program ' in c for c in commands))
                self.assertNotIn('flash probe 1',commands)
                self.assertNotIn('flash info 1 sectors',commands)
                debugger.close()


if __name__ == '__main__': unittest.main()
