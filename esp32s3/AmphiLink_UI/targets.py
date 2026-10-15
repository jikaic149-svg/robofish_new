"""STM32 identity and flash layout, matched before loading vendor target scripts.

Device IDs and geometry follow the bundled OpenOCD stm32f1x/f2x/l4x/h7x
drivers. IDCODE identifies a die family, not its package or exact order code.
"""
from dataclasses import dataclass, replace
import re


@dataclass(frozen=True)
class Target:
    family: str
    name: str
    config: str
    cpu: str
    part: int
    size_address: int
    uid_address: int
    max_kib: int


F1 = Target('F1', 'STM32F1', 'stm32f1x.cfg', 'stm32f1x.cpu', 0xC23, 0x1FFFF7E0, 0x1FFFF7E8, 1024)
F4 = Target('F4', 'STM32F4', 'stm32f4x.cfg', 'stm32f4x.cpu', 0xC24, 0x1FFF7A22, 0x1FFF7A10, 2048)
G4 = Target('G4', 'STM32G4', 'stm32g4x.cfg', 'stm32g4x.cpu', 0xC24, 0x1FFF75E0, 0x1FFF7590, 512)
H7 = Target('H7', 'STM32H7', 'stm32h7x.cfg', 'stm32h7x.cpu0', 0xC27, 0x1FF1E880, 0x1FF1E800, 2048)

TARGETS = {}
for template, names in [
    (F1, {0x410:'STM32F1 中容量', 0x412:'STM32F1 小容量', 0x414:'STM32F1 大容量',
          0x418:'STM32F105/F107', 0x420:'STM32F100 小/中容量', 0x428:'STM32F100 大容量', 0x430:'STM32F1 XL'}),
    (F4, {0x413:'STM32F405/F407/F415/F417', 0x419:'STM32F42x/F43x', 0x423:'STM32F401xB/C',
          0x433:'STM32F401xD/E', 0x431:'STM32F411', 0x421:'STM32F446', 0x434:'STM32F469/F479',
          0x441:'STM32F412', 0x458:'STM32F410', 0x463:'STM32F413/F423'}),
    (G4, {0x468:'STM32G43x/G44x', 0x469:'STM32G47x/G48x', 0x479:'STM32G49x/G4Ax'}),
    (H7, {0x450:'STM32H74x/H75x', 0x480:'STM32H7Ax/H7Bx', 0x483:'STM32H72x/H73x'}),
]:
    for device_id, name in names.items():
        TARGETS[device_id] = replace(template, name=name)
TARGETS[0x430] = replace(TARGETS[0x430], config='stm32xl.cfg')
TARGETS[0x480] = replace(TARGETS[0x480], size_address=0x08FFF80C, uid_address=0x08FFF800)
TARGETS[0x483] = replace(TARGETS[0x483], max_kib=1024)


def identify(read):
    """Read identity while the caller has halted the core; never guess a driver."""
    cpuid = read(0xE000ED00, 32, 1)[0]
    part = (cpuid >> 4) & 0xFFF
    if cpuid >> 24 != 0x41 or part not in (0xC23, 0xC24, 0xC27):
        raise ValueError(f'暂不支持此内核：CPUID=0x{cpuid:08X}；当前自动适配 STM32 F1/F4/G4/H7')
    id_address = 0x5C001000 if part == 0xC27 else 0xE0042000
    code = read(id_address, 32, 1)[0]
    device_id = code & 0xFFF
    profile = TARGETS.get(device_id)
    if profile is None or profile.part != part:
        raise ValueError(f'未匹配受支持的 STM32：IDCODE=0x{code:08X}，CPUID=0x{cpuid:08X}；未加载烧录驱动')
    word = read(profile.size_address & ~3, 32, 1)[0]
    size = (word >> ((profile.size_address & 3) * 8)) & 0xFFFF
    if not 16 <= size <= profile.max_kib:
        raise ValueError(f'{profile.name} Flash 容量读回无效：{size} KiB；不按最大容量猜测')
    if profile.family == 'H7' and device_id in (0x450, 0x480):
        if size not in (128, 1024, 2048):
            raise ValueError(f'H7 Flash 布局尚未适配：{size} KiB')
        if size > 128:
            profile = replace(profile, config='stm32h7x_dual_bank.cfg')
    uid = []
    warning = ''
    try:
        uid = read(profile.uid_address, 32, 3)
        if all(v == 0 for v in uid) or all(v == 0xFFFFFFFF for v in uid):
            raise ValueError('UID 无效')
    except Exception:
        warning = 'UID 未读出；系列识别基于 IDCODE、CPUID 和容量，不推断完整料号'
        uid = []
    return profile, dict(name=profile.name, family=profile.family, id=f'0x{code:08X}',
        cpuid=f'0x{cpuid:08X}', core={0xC23:'Cortex-M3',0xC24:'Cortex-M4',0xC27:'Cortex-M7'}[part],
        flash_kib=size, uid=[f'{v:08X}' for v in uid], warning=warning,
        config=profile.config, automatic=True, id_address=id_address)


def flash_ranges(bank_info, size):
    """Use driver sector maps: H7 and reduced-density G4 can have bank holes."""
    sectors = []
    for text in bank_info:
        header = re.search(r'(?m)^#\d+\s*:.*? at (0x[0-9a-fA-F]+), size', text)
        if not header:
            raise ValueError('OpenOCD 未返回有效 Flash Bank 信息')
        base = int(header[1], 16)
        entries = re.findall(r'(?m)^\s*#\s*\d+:\s*(0x[0-9a-fA-F]+)\s*\((0x[0-9a-fA-F]+)\s', text)
        if not entries:
            raise ValueError('OpenOCD 未返回有效 Flash 扇区信息')
        for offset, length in entries:
            start = base + int(offset, 16)
            stop = start + int(length, 16)
            if not 0x08000000 <= start < stop <= 0x08400000:
                raise ValueError('Flash 扇区范围异常')
            sectors.append((start, stop))
    merged = []
    for start, stop in sorted(sectors):
        if merged and start < merged[-1][1]:
            raise ValueError('Flash 扇区重叠')
        if merged and start == merged[-1][1]:
            merged[-1] = (merged[-1][0], stop)
        else:
            merged.append((start, stop))
    if sum(b-a for a,b in merged) != size * 1024:
        raise ValueError('Flash 驱动容量与芯片容量寄存器不一致，未启用烧录')
    return merged


def contains_image(ranges, address, length):
    end = address + length
    for start, stop in sorted(ranges):
        if start > address:
            break
        if start <= address < stop:
            address = min(stop, end)
            if address == end:
                return True
    return False
