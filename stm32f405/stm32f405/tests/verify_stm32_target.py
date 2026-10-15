from pathlib import Path
import argparse
import json
import re
import struct
import subprocess

ROOT = Path(__file__).resolve().parents[1]
IOC = (ROOT / "RoboFishF405.ioc").read_text(encoding="utf-8")
BOARD = (ROOT / "bsp" / "BoardResources.h").read_text(encoding="utf-8")
MAIN = (ROOT / "Core" / "Src" / "main.cpp").read_text(encoding="utf-8")
INIT = (ROOT / "bsp" / "BoardInit.cpp").read_text(encoding="utf-8")
TASKS = (ROOT.parent / "src" / "task" / "RobotTasks.cpp").read_text(encoding="utf-8")

def required(text: str, needles: list[str], source: str) -> None:
    missing = [needle for needle in needles if needle not in text]
    if missing:
        raise AssertionError(f"{source} missing: {', '.join(missing)}")

required(
    IOC,
    [
        "Mcu.CPN=STM32F405RGT6",
        "RCC.HSE_VALUE=8000000",
        "RCC.SYSCLKFreq_VALUE=168000000",
        "PA0.Signal=TIM5_CH1",
        "PA1.Signal=TIM5_CH2",
        "PA15.Signal=TIM2_CH1",
        "PA8.Signal=TIM1_CH1",
        "PB0.Signal=TIM3_CH3",
        "PB1.Signal=TIM3_CH4",
        "PB8.Signal=TIM4_CH3",
        "PA9.Signal=USART1_TX",
        "PA10.Signal=USART1_RX",
        "PA13.Signal=SYS_JTMS-SWDIO",
        "PA14.Signal=SYS_JTCK-SWCLK",
    ],
    "RoboFishF405.ioc",
)
required(
    BOARD,
    [
        "kTailServo1 = {BoardPin::kPa1, 2, PwmTimer::kTim5",
        "kTailServo2 = {BoardPin::kPa15, 1, PwmTimer::kTim2",
        "kPectoralLeftServo = {BoardPin::kPa8, 1, PwmTimer::kTim1",
        "kPectoralRightServo = {BoardPin::kPa0, 1, PwmTimer::kTim5",
        "kPropulsionLeft = {BoardPin::kPb0, 3, PwmTimer::kTim3",
        "kPropulsionRight = {BoardPin::kPb1, 4, PwmTimer::kTim3",
    ],
    "BoardResources.h",
)
required(
    MAIN,
    [
        "robofish::bsp::initializeBoard()",
        "robofish::task::startRobotTasks()",
        "SysTick_Handler()",
        "vTaskStartScheduler()",
    ],
    "main.cpp",
)

required(INIT, ["GPIO_AF2_TIM5", "HAL_TIM_PWM_MspInit", "HAL_UART_RxCpltCallback",
                "HAL_UART_ErrorCallback", "DMA1_Stream7", "DMA2_Stream7"], "BoardInit.cpp")
required(TASKS, ["gMotion.update()", "gMotion.observeLoRa(gLoRa.status())",
                 "command.code == 8 || command.code == 18 || command.code == 19",
                 "gDriver.begin()", "kEscArmMs"], "RobotTasks.cpp")

assert "PB10.Signal=" not in IOC and "PB11.Signal=" not in IOC
assert "GPIO_PIN_11" not in INIT
assert not (ROOT / "bsp/PumpGpio.cpp").exists()
resources = re.findall(r"ServoResource\s+\w+\s*=\s*\{BoardPin::(\w+),\s*(\d+),\s*PwmTimer::(\w+)", BOARD)
assert len(resources) == 6 and len({(timer, channel) for _, channel, timer in resources}) == 6
assert len({pin for pin, _, _ in resources}) == 6

hardware = json.loads((ROOT / "hardware-map.json").read_text(encoding="utf-8"))
named_resources = {
    name: (pin, int(channel), timer)
    for name, pin, channel, timer in re.findall(
        r"ServoResource\s+(\w+)\s*=\s*\{BoardPin::(\w+),\s*(\d+),\s*PwmTimer::(\w+)", BOARD
    )
}
for mapping in hardware["mappings"]:
    assert named_resources[mapping["firmware_resource"]] == (
        "k" + mapping["mcu_pin"].title(),
        mapping["channel"],
        "k" + mapping["timer"].title(),
    ), mapping["role"]

parser = argparse.ArgumentParser()
parser.add_argument("--build-dir", type=Path)
parser.add_argument("--nm", default="arm-none-eabi-nm")
parser.add_argument("--image", default="RoboFishF405")
parser.add_argument("--base", type=lambda value: int(value, 0), default=0x08010000)
parser.add_argument("--boot", action="store_true")
args = parser.parse_args()
if args.build_dir:
    binary = (args.build_dir / (args.image + ".bin")).read_bytes()
    elf = args.build_dir / (args.image + ".elf")
    header = elf.read_bytes()[:52]
    assert header[:6] == b"\x7fELF\x01\x01" and struct.unpack_from("<H", header, 18)[0] == 40
    assert struct.unpack_from("<I", header, 36)[0] & 0x400, "hard-float ABI required"
    symbols = {}
    for line in subprocess.check_output([args.nm, str(elf)], text=True).splitlines():
        fields = line.split()
        if len(fields) == 3:
            symbols[fields[2]] = int(fields[0], 16)
    assert struct.unpack_from("<I", binary)[0] == 0x2001FFC0
    assert len(binary) <= (32*1024 if args.boot else 448*1024)
    assert symbols["_ram_usage_end"] <= 0x2001FFC0
    if not args.boot:
        retained = [address for name, address in symbols.items() if "faultRecord" in name]
        assert retained == [0x2001FFC0], retained
        disassembler = args.nm.replace("-nm", "-objdump")
        code = subprocess.check_output([disassembler, "-d", "--disassemble=Reset_Handler", str(elf)], text=True)
        first_call = re.search(r"\bbl\s+[0-9a-f]+\s+<([^>]+)>", code)
        assert first_call and first_call.group(1) == "robofish_early_watchdog_start", code
    vectors = [(1, "Reset_Handler")] if args.boot else [(1, "Reset_Handler"), (3, "HardFault_Handler"), (11, "SVC_Handler"),
                         (14, "PendSV_Handler"), (15, "SysTick_Handler"),
                         (16 + 14, "DMA1_Stream3_IRQHandler"),
                         (16 + 18, "ADC_IRQHandler"),
                         (16 + 56, "DMA2_Stream0_IRQHandler"),
                         (16 + 15, "DMA1_Stream4_IRQHandler"),
                         (16 + 36, "SPI2_IRQHandler"),
                         (16 + 40, "EXTI15_10_IRQHandler"),
                         (16 + 37, "USART1_IRQHandler"),
                         (16 + 47, "DMA1_Stream7_IRQHandler"),
                         (16 + 70, "DMA2_Stream7_IRQHandler")]
    for vector, name in vectors:
        address = struct.unpack_from("<I", binary, vector * 4)[0]
        assert address == (symbols[name] | 1), (vector, name, hex(address))
        assert address != (symbols["Default_Handler"] | 1), name
        assert args.base <= address < args.base + len(binary)
    assert "pvPortMalloc" not in symbols, "target must use static RTOS allocation"
    print(f"PASS: {args.image}, ARM/ABI/vectors, fixed layout and early watchdog; {len(binary)} flash bytes")
print("PASS: six independent PWM channels, EDA pin mapping, pump absent, maintenance calls")
