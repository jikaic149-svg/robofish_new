"""Check the firmware control-layer dependency boundaries without a compiler."""

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "stm32f405" / "src"
CONTROL = SOURCE / "control"
CHILDREN = {"tail", "propulsion", "pectoral"}
INCLUDE = re.compile(r'^\s*#\s*include\s*["<]([^">]+)[">]', re.MULTILINE)
FORBIDDEN = re.compile(
    r"(?:^|/)(?:task|device|bsp|service|protocol|network|mode)(?:/|$)"
    r"|(?:^|/)(?:FreeRTOS|queue|semphr|task)\.h$",
    re.IGNORECASE,
)


def includes(path):
    text = path.read_text(encoding="utf-8")
    # Comments must not conceal an include or create a spurious dependency.
    text = re.sub(r"/\*.*?\*/|//[^\n]*", "", text, flags=re.DOTALL)
    return INCLUDE.findall(text), text


def normalized_include(path, name):
    candidate = SOURCE / name
    if not candidate.exists():
        candidate = path.parent / name
    try:
        return candidate.resolve().relative_to(SOURCE.resolve()).as_posix()
    except ValueError:
        return name.replace("\\", "/")


def check():
    errors = []
    checked = 0
    root_children = set()
    for path in sorted(CONTROL.rglob("*")):
        if path.suffix not in {".h", ".cpp"}:
            continue
        checked += 1
        relative = path.relative_to(SOURCE).as_posix()
        parts = path.relative_to(CONTROL).parts
        headers, text = includes(path)
        for header in headers:
            dependency = normalized_include(path, header)
            if parts[0] in {"l0", "l1", "l2"} and FORBIDDEN.search(dependency):
                errors.append(f"{relative}: forbidden runtime/hardware dependency {header}")
            if parts[0] == "l1" and re.search(r"(?:^|/)control/l[023]/", dependency):
                errors.append(f"{relative}: L1 depends on another control layer: {header}")
            if parts[0] == "l0" and re.search(r"(?:^|/)control/l[23]/", dependency):
                errors.append(f"{relative}: L0 bypasses its L1 boundary: {header}")
            if parts[0] == "l2" and re.search(r"(?:^|/)control/l[03]/", dependency):
                errors.append(f"{relative}: L2 depends on strategy or execution: {header}")
            child = re.search(r"(?:^|/)control/l2/([^/]+)/", dependency)
            if child and child.group(1) in CHILDREN:
                node = child.group(1)
                if parts[0] == "l2" and len(parts) > 2 and parts[1] in CHILDREN:
                    if parts[1] != node:
                        errors.append(f"{relative}: leaf calls sibling subtree {header}")
                elif relative == "control/l2/Allocator.cpp":
                    root_children.add(node)
                else:
                    errors.append(f"{relative}: only L2 root may access leaf {header}")
        if parts[0] == "l2" and len(parts) > 2 and parts[1] in CHILDREN:
            for sibling in CHILDREN - {parts[1]}:
                if re.search(rf"\b{sibling}\s*::", text):
                    errors.append(f"{relative}: direct sibling call {sibling}::")
    # Task, Device and other source areas must also enter at the L2 root, never
    # bypass allocation policy by directly including one of its child nodes.
    for path in sorted(SOURCE.rglob("*")):
        if path.suffix not in {".h", ".cpp"} or path.is_relative_to(CONTROL):
            continue
        headers, _ = includes(path)
        for header in headers:
            dependency = normalized_include(path, header)
            if re.search(r"(?:^|/)control/l2/(?:tail|propulsion|pectoral)/", dependency):
                errors.append(f"{path.relative_to(SOURCE).as_posix()}: external leaf access {header}")
    if root_children != CHILDREN:
        errors.append(f"L2 root child set must be {sorted(CHILDREN)}, got {sorted(root_children)}")
    if not checked:
        errors.append("control source tree is missing")
    if errors:
        for error in errors:
            print("FAIL:", error, file=sys.stderr)
        return 1
    print(f"PASS: control dependency boundaries ({checked} files); L2 root -> tail / propulsion / pectoral")
    return 0


if __name__ == "__main__":
    raise SystemExit(check())
