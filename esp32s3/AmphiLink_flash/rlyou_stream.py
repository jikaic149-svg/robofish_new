"""Capture subprocess output while forwarding lines as they arrive."""
import subprocess
import threading


def run_stream(args, on_line, timeout=180, env=None):
    proc = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, encoding='utf-8', errors='replace', env=env,
        creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
    output = []
    errors = []
    def reader():
        try:
            for line in proc.stdout:
                output.append(line)
                on_line(line.rstrip())
        except Exception as exc:
            errors.append(exc)
    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    try:
        code = proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
        raise
    finally:
        thread.join(timeout=3)
        if not thread.is_alive():
            proc.stdout.close()
    if errors:
        raise errors[0]
    if thread.is_alive():
        raise RuntimeError('Process output did not close; operation status is uncertain')
    return subprocess.CompletedProcess(args, code, ''.join(output), '')
