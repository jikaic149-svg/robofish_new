"""Thread-safe operation progress, available while the debugger lock is held."""
import threading
import time


class Progress:
    def __init__(self):
        self.lock = threading.Lock()
        self.data = {'revision':0, 'kind':None, 'state':'idle'}

    def update(self, **values):
        with self.lock:
            self.data.update(values)
            self.data['revision'] = max(self.data['revision']+1, time.time_ns()//1000)

    def begin(self, kind):
        self.update(kind=kind, state='active', phase='准备与检查', percent=None, detail='')

    def stage(self, phase, percent=None, detail=''):
        self.update(phase=phase, percent=percent, detail=detail)

    def finish(self, error=None):
        self.update(state='failed' if error else 'complete', percent=None if error else 100,
                    phase='未完成' if error else '完成', detail=str(error or '操作已确认完成'))

    def snapshot(self):
        with self.lock:
            return dict(self.data)


PROGRESS = Progress()
