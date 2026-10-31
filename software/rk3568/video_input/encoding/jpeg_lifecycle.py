"""Join encoder workers before interpreter exit; keep the proven encoding path."""
import queue
import threading
from hw_jpeg import JpegEncoder


class JoinedJpegEncoder(JpegEncoder):
    def _stop(self):
        proc, self._proc = self._proc, None
        pending = self._in_q
        if pending is not None:
            try:
                while True:
                    pending.get_nowait()
            except queue.Empty:
                pass
            pending.put_nowait(None)
        if proc is not None:
            # Stop the pipe consumer first: releases a writer blocked in os.write.
            if proc.poll() is None:
                proc.terminate()
            try:
                proc.wait(timeout=2)
            except Exception:
                proc.kill()
                proc.wait(timeout=2)
        for worker in (self._writer, self._reader):
            if worker is not None and worker is not threading.current_thread():
                worker.join(timeout=3)
                if worker.is_alive():
                    raise RuntimeError("JPEG worker failed to stop")
        if proc is not None:
            for stream in (proc.stdin, proc.stdout, proc.stderr):
                if stream is not None:
                    stream.close()
        self._writer = self._reader = None
