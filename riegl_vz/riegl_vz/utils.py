import signal
import subprocess
import time

class SubProcess(object):
    def __init__(
        self,
        proc: subprocess.Popen):
        self.canceled = False
        self.proc = proc

    def cancel(self):
        self.canceled = True

    def waitFor(
        self,
        errorMessage: str,
        block: bool = True):
        if block:
            while self.proc.poll() is None:
                if self.canceled:
                    self.proc.terminate()
                    break
                else:
                    time.sleep(0.2)
        else:
            if self.proc.poll() is None:
                if self.canceled:
                    self.proc.terminate()
            else:
                return False
        if not self.canceled and self.proc.returncode != 0:
            outs, errs = self.proc.communicate()
            if not isinstance(outs, str):
                outs = outs.decode()
            if not isinstance(errs, str):
                errs = errs.decode()
            msg = "".join([errorMessage + "\n", outs, errs])
            raise RuntimeError(msg)
        return True
