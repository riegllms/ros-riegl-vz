import signal
import subprocess
import time
import re
import csv

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
            msg = ''.join([errorMessage + '\n', outs, errs])
            raise RuntimeError(msg)
        return True

def parseCSV(csvFilepath: str):
    delimiter = None
    # auto detect delimiter
    with open(csvFilepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if delimiter is None:
                for d in (',', ';', ':'):
                    values = re.split(d+"\\s*", line)
                    if len(values) > 3:
                        delimiter = d
                        break
            if delimiter is None:
                raise RuntimeError("Unable to detect CSV delimiter.")
            break
    content = []
    with open(csvFilepath, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=delimiter)
        for row in reader:
            content.append(row)
    return content
