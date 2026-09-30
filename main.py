"""Workspace-level launcher; also works when VS Code opens the outer LQR folder."""
from pathlib import Path
import runpy
import sys

if __name__ == '__main__':
    project = Path(__file__).resolve().parent/'WheelLeg_LQR'
    sys.path.insert(0,str(project))
    runpy.run_path(str(project/'main.py'),run_name='__main__')
