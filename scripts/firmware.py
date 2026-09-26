#!/usr/bin/env python3
"""Stage shared C++ once, build with Arduino CLI, optionally upload (motors idle)."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--upload', metavar='PORT')
parser.add_argument('--cli', default=os.environ.get('ARDUINO_CLI'))
parser.add_argument('--config', help='Optional arduino-cli.yaml path')
args = parser.parse_args()
cli = args.cli or shutil.which('arduino-cli')
if not cli:
    bundled = Path('/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli')
    if bundled.exists():
        cli = str(bundled)
if not cli:
    parser.error('Install Arduino CLI or pass --cli PATH')
base = [cli]
config = args.config or (str(Path.home()/'.arduinoIDE/arduino-cli.yaml') if (Path.home()/'.arduinoIDE/arduino-cli.yaml').exists() else None)
if config:
    base += ['--config-file', config]
sketch = ROOT/'build/arduino/ucd_mouse'
# This directory is generated only; avoid compiling deleted source leftovers.
if sketch.exists():
    shutil.rmtree(sketch)
sketch.mkdir(parents=True, exist_ok=True)
# Copy exact shared sources: no duplicate maintained algorithm implementation.
shutil.copytree(ROOT/'src', sketch/'src', dirs_exist_ok=True)
for f in (ROOT/'firmware').iterdir():
    if f.suffix in ('.ino', '.h', '.cpp'):
        shutil.copy2(f, sketch/f.name)
build = ROOT/'build/esp32'
fqbn = 'esp32:esp32:esp32c6:CDCOnBoot=cdc'
subprocess.run(base+['compile', '--fqbn', fqbn, '--build-path', str(build), str(sketch)], check=True)
if args.upload:
    subprocess.run(base+['upload', '--fqbn', fqbn, '--port', args.upload, '--input-dir', str(build), str(sketch)], check=True)
print('Sketch:', sketch)
print('Boot is idle. Use help/status at 115200 baud; calibrate before floor movement.')
