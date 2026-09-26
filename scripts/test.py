#!/usr/bin/env python3
"""Build/test shared logic and the unchanged mms text protocol (Python + C++17)."""
import os
from pathlib import Path
import queue
import shutil
import subprocess
import tempfile
import threading

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / 'build/host'
BUILD.mkdir(parents=True, exist_ok=True)
compiler = os.environ.get('CXX') or shutil.which('clang++') or shutil.which('g++')
if not compiler:
    raise SystemExit('Need clang++/g++ (or use CMake with MSVC)')
shared = ['src/MazeMap.cpp', 'src/FloodFill.cpp', 'src/MouseAgent.cpp']
flags = ['-std=c++17', '-Wall', '-Wextra', '-Werror', '-g', '-Isrc']
if os.environ.get('SANITIZE') == '1':
    flags += ['-fsanitize=address,undefined', '-fno-omit-frame-pointer']
for name, sources in [('solver_tests', ['tests/solver_tests.cpp']), ('micromouse', ['Main.cpp', 'API.cpp'])]:
    subprocess.run([compiler, *flags, *sources, *shared, '-o', str(BUILD/name)], cwd=ROOT, check=True)
subprocess.run([str(BUILD/'solver_tests')], check=True, timeout=20)


def read_maze(path):
    lines = path.read_text().splitlines()
    walls = {}
    if path.suffix == '.num':
        for line in lines:
            x, y, *edges = map(int, line.split())
            assert len(edges) == 4
            walls[x, y] = edges  # N/E/S/W
    else:
        assert len(lines) == 33
        for x in range(16):
            for y in range(16):
                row = 2 * (15-y)
                walls[x, y] = [lines[row][4*x+2] != ' ', lines[row+1][4*x+4] != ' ',
                               lines[row+2][4*x+2] != ' ', lines[row+1][4*x] != ' ']
    assert len(walls) == 256
    return walls


def check_protocol(path):
    walls = read_maze(path)
    with tempfile.TemporaryFile(mode='w+t') as logs:
        proc = subprocess.Popen([str(BUILD/'micromouse')], stdin=subprocess.PIPE,
                                stdout=subprocess.PIPE, stderr=logs, text=True, bufsize=1)
        lines = queue.Queue()
        def reader():
            for line in proc.stdout:
                lines.put(line.strip())
            lines.put(None)
        thread = threading.Thread(target=reader, daemon=True)
        thread.start()
        x = y = facing = moves = turns = arrivals = 0
        try:
            for _ in range(100000):
                command = lines.get(timeout=5)
                if command is None:
                    break
                if command.startswith('wall'):
                    relative = {'wallFront': 0, 'wallLeft': -1, 'wallRight': 1}[command]
                    reply = 'true' if walls[x, y][(facing+relative) % 4] else 'false'
                elif command == 'wasReset':
                    reply = 'false'
                elif command in ('turnLeft', 'turnRight'):
                    facing = (facing + (1 if command == 'turnRight' else -1)) % 4
                    turns += 1
                    reply = 'ack'
                elif command == 'moveForward':
                    assert not walls[x, y][facing], (path.name, 'collision', x, y, facing)
                    dx, dy = [(0, 1), (1, 0), (0, -1), (-1, 0)][facing]
                    x += dx
                    y += dy
                    assert 0 <= x < 16 and 0 <= y < 16
                    moves += 1
                    if (arrivals == 1 and x == y == 0) or (arrivals != 1 and x in (7, 8) and y in (7, 8)):
                        arrivals += 1
                    reply = 'ack'
                else:
                    raise AssertionError(('Unexpected stdout/protocol', command))
                proc.stdin.write(reply+'\n')
                proc.stdin.flush()
            else:
                raise AssertionError('Algorithm never terminated')
            assert proc.wait(timeout=5) == 0
            assert arrivals == 3 and x in (7, 8) and y in (7, 8)
            logs.seek(0)
            assert logs.read().count('phase complete') == 3
            print(f'{path.name}: all three phases, {moves} cells, {turns} quarter turns, no collision PASS')
        finally:
            if proc.poll() is None:
                proc.kill()
            proc.wait()
            proc.stdin.close()
            proc.stdout.close()
            thread.join(timeout=1)


for maze in sorted((ROOT/'mazes').iterdir()):
    if maze.suffix in ('.txt', '.num'):
        check_protocol(maze)
