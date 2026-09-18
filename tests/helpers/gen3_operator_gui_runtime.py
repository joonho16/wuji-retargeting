"""Run with xvfb-run in Docker --network none. No physical keyboard or robot."""
import ctypes as C
from pathlib import Path
import os
import signal
import subprocess
import sys
import tempfile
import time

from example.operator_hold import read_intent, STOP, TELEOP, HOME
from example.arm_teleop.gen3_operator_keyboard import X11Keys, SHUTDOWN_STACK_EXIT


def main():
    if {p.name for p in Path('/sys/class/net').iterdir()} != {'lo'}:
        raise SystemExit('GUI key injection test requires Docker --network none and xvfb-run')
    if os.environ.get('DISPLAY') in (':0', ':1'):
        raise SystemExit('Refusing to inject keys into a desktop display')
    keys = X11Keys()
    lib, display = keys.lib, keys.display
    lib.XDefaultRootWindow.argtypes = [C.c_void_p]
    lib.XDefaultRootWindow.restype = C.c_ulong
    lib.XQueryTree.argtypes = [C.c_void_p, C.c_ulong, C.POINTER(C.c_ulong), C.POINTER(C.c_ulong),
                              C.POINTER(C.POINTER(C.c_ulong)), C.POINTER(C.c_uint)]
    lib.XFetchName.argtypes = [C.c_void_p, C.c_ulong, C.POINTER(C.c_char_p)]
    lib.XFree.argtypes = [C.c_void_p]
    lib.XSetInputFocus.argtypes = [C.c_void_p, C.c_ulong, C.c_int, C.c_ulong]
    lib.XFlush.argtypes = [C.c_void_p]
    xtest = C.CDLL('libXtst.so.6')
    xtest.XTestFakeKeyEvent.argtypes = [C.c_void_p, C.c_uint, C.c_int, C.c_ulong]
    root = lib.XDefaultRootWindow(display)

    def find_window(window):
        name = C.c_char_p()
        lib.XFetchName(display, window, C.byref(name))
        if name.value:
            matched = name.value == b'Gen3 + WH110 control'
            lib.XFree(name)
            if matched: return window
        r, p, children, count = C.c_ulong(), C.c_ulong(), C.POINTER(C.c_ulong)(), C.c_uint()
        lib.XQueryTree(display, window, C.byref(r), C.byref(p), C.byref(children), C.byref(count))
        ids = [children[i] for i in range(count.value)]
        if children: lib.XFree(children)
        for child in ids:
            found = find_window(child)
            if found: return found
        return None

    def key(name, down):
        xtest.XTestFakeKeyEvent(display, keys.codes[name], int(down), 0)
        lib.XFlush(display)

    def focus(window):
        lib.XSetInputFocus(display, window, 1, 0)
        lib.XFlush(display)

    with tempfile.TemporaryDirectory() as d:
        state = Path(d) / 'intent.json'
        with tempfile.TemporaryFile(mode='w+') as output:
            process = subprocess.Popen([sys.executable, '-m', 'example.arm_teleop.gen3_operator_keyboard',
                                        '--state', str(state)], stdout=output, stderr=subprocess.STDOUT)
            def expect(mode, timeout=.3):
                until = time.monotonic()+timeout
                while time.monotonic() < until:
                    if process.poll() is not None: raise AssertionError('Operator window exited')
                    if read_intent(state).mode == mode: return
                    time.sleep(.005)
                raise AssertionError(f'Expected {mode}, got {read_intent(state)}')
            try:
                until = time.monotonic()+5
                window = None
                while not window and time.monotonic() < until:
                    window = find_window(root)
                    time.sleep(.02)
                assert window, 'No operator window appeared'
                focus(window)
                time.sleep(.08)
                key('a', True)
                expect(TELEOP)
                if '--q-only' in sys.argv:
                    key('q', True)  # Q must win while A is still held.
                    assert process.wait(timeout=3) == SHUTDOWN_STACK_EXIT
                    assert read_intent(state).mode == STOP
                    print('GUI: Q while A held publishes STOP and requests whole-stack shutdown')
                    return
                time.sleep(.7)  # continuously held through normal key-repeat delay
                assert read_intent(state).mode == TELEOP
                key('a', False)
                expect(STOP)
                time.sleep(.04)
                key('b', True)
                expect(HOME)
                key('a', True)
                expect(STOP)
                key('a', False)
                time.sleep(.08)
                assert read_intent(state).mode == STOP
                key('b', False)
                time.sleep(.08)
                key('a', True)
                expect(TELEOP)
                focus(root)
                expect(STOP)
                focus(window)
                time.sleep(.08)
                assert read_intent(state).mode == STOP
                key('a', False)
                time.sleep(.08)
                key('b', True)
                expect(HOME)
                process.send_signal(signal.SIGSTOP)
                time.sleep(.16)
                assert not read_intent(state).fresh
                process.send_signal(signal.SIGCONT)
                time.sleep(.08)
                assert read_intent(state).mode == STOP
                key('b', False)
                time.sleep(.08)
                key('b', True)
                expect(HOME)
                process.terminate()
                process.wait(timeout=3)
                assert read_intent(state).mode == STOP
                print('GUI: A/B hold, release, conflict, focus loss, stalled writer and shutdown passed')
            except BaseException:
                output.seek(0)
                print(output.read())
                raise
            finally:
                key('a', False)
                key('b', False)
                key('q', False)
                if process.poll() is None:
                    process.send_signal(signal.SIGCONT)
                    process.terminate()
                    process.wait(timeout=3)
                keys.close()


if __name__ == '__main__': main()
