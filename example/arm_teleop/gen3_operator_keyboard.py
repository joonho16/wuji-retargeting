"""Focused X11 operator window: physical A/B hold, with no key-repeat guessing."""
import argparse
import ctypes
import ctypes.util
from pathlib import Path
import signal
import time

from example.operator_hold import HoldKeys, IntentWriter, STOP, TELEOP, HOME, TIMEOUT

SHUTDOWN_STACK_EXIT = 42


class X11Keys:
    def __init__(self):
        self.lib = ctypes.CDLL(ctypes.util.find_library("X11") or "libX11.so.6")
        self.lib.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self.lib.XOpenDisplay.restype = ctypes.c_void_p
        self.lib.XCloseDisplay.argtypes = [ctypes.c_void_p]
        self.lib.XStringToKeysym.argtypes = [ctypes.c_char_p]
        self.lib.XStringToKeysym.restype = ctypes.c_ulong
        self.lib.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self.lib.XKeysymToKeycode.restype = ctypes.c_ubyte
        self.lib.XQueryKeymap.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        self.display = self.lib.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError("Cannot open DISPLAY. Run from the Linux desktop with X11 access.")
        self.codes = {key: self.lib.XKeysymToKeycode(self.display, self.lib.XStringToKeysym(key.encode()))
                      for key in ("a", "b", "q", "Escape")}
        if not all(self.codes.values()):
            self.close()
            raise RuntimeError("Keyboard layout has no A/B/Q/Escape key mapping")

    def read(self):
        bits = (ctypes.c_ubyte * 32)()
        self.lib.XQueryKeymap(self.display, bits)
        return {key: bool(bits[code // 8] & (1 << (code % 8))) for key, code in self.codes.items()}

    def close(self):
        if self.display:
            self.lib.XCloseDisplay(self.display)
            self.display = None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state", type=Path, default=Path("/operator/intent.json"))
    args = parser.parse_args(argv)
    import tkinter as tk

    keys = X11Keys()
    root = None
    writer = None
    exit_code = 0
    try:
        root = tk.Tk()
        root.title("Gen3 + WH110 control")
        root.geometry("520x240")
        root.resizable(False, False)
        tk.Label(root, text="Hold A: arm + hand teleop\nHold B: ready + open hand",
                 font=("sans", 19), pady=20).pack()
        label = tk.Label(root, text="STOPPED", font=("sans", 22, "bold"), fg="#a32a2a")
        label.pack()
        tk.Label(root, text="Release to stop. Keep this window focused.\nQ: shut down everything. Esc / close: exit this window.",
                 font=("sans", 12), pady=14).pack()
        writer = IntentWriter(args.state)
        state = HoldKeys()
        last_tick = time.monotonic()

        def focus_lost(_event=None):
            state.update(a=False, b=False, focused=False)
            writer.publish(STOP)

        def update():
            nonlocal last_tick, exit_code
            now = time.monotonic()
            if now-last_tick > TIMEOUT:
                state.update(a=False, b=False, focused=False)
            last_tick = now
            pressed = keys.read()
            focused = root.focus_displayof() is not None
            if focused and (pressed["q"] or pressed["Escape"]):
                # Stop wins even if A or B is still held. The host launcher only
                # tears down the stack after this process has closed its lease.
                writer.publish(STOP)
                exit_code = SHUTDOWN_STACK_EXIT if pressed["q"] else 0
                root.quit()
                return
            mode = state.update(a=pressed["a"], b=pressed["b"], focused=focused)
            writer.publish(mode)
            label.configure(text={STOP: "STOPPED", TELEOP: "A: TELEOP", HOME: "B: READY + OPEN"}[mode],
                            fg="#a32a2a" if mode == STOP else "#167041")
            root.after(20, update)

        root.bind("<FocusOut>", focus_lost)
        root.protocol("WM_DELETE_WINDOW", root.quit)
        signal.signal(signal.SIGINT, lambda *_: root.quit())
        signal.signal(signal.SIGTERM, lambda *_: root.quit())
        root.after(0, update)
        print("Hold A for teleop; hold B for ready + open. Release to stop; Q shuts down the stack.", flush=True)
        root.mainloop()
    finally:
        if writer is not None:
            writer.close()
        if root is not None:
            root.destroy()
        keys.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
