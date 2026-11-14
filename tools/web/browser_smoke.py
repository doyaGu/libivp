#!/usr/bin/env python3
"""Open the WebAssembly example pages in headless Chrome and check they draw.

    browser_smoke.py --build-dir <build> [--chrome PATH] [--frames N]
                     [--timeout S] [--screenshots DIR] [--only NAME ...]

The pages under <build>/examples are served over HTTP on a local port. Each
example_<name>.html must show --frames frames (Module.ivpFrames, counted by
the OpenGL ES backend) within --timeout seconds. Then a screenshot of the
canvas is taken. A page fails when it

  * does not show the frames in time,
  * throws, aborts, logs a console error, or causes a WebGL error,
  * leaves the canvas empty: fewer than --min-drawn of its pixels differ
    from their row's background colour (the sky is a vertical gradient),
  * has a drawing buffer other than its CSS size times devicePixelRatio
    (--device-scale-factor 2 emulates a Retina screen).

The index page must load cleanly and link exactly the example pages, and the
first example must keep running while its canvas has no height (a page
shorter than its header) and draw again when the height comes back.

The pixel check catches a canvas left blank. It cannot tell a scene with
some of its bodies missing from one that has few of them: scenes differ too
much for one threshold. The screenshots are kept for a person to look at.

smoke_test_examples.py runs the examples headless under node, which skips
the renderer. This runs the pages as a browser does, so the Asyncify yields
of the main loop, the shaders and the page shell are exercised. WebGL2 comes
from Chrome's software rasterizer, SwiftShader, which is what CI machines
have; local runs ask for it too, so they draw the same way. Chrome is driven
over the DevTools protocol on a pipe (--remote-debugging-pipe), so only the
standard library is needed.
"""

import argparse
import base64
import collections
import functools
import glob
import http.server
import json
import os
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import threading
import time
import zlib

CHROME_NAMES = ("google-chrome", "google-chrome-stable", "chromium",
                "chromium-browser", "chrome")
CHROME_PATHS = ("/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
                "/Applications/Chromium.app/Contents/MacOS/Chromium")

VIEWPORT = (1280, 720)
SHOT_SCALE = 0.5

# "WebGL: INVALID_VALUE: ..." or "GL_INVALID_VALUE : glDrawArrays: ...", but
# not the performance notes ("GPU stall due to ReadPixels") on the same source
WEBGL_ERROR = re.compile(
    r"\b(?:WebGL: |GL_)(?:INVALID_\w+|OUT_OF_MEMORY|CONTEXT_LOST\w*)")


def find_chrome():
    for name in CHROME_NAMES:
        path = shutil.which(name)
        if path:
            return path
    for path in CHROME_PATHS:
        if os.access(path, os.X_OK):
            return path
    return None


# ── HTTP ───────────────────────────────────────────────────────────────────

class QuietHandler(http.server.SimpleHTTPRequestHandler):
    extensions_map = {
        **http.server.SimpleHTTPRequestHandler.extensions_map,
        ".js": "text/javascript",
        ".wasm": "application/wasm",  # for WebAssembly.compileStreaming
    }

    def do_GET(self):
        if self.path == "/favicon.ico":  # the pages have none
            self.send_response(204)
            self.end_headers()
            return
        super().do_GET()

    def log_message(self, *args):
        pass


class QuietServer(http.server.ThreadingHTTPServer):
    daemon_threads = True

    def handle_error(self, request, client_address):
        pass  # a tab closed while loading


def serve(directory):
    handler = functools.partial(QuietHandler, directory=directory)
    server = QuietServer(("127.0.0.1", 0), handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


# ── Chrome over the DevTools protocol ──────────────────────────────────────

class ChromeError(Exception):
    pass


# Chrome reads the protocol from fd 3 and writes it to fd 4. Python's
# subprocess keeps fds only at their own numbers, so a small exec wrapper
# moves the pipe ends there (first above 9, so that neither is overwritten).
EXEC_WITH_PIPE = (
    "import fcntl, os, sys\n"
    "r = fcntl.fcntl(int(sys.argv[1]), fcntl.F_DUPFD, 10)\n"
    "w = fcntl.fcntl(int(sys.argv[2]), fcntl.F_DUPFD, 10)\n"
    "os.dup2(r, 3)\n"
    "os.dup2(w, 4)\n"
    "os.execv(sys.argv[3], sys.argv[3:])\n")


class Chrome:
    def __init__(self, path, profile_dir, log):
        to_chrome_r, self._to_chrome = os.pipe()
        self._from_chrome, from_chrome_w = os.pipe()
        flags = [
            "--headless",
            "--remote-debugging-pipe",
            "--user-data-dir=" + profile_dir,
            "--no-first-run",
            "--no-default-browser-check",
            "--disable-extensions",
            "--disable-sync",
            "--mute-audio",
            "--hide-scrollbars",
            # Software WebGL everywhere, as on GPU-less CI machines
            "--use-angle=swiftshader",
            "--enable-unsafe-swiftshader",
            # Keep animation frames coming for pages that are not focused
            "--disable-background-timer-throttling",
            "--disable-renderer-backgrounding",
            "--disable-backgrounding-occluded-windows",
        ]
        if sys.platform.startswith("linux"):
            # CI kernels may refuse the namespace sandbox; the pages are
            # this build's, served from localhost
            flags.append("--no-sandbox")
        self.proc = subprocess.Popen(
            [sys.executable, "-I", "-c", EXEC_WITH_PIPE,
             str(to_chrome_r), str(from_chrome_w), path] + flags + ["about:blank"],
            pass_fds=(to_chrome_r, from_chrome_w),
            stdin=subprocess.DEVNULL, stdout=log, stderr=log)
        os.close(to_chrome_r)
        os.close(from_chrome_w)
        self._write_lock = threading.Lock()
        self._cond = threading.Condition()
        self._next_id = 0
        self._replies = {}
        self._events = {}
        self._closed = False
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        buf = bytearray()
        while True:
            try:
                chunk = os.read(self._from_chrome, 1 << 16)
            except OSError:
                chunk = b""
            if not chunk:
                break
            buf += chunk
            while True:
                end = buf.find(b"\0")
                if end < 0:
                    break
                message = json.loads(bytes(buf[:end]))
                del buf[:end + 1]
                with self._cond:
                    if "id" in message:
                        self._replies[message["id"]] = message
                    else:
                        self._events.setdefault(
                            message.get("sessionId"), []).append(message)
                    self._cond.notify_all()
        with self._cond:
            self._closed = True
            self._cond.notify_all()

    def call(self, method, params=None, session=None, timeout=30.0):
        with self._cond:
            self._next_id += 1
            ident = self._next_id
        message = {"id": ident, "method": method, "params": params or {}}
        if session:
            message["sessionId"] = session
        data = json.dumps(message).encode() + b"\0"
        with self._write_lock:
            try:
                while data:
                    data = data[os.write(self._to_chrome, data):]
            except OSError as exc:
                raise ChromeError("%s: %s" % (method, exc))
        deadline = time.monotonic() + timeout
        with self._cond:
            while ident not in self._replies:
                if self._closed:
                    raise ChromeError("%s: Chrome closed the connection" % method)
                left = deadline - time.monotonic()
                if left <= 0:
                    raise ChromeError("%s: no reply in %g s" % (method, timeout))
                self._cond.wait(left)
            reply = self._replies.pop(ident)
        if "error" in reply:
            raise ChromeError("%s: %s" % (method, reply["error"].get("message")))
        return reply.get("result", {})

    def take_events(self, session):
        with self._cond:
            return self._events.pop(session, [])

    def evaluate(self, session, expression):
        result = self.call("Runtime.evaluate",
                           {"expression": expression, "returnByValue": True},
                           session=session)
        if "exceptionDetails" in result:
            raise ChromeError("evaluate %r: %s" % (
                expression, result["exceptionDetails"].get("text")))
        return result["result"].get("value")

    def close(self):
        try:
            self.call("Browser.close", timeout=5)
        except ChromeError:
            pass
        try:
            self.proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()
        for fd in (self._to_chrome, self._from_chrome):
            try:
                os.close(fd)
            except OSError:
                pass


class Page:
    """A fresh tab with the console, the log and exceptions reported."""

    def __init__(self, chrome, device_scale=1.0):
        self.chrome = chrome
        self.device_scale = device_scale
        self.target = chrome.call("Target.createTarget",
                                  {"url": "about:blank"})["targetId"]
        self.session = chrome.call("Target.attachToTarget",
                                   {"targetId": self.target,
                                    "flatten": True})["sessionId"]
        self.console = []
        self.problems = []
        for method in ("Runtime.enable", "Log.enable", "Page.enable"):
            chrome.call(method, session=self.session)
        chrome.call("Emulation.setDeviceMetricsOverride",
                    {"width": VIEWPORT[0], "height": VIEWPORT[1],
                     "deviceScaleFactor": device_scale, "mobile": False},
                    session=self.session)

    def navigate(self, url):
        self.chrome.call("Page.navigate", {"url": url}, session=self.session)

    def evaluate(self, expression):
        return self.chrome.evaluate(self.session, expression)

    def poll(self):
        """Collect new events; return True when the page has a problem."""
        for event in self.chrome.take_events(self.session):
            method = event.get("method")
            params = event.get("params", {})
            if method == "Runtime.consoleAPICalled":
                text = " ".join(str(a.get("value", a.get("description", "")))
                                for a in params.get("args", []))
                self.console.append(text)
                if params.get("type") in ("error", "assert"):
                    self.problems.append("console error: " + first_line(text))
            elif method == "Runtime.exceptionThrown":
                details = params.get("exceptionDetails", {})
                text = (details.get("exception", {}).get("description")
                        or details.get("text", ""))
                self.problems.append("exception: " + first_line(text))
            elif method == "Log.entryAdded":
                entry = params.get("entry", {})
                text = entry.get("text", "")
                if entry.get("level") == "error":
                    self.problems.append("log error: " + first_line(text) + (
                        " (%s)" % entry["url"] if entry.get("url") else ""))
                elif (entry.get("source") == "rendering"
                      and WEBGL_ERROR.search(text)):
                    # Chrome reports GL errors as warnings
                    self.problems.append("WebGL error: " + first_line(text))
            elif method == "Inspector.targetCrashed":
                self.problems.append("the page crashed")
        return bool(self.problems)

    def screenshot(self, selector):
        rect = self.evaluate(
            "(function () { var r = document.querySelector(%s)"
            ".getBoundingClientRect(); return [r.left, r.top, r.width, r.height]; })()"
            % json.dumps(selector))
        shot = self.chrome.call(
            "Page.captureScreenshot",
            {"format": "png",
             "clip": {"x": rect[0], "y": rect[1], "width": rect[2],
                      "height": rect[3],
                      # the same image size at any devicePixelRatio
                      "scale": SHOT_SCALE / self.device_scale}},
            session=self.session, timeout=60)
        return base64.b64decode(shot["data"])

    def close(self):
        try:
            self.chrome.call("Target.closeTarget", {"targetId": self.target})
        except ChromeError:
            pass
        self.chrome.take_events(self.session)


def describe(problems):
    """The first three distinct problems."""
    return "; ".join(list(dict.fromkeys(problems))[:3])


def first_line(text):
    lines = str(text).strip().splitlines()
    return lines[0][:300] if lines else ""


# ── Screenshot analysis ────────────────────────────────────────────────────

def decode_png(data):
    """Width, height and RGB pixel rows of an 8-bit RGB(A) PNG."""
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("not a PNG")
    pos, idat, header = 8, [], None
    while pos < len(data):
        length, kind = struct.unpack(">I4s", data[pos:pos + 8])
        body = data[pos + 8:pos + 8 + length]
        pos += 12 + length
        if kind == b"IHDR":
            header = struct.unpack(">IIBBBBB", body)
        elif kind == b"IDAT":
            idat.append(body)
        elif kind == b"IEND":
            break
    width, height, depth, colour, _, _, interlace = header
    if depth != 8 or colour not in (2, 6) or interlace:
        raise ValueError("unsupported PNG (depth %d, colour %d, interlace %d)"
                         % (depth, colour, interlace))
    bpp = 3 if colour == 2 else 4
    stride = width * bpp
    raw = zlib.decompress(b"".join(idat))
    rows, prev, pos = [], bytearray(stride), 0
    for _ in range(height):
        kind = raw[pos]
        line = bytearray(raw[pos + 1:pos + 1 + stride])
        pos += 1 + stride
        if kind == 1:
            for i in range(bpp, stride):
                line[i] = (line[i] + line[i - bpp]) & 0xFF
        elif kind == 2:
            for i in range(stride):
                line[i] = (line[i] + prev[i]) & 0xFF
        elif kind == 3:
            for i in range(stride):
                left = line[i - bpp] if i >= bpp else 0
                line[i] = (line[i] + ((left + prev[i]) >> 1)) & 0xFF
        elif kind == 4:
            for i in range(stride):
                a = line[i - bpp] if i >= bpp else 0
                b = prev[i]
                c = prev[i - bpp] if i >= bpp else 0
                p = a + b - c
                pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
                pred = a if pa <= pb and pa <= pc else (b if pb <= pc else c)
                line[i] = (line[i] + pred) & 0xFF
        elif kind != 0:
            raise ValueError("bad PNG filter %d" % kind)
        rows.append(bytes(line))
        prev = line
    return width, height, [
        list(zip(r[0::bpp], r[1::bpp], r[2::bpp])) for r in rows]


def drawn_share(png, tolerance=12):
    """Share of pixels that differ from the most common colour of their row.

    The background is a vertical gradient, so each row has its own
    background colour; what differs from it is scene, lines or HUD text.
    """
    width, height, rows = decode_png(png)
    drawn = 0
    for row in rows:
        r0, g0, b0 = collections.Counter(row).most_common(1)[0][0]
        drawn += sum(1 for r, g, b in row
                     if abs(r - r0) > tolerance or abs(g - g0) > tolerance
                     or abs(b - b0) > tolerance)
    return drawn / float(width * height)


# ── Checks ─────────────────────────────────────────────────────────────────

FRAMES_JS = "(window.Module && Module.ivpFrames) | 0"
CANVAS_JS = ("(function () { var c = document.getElementById('canvas');"
             " return [c.width, c.height, c.clientWidth, c.clientHeight,"
             " window.devicePixelRatio]; })()")
STATUS_JS = ("(function () { var s = document.getElementById('status');"
             " return s && s.className === 'error' ? s.textContent : ''; })()")


def check_example(chrome, url, args, shot_path):
    """(ok, summary, console lines) for one example page."""
    page = Page(chrome, args.device_scale_factor)
    try:
        ok, summary = check_page(page, url, args, shot_path)
    except ChromeError as exc:  # a hung page stops answering
        ok, summary = False, str(exc)
    finally:
        page.close()
    return ok, summary, page.console


def check_page(page, url, args, shot_path):
    start = time.monotonic()
    page.navigate(url)
    while True:
        if page.poll():
            return False, describe(page.problems)
        frames = page.evaluate(FRAMES_JS) or 0
        if frames >= args.frames:
            break
        status = page.evaluate(STATUS_JS)
        if status:
            return False, "page error: " + first_line(status)
        if time.monotonic() - start > args.timeout:
            return False, "showed %d of %d frames in %g s" % (
                frames, args.frames, args.timeout)
        time.sleep(0.1)
    elapsed = time.monotonic() - start
    canvas = page.evaluate(CANVAS_JS)
    if (abs(canvas[0] - round(canvas[2] * canvas[4])) > 1
            or abs(canvas[1] - round(canvas[3] * canvas[4])) > 1):
        return False, ("canvas buffer %dx%d for %gx%g CSS pixels at"
                       " devicePixelRatio %g: not at device resolution"
                       % tuple(canvas))
    png = page.screenshot("#canvas")
    if page.poll():
        return False, describe(page.problems)
    if shot_path:
        with open(shot_path, "wb") as f:
            f.write(png)
    drawn = drawn_share(png)
    summary = "%d frames in %.1f s, %.0f%% drawn" % (frames, elapsed, 100.0 * drawn)
    if drawn < args.min_drawn:
        return False, "too little drawn: " + summary
    return True, summary


def check_zero_height(chrome, url, args):
    """Problems when a page's canvas loses its height and gets it back.

    A page shorter than its header leaves the canvas no height. The example
    then has nothing to draw into, and its loop must still give the browser
    its turns, or the tab hangs and never sees the size come back.
    """
    page = Page(chrome, args.device_scale_factor)

    def resize(height):
        chrome.call("Emulation.setDeviceMetricsOverride",
                    {"width": VIEWPORT[0], "height": height,
                     "deviceScaleFactor": args.device_scale_factor,
                     "mobile": False}, session=page.session)

    def frames_after(least):
        start = time.monotonic()
        while True:
            frames = page.evaluate(FRAMES_JS) or 0
            if frames >= least or time.monotonic() - start > args.timeout:
                return frames
            time.sleep(0.1)

    try:
        page.navigate(url)
        before = frames_after(10)
        if before < 10:
            return ["showed %d frames before the resize" % before]
        resize(30)
        time.sleep(0.5)
        try:
            page.evaluate(CANVAS_JS)
        except ChromeError:
            return ["the page stops answering while its canvas has no height"]
        resize(VIEWPORT[1])
        resumed = page.evaluate(FRAMES_JS) or 0
        after = frames_after(resumed + 10)
        page.poll()
        problems = list(dict.fromkeys(page.problems))
        if after < resumed + 10:
            problems.append("no frames after the canvas got its height back")
        return problems
    except ChromeError as exc:
        return [str(exc)]
    finally:
        page.close()


def check_index(chrome, url, pages, timeout):
    """Problems with the index page: errors, missing or extra links."""
    page = Page(chrome)
    try:
        page.navigate(url)
        start = time.monotonic()
        while page.evaluate("document.readyState") != "complete":
            if time.monotonic() - start > timeout:
                return ["index.html did not load in %g s" % timeout]
            time.sleep(0.1)
        links = page.evaluate(
            "Array.from(document.querySelectorAll('a[href]'),"
            " function (a) { return a.getAttribute('href'); })")
        page.poll()
        problems = list(page.problems)
        linked = {h for h in links if h.startswith("example_")}
        problems += ["index.html does not link %s" % p
                     for p in sorted(pages - linked)]
        problems += ["index.html links %s, which was not built" % p
                     for p in sorted(linked - pages)]
        return problems
    finally:
        page.close()


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--build-dir", required=True)
    ap.add_argument("--chrome", default=None,
                    help="Chrome or Chromium executable (default: search)")
    ap.add_argument("--device-scale-factor", type=float, default=1.0,
                    help="devicePixelRatio of the emulated screen (default 1;"
                         " 2 is a Retina screen)")
    ap.add_argument("--frames", type=int, default=30,
                    help="frames each page must show (default 30)")
    ap.add_argument("--timeout", type=float, default=60.0,
                    help="seconds allowed per page (default 60)")
    ap.add_argument("--min-drawn", type=float, default=0.05,
                    help="least share of canvas pixels off their row's"
                         " background colour (default 0.05)")
    ap.add_argument("--screenshots", default=None,
                    help="directory for a PNG of every example's canvas")
    ap.add_argument("--only", nargs="*", default=None,
                    help="example names to check (default: all)")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="print each page's console output")
    args = ap.parse_args()

    examples_dir = os.path.join(os.path.abspath(args.build_dir), "examples")
    pages = {os.path.basename(p)
             for p in glob.glob(os.path.join(examples_dir, "example_*.html"))}
    if not pages:
        print("no example_*.html pages under %s" % examples_dir)
        return 1
    names = sorted(p[len("example_"):-len(".html")] for p in pages)
    if args.only:
        unknown = sorted(set(args.only) - set(names))
        if unknown:
            print("no page for: %s" % ", ".join(unknown))
            return 1
        names = [n for n in names if n in args.only]
    chrome_path = args.chrome or find_chrome()
    if not chrome_path:
        print("no Chrome or Chromium found; pass --chrome")
        return 1
    if args.screenshots:
        os.makedirs(args.screenshots, exist_ok=True)

    server = serve(examples_dir)
    base = "http://127.0.0.1:%d/" % server.server_address[1]
    with tempfile.TemporaryDirectory(prefix="ivp-browser-smoke-") as tmp:
        log_path = os.path.join(tmp, "chrome.log")
        with open(log_path, "wb") as log:
            chrome = Chrome(chrome_path, os.path.join(tmp, "profile"), log)
            try:
                failed = run_checks(chrome, base, names, pages, args)
            except ChromeError as exc:
                print("Chrome failed: %s" % exc)
                with open(log_path, "rb") as f:
                    tail = f.read().decode(errors="replace").strip().splitlines()[-20:]
                print("\n".join("     | " + l for l in tail))
                failed = ["chrome"]
            finally:
                chrome.close()
                server.shutdown()
    return 1 if failed else 0


def run_checks(chrome, base, names, pages, args):
    version = chrome.call("Browser.getVersion")
    print("%s, WebGL through SwiftShader, devicePixelRatio %g, %d frames per page"
          % (version.get("product"), args.device_scale_factor, args.frames))
    failed = []
    problems = check_index(chrome, base + "index.html", pages, args.timeout)
    print("%-4s index.html%s" % ("FAIL" if problems else "ok",
                                 "".join("\n     | " + p for p in problems)))
    if problems:
        failed.append("index.html")
    if names:
        problems = check_zero_height(
            chrome, base + "example_%s.html" % names[0], args)
        print("%-4s zero-height canvas (%s)%s" % (
            "FAIL" if problems else "ok", names[0],
            "".join("\n     | " + p for p in problems)))
        if problems:
            failed.append("zero-height canvas")
    for name in names:
        shot = (os.path.join(args.screenshots, "example_%s.png" % name)
                if args.screenshots else None)
        ok, summary, console = check_example(
            chrome, base + "example_%s.html" % name, args, shot)
        print("%-4s %-22s %s" % ("ok" if ok else "FAIL", name, summary))
        if not ok or args.verbose:
            tail = [l for c in console for l in str(c).splitlines()][-15:]
            if tail:
                print("\n".join("     | " + l for l in tail))
        if not ok:
            failed.append(name)
    print("%d/%d example pages passed" % (len(names) - len([
        f for f in failed if f in names]), len(names)))
    return failed


if __name__ == "__main__":
    sys.exit(main())
