"""Local server for the web page, made for working on the program.

Usage:  python scripts/serve.py [port]        then open http://localhost:8010

- It rebuilds the packed Python (web/vdcalc.zip) and the examples every time the page asks for them, so after you
  change the Python code or the examples you only reload the page. No need to stop the server or run build_web.py.
- It tells the browser not to keep copies of anything, so a reload always shows the current files.
Stop it with Ctrl+C. Leave it running in its own terminal and use another terminal for other commands.
"""
import sys
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

from build_web import WEB, build


class Handler(SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path.split("?")[0] == "/vdcalc.zip" or self.path.startswith("/examples/"):
            build()
        super().do_GET()

    def end_headers(self):
        self.send_header("Cache-Control", "no-store, max-age=0")
        super().end_headers()

    def log_message(self, fmt, *args):          # keep the terminal quiet except for errors
        if args and str(args[1]).startswith(("4", "5")):
            super().log_message(fmt, *args)


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8010
    build()
    server = ThreadingHTTPServer(("127.0.0.1", port), partial(Handler, directory=str(WEB)))
    print(f"Serving {WEB}\nOpen http://localhost:{port}   (Ctrl+C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
