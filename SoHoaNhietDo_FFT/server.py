"""Web server nho de mo trang index.html qua http://localhost.

Web Serial chi chay trong "secure context" (https hoac localhost), nen mo trang
qua server nay thay vi bam dup file .html.
"""
import http.server
import os
import socketserver
import sys

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
os.chdir(os.path.dirname(os.path.abspath(__file__)))


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    """Tat cache de sua code xong chi can F5 la thay ngay."""

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


if __name__ == "__main__":
    with Server(("127.0.0.1", PORT), NoCacheHandler) as httpd:
        print(f"[OK] Mo trinh duyet Chrome/Edge tai: http://localhost:{PORT}/")
        print("     Nhan Ctrl+C de dung.")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\n[Dung] Da tat server.")
