"""JSON integration endpoint; retain legacy text /stat and MJPEG behavior."""
import json


def make_handler(original, stats):
    class AebStatusHandler(original):
        def do_GET(self):
            if self.path.split('?',1)[0] != '/aeb_status':
                return super().do_GET()
            body = json.dumps(dict(stats),ensure_ascii=False).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type','application/json; charset=utf-8')
            self.send_header('Cache-Control','no-store')
            self.send_header('Content-Length',str(len(body)))
            self.end_headers()
            self.wfile.write(body)
    return AebStatusHandler
