from http.server import BaseHTTPRequestHandler, HTTPServer

from realsense import RealSense

rs = RealSense(1280, 720)


class RealsenseHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-type", "text/plain")
            self.end_headers()
            self.wfile.write(b"Hello from RealSense")
        else:
            target = "_get" + "_".join(self.path.split("/"))
            if not hasattr(self, target):
                self.send_response(404)
                self.send_header("Content-type", "text/plain")
                self.end_headers()
                self.wfile.write(b"404 Not Found")
            else:
                self.send_response(200)
                self.send_header("Content-type", "application/numpy")
                self.end_headers()
                m = getattr(self, target)
                content = m()
                self.wfile.write(content)

    def _get_depth(self) -> bytes:
        img = rs.get_depth(post_process=False)
        return img.tobytes()

    def _get_color(self) -> bytes:
        img = rs.get_color()
        return img.tobytes()

    def _get_intrinsic_color(self) -> bytes:
        return rs.color_intrinsic.tobytes()

    def _get_intrinsic_depth(self) -> bytes:
        return rs.depth_intrinsic.tobytes()


def run(server_class=HTTPServer, handler_class=RealsenseHandler, port=8000):
    server_address = ("0.0.0.0", port)
    httpd = server_class(server_address, handler_class)
    print(f"Starting httpd server on port {port}")
    httpd.serve_forever()


if __name__ == "__main__":
    run()
