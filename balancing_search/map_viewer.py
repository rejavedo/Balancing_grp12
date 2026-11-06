# Map viewer: browse the live SLAM/AMCL map and saved maps from any browser on the LAN.
"""HTTP map viewer. Open http://<robot>:8080/ (default port).

/            live /map with robot pose (TF map->base_footprint) and target, auto-refresh
/map.png     live map image
/saved       list of saved maps (*.yaml in map_dirs)
/saved/<name>.png   one saved map rendered
"""
import glob
import math
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener

PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>Map viewer</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>body{{font-family:sans-serif;margin:16px;background:#222;color:#eee}}
a{{color:#8cf}}img{{max-width:100%;image-rendering:pixelated;border:1px solid #555}}</style></head>
<body><b>{title}</b> &nbsp; <a href="/">live</a> | <a href="/saved">saved maps</a><br><p>{info}</p>{body}</body></html>"""


def render(grid, scale):
    """grid: int8 HxW (row 0 = bottom). Returns BGR image, flipped so +y is up."""
    img = np.full(grid.shape, 205, np.uint8)
    img[grid == 0] = 254
    occ = grid > 0
    img[occ] = (255 - np.clip(grid[occ], 0, 100) * 255 // 100).astype(np.uint8)
    img = cv2.cvtColor(np.flipud(img), cv2.COLOR_GRAY2BGR)
    if scale != 1:
        img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    return img


class MapViewer(Node):
    def __init__(self):
        super().__init__('map_viewer')
        self.port = self.declare_parameter('port', 8080).value
        self.scale = self.declare_parameter('scale', 3).value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.map_dirs = self.declare_parameter(
            'map_dirs', ['/home/ros2_ws/maps', '/home/ros2_ws/src/balance_mapping/maps']).value
        self.map = None
        self.target = None
        self.lock = threading.Lock()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, qos)
        self.create_subscription(PointStamped, '/target/map_position', self.on_target, qos)
        self.tf = Buffer()
        TransformListener(self.tf, self)

    def on_map(self, msg):
        with self.lock:
            self.map = msg

    def on_target(self, msg):
        self.target = (msg.point.x, msg.point.y)

    def pose(self):
        try:
            t = self.tf.lookup_transform('map', self.base_frame, rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def live_png(self):
        with self.lock:
            m = self.map
        if m is None:
            return None, 'no /map received yet (is SLAM or map_server running?)'
        i = m.info
        grid = np.array(m.data, np.int8).reshape(i.height, i.width)
        img = render(grid, self.scale)
        ox, oy, res, s, h = i.origin.position.x, i.origin.position.y, i.resolution, self.scale, i.height

        def px(x, y):
            return int((x - ox) / res * s), int((h - (y - oy) / res) * s)
        cv2.circle(img, px(0, 0), 4, (0, 160, 0), -1)  # map origin
        info = f'{i.width}x{i.height} cells, {res:.3f} m/cell, origin ({ox:.2f}, {oy:.2f})'
        p = self.pose()
        if p:
            x, y, yaw = p
            cv2.circle(img, px(x, y), 6, (255, 0, 0), -1)
            cv2.arrowedLine(img, px(x, y), px(x + 0.4 * math.cos(yaw), y + 0.4 * math.sin(yaw)),
                            (255, 0, 0), 2, tipLength=0.4)
            info += f' | robot ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)'
        else:
            info += f' | robot pose: no TF map->{self.base_frame}'
        if self.target:
            cv2.drawMarker(img, px(*self.target), (0, 0, 255), cv2.MARKER_CROSS, 16, 3)
            info += f' | target ({self.target[0]:.2f}, {self.target[1]:.2f})'
        return cv2.imencode('.png', img)[1].tobytes(), info

    def saved(self):
        out = {}
        for d in self.map_dirs:
            for y in sorted(glob.glob(os.path.join(d, '*.yaml'))):
                out[os.path.splitext(os.path.basename(y))[0]] = y
        return out

    def saved_png(self, path):
        meta = yaml.safe_load(open(path))
        img_path = os.path.join(os.path.dirname(path), meta['image'])
        raw = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        img = cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
        img = cv2.resize(img, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_NEAREST)
        info = f'{raw.shape[1]}x{raw.shape[0]} px, {meta["resolution"]} m/px, origin {meta["origin"]}'
        return cv2.imencode('.png', img)[1].tobytes(), info


def make_handler(node):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def send(self, body, ctype='text/html; charset=utf-8', code=200):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body if isinstance(body, bytes) else body.encode())

        def do_GET(self):
            p = unquote(self.path.split('?')[0])
            try:
                if p == '/':
                    _, info = node.live_png()
                    body = ('<img id=m src="/map.png"><script>setInterval(()=>{'
                            'm.src="/map.png?"+Date.now();fetch("/info").then(r=>r.text())'
                            '.then(t=>document.querySelector("p").textContent=t)},1000)</script>')
                    self.send(PAGE.format(title='Live /map', info=info, body=body))
                elif p == '/info':
                    self.send(node.live_png()[1], 'text/plain; charset=utf-8')
                elif p == '/map.png':
                    png, info = node.live_png()
                    self.send(png, 'image/png') if png else self.send(info, 'text/plain', 503)
                elif p == '/saved':
                    links = ''.join(f'<li><a href="/saved/{n}">{n}</a> <small>{y}</small></li>'
                                    for n, y in node.saved().items())
                    self.send(PAGE.format(title='Saved maps', info='', body=f'<ul>{links}</ul>'))
                elif p.startswith('/saved/') and p.endswith('.png'):
                    self.send(node.saved_png(node.saved()[p[7:-4]])[0], 'image/png')
                elif p.startswith('/saved/'):
                    n = p[7:]
                    _, info = node.saved_png(node.saved()[n])
                    self.send(PAGE.format(title=n, info=info, body=f'<img src="/saved/{n}.png">'))
                else:
                    self.send('not found', 'text/plain', 404)
            except KeyError:
                self.send('no such map', 'text/plain', 404)
            except Exception as e:
                self.send(f'error: {e}', 'text/plain', 500)
    return H


def main():
    rclpy.init()
    node = MapViewer()
    srv = ThreadingHTTPServer(('0.0.0.0', node.port), make_handler(node))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    node.get_logger().info(f'map viewer on http://0.0.0.0:{node.port}/')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        srv.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
