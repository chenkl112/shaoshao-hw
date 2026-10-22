"""Private local task API for the C++ web service; no TCP/HTTP listener."""
import json
import os
from pathlib import Path
import socket
import socketserver
import stat
import threading

COMMANDS = frozenset(('/api/mode', '/api/spin', '/api/spin/heartbeat',
    '/api/navigation/goal', '/api/navigation/plan', '/api/navigation/cancel',
    '/api/mapping/filter', '/api/localization/retry'))


def dispatch(panel, request):
    if not isinstance(request, dict):
        raise ValueError('Expected a request object')
    operation = request.get('operation')
    if operation == 'snapshot':
        with panel.lock:
            info, image = panel.saved_map()
            return {'state': panel.state(), 'map': info,
                    'image_path': str(image) if image else ''}
    if operation == 'costmap':
        with panel.lock:
            return dict(panel.navigation_costmap)
    if operation != 'command' or request.get('path') not in COMMANDS:
        raise ValueError('Unsupported supervisor operation')
    path, data = request['path'], request.get('body')
    if not isinstance(data, dict):
        raise ValueError('Expected a command object')
    if path == '/api/mode':
        panel.request_mode(data['mode'])
    elif path == '/api/spin':
        panel.set_spin(data['enabled'], data.get('rate_rad_s'))
    elif path == '/api/spin/heartbeat':
        panel.heartbeat_spin()
    elif path == '/api/navigation/goal':
        panel.send_navigation_goal(data['x'], data['y'], data['map_revision'])
    elif path == '/api/navigation/plan':
        panel.plan_navigation_goal(data['x'], data['y'], data['map_revision'])
    elif path == '/api/navigation/cancel':
        panel.cancel_navigation()
    elif path == '/api/mapping/filter':
        panel.set_mapping_filter(data['enabled'], data['height_m'], data.get('dynamic_enabled'))
    elif path == '/api/localization/retry':
        if not panel.bridge or not hasattr(panel.bridge, 'retry_localization'):
            raise RuntimeError('定位服务未运行')
        panel.bridge.retry_localization()
    return panel.state()


class SupervisorRPC:
    def __init__(self, path, handler):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if self.path.exists():
            status = self.path.lstat()
            if not stat.S_ISSOCK(status.st_mode) or status.st_uid != os.getuid():
                raise RuntimeError('Supervisor socket path is occupied')
            with socket.socket(socket.AF_UNIX) as probe:
                probe.settimeout(.2)
                try:
                    probe.connect(str(self.path))
                except (ConnectionRefusedError, FileNotFoundError):
                    self.path.unlink(missing_ok=True)
                else:
                    raise RuntimeError('Supervisor socket is already active')

        class Handler(socketserver.StreamRequestHandler):
            def handle(self):
                self.request.settimeout(.5)
                try:
                    line = self.rfile.readline(16385)
                    if not line:
                        return
                    if not line.endswith(b'\n') or len(line) > 16384:
                        raise ValueError('Invalid RPC request length')
                    result = {'status': 200, 'body': handler(json.loads(line))}
                except (ValueError, KeyError, TypeError) as exc:
                    result = {'status': 400, 'body': {'error': str(exc)}}
                except RuntimeError as exc:
                    result = {'status': 409, 'body': {'error': str(exc)}}
                except Exception:
                    result = {'status': 503, 'body': {'error': 'Supervisor unavailable'}}
                try:
                    self.wfile.write(json.dumps(result, ensure_ascii=False, allow_nan=False).encode()+b'\n')
                except OSError:
                    pass  # A disconnected client must not interrupt the supervisor.

        class Server(socketserver.ThreadingUnixStreamServer):
            daemon_threads = True
        self.server = Server(str(self.path), Handler)
        os.chmod(self.path, 0o600)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(2)
        self.path.unlink(missing_ok=True)
