"""HTTP API so external software can drive the plotter.

While remote control is on, the console is display-only: the seven segment
displays keep showing the machine position, but its buttons do nothing (the
long press on A stays available as an emergency stop unless it is turned off
in the config). Everything else comes in over this API, which makes it usable
by any program, a notebook, or a language model wrapping these calls as tools.

Endpoints (all JSON; POST bodies are JSON objects):

  GET  /api/status                  machine state, position, limits, pen, camera
  POST /api/control    {"mode": "remote"|"console"}
  POST /api/home
  POST /api/stop                    feed hold + flush, keeps position
  POST /api/move       {"x","y","z","feed","draw","wait"}   machine coordinates
  POST /api/pen        {"action": "up"|"down", "z", "feed"}
  POST /api/pen/z      {"z"}        set the drawing height
  POST /api/pen/pickup {"index"}
  POST /api/pen/return {"index"}
  POST /api/sync                    wait until everything has been executed
  POST /api/photo      {"name"}     take a picture, returns its path
  GET  /api/photo/last              the JPEG itself (most recent picture)

The server binds to localhost by default. Opening it up to the network means
anything on that network can move the machine, so set remote_api.token as well
when you do.
"""

import json
import logging
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable, Dict, Optional

from .machine_control import MachineController, MachineError


class _Handler(BaseHTTPRequestHandler):
    server_version = "AlphaPaint/1.0"
    controller: MachineController = None      # set by RemoteAPI
    state_machine = None
    token: str = ''
    logger = logging.getLogger(__name__)
    last_photo: Optional[str] = None

    # -- plumbing ---------------------------------------------------------

    def log_message(self, fmt, *args):        # quieter than the default
        self.logger.debug("api: " + fmt % args)

    def _send_json(self, code: int, payload: Dict):
        body = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _authorized(self) -> bool:
        if not self.token:
            return True
        return self.headers.get('X-Api-Token', '') == self.token

    def _body(self) -> Dict:
        length = int(self.headers.get('Content-Length') or 0)
        if not length:
            return {}
        try:
            return json.loads(self.rfile.read(length).decode() or '{}')
        except json.JSONDecodeError as e:
            raise MachineError(f"invalid JSON: {e}")

    # -- routing ----------------------------------------------------------

    def do_GET(self):
        if not self._authorized():
            return self._send_json(401, {'error': 'invalid token'})
        try:
            if self.path == '/api/status':
                return self._send_json(200, self.controller.status())
            if self.path == '/api/photo/last':
                return self._send_photo()
            return self._send_json(404, {'error': f'unknown path {self.path}'})
        except MachineError as e:
            return self._send_json(409, {'error': str(e)})
        except Exception as e:
            self.logger.error(f"api GET {self.path}: {e}", exc_info=True)
            return self._send_json(500, {'error': str(e)})

    def do_POST(self):
        if not self._authorized():
            return self._send_json(401, {'error': 'invalid token'})
        try:
            body = self._body()
            handler = self._route(self.path)
            if handler is None:
                return self._send_json(404, {'error': f'unknown path {self.path}'})
            return self._send_json(200, handler(body) or {'ok': True})
        except MachineError as e:
            return self._send_json(409, {'error': str(e)})
        except Exception as e:
            self.logger.error(f"api POST {self.path}: {e}", exc_info=True)
            return self._send_json(500, {'error': str(e)})

    def _route(self, path: str) -> Optional[Callable[[Dict], Dict]]:
        c = self.controller
        routes = {
            '/api/control': self._control,
            '/api/home': lambda body: c.home(),
            '/api/stop': lambda body: c.stop(),
            '/api/sync': lambda body: {'position': c.sync()},
            '/api/move': lambda body: c.move_to(
                x=body.get('x'), y=body.get('y'), z=body.get('z'),
                feed=body.get('feed'), draw=bool(body.get('draw')),
                wait=body.get('wait', True)),
            '/api/pen': self._pen,
            '/api/pen/z': lambda body: c.set_pen_z(float(body['z'])),
            '/api/pen/pickup': lambda body: c.pickup_pen(int(body['index'])),
            '/api/pen/return': lambda body: c.return_pen(
                int(body['index']) if 'index' in body else None),
            '/api/photo': self._photo,
        }
        return routes.get(path)

    def _control(self, body: Dict) -> Dict:
        mode = body.get('mode', 'remote')
        if mode == 'remote':
            self.state_machine.enter_remote_control()
        elif mode == 'console':
            self.state_machine.leave_remote_control()
        else:
            raise MachineError("mode must be 'remote' or 'console'")
        return {'mode': mode, 'state': self.state_machine.state}

    def _pen(self, body: Dict) -> Dict:
        action = body.get('action', 'up')
        if action == 'up':
            return self.controller.pen_up()
        if action == 'down':
            return self.controller.pen_down(z=body.get('z'), feed=body.get('feed'))
        raise MachineError("action must be 'up' or 'down'")

    def _photo(self, body: Dict) -> Dict:
        result = self.controller.photo(body.get('name'))
        _Handler.last_photo = result['path']
        return result

    def _send_photo(self):
        path = _Handler.last_photo
        if not path or not os.path.exists(path):
            return self._send_json(404, {'error': 'no photo taken yet'})
        with open(path, 'rb') as f:
            data = f.read()
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)


class RemoteAPI:
    """Runs the HTTP API in a background thread."""

    def __init__(self, state_machine, config: Dict):
        self.logger = logging.getLogger(__name__)
        self.config = config.get('remote_api', {}) or {}
        self.state_machine = state_machine
        self.controller = MachineController(state_machine, config)
        self._server: Optional[ThreadingHTTPServer] = None
        self._thread: Optional[threading.Thread] = None

    def start(self):
        if not self.config.get('enabled', False):
            self.logger.info("Remote API disabled in the config")
            return
        host = self.config.get('host', '127.0.0.1')
        port = int(self.config.get('port', 8080))
        _Handler.controller = self.controller
        _Handler.state_machine = self.state_machine
        _Handler.token = self.config.get('token', '') or ''
        try:
            self._server = ThreadingHTTPServer((host, port), _Handler)
        except Exception as e:
            self.logger.error(f"Could not start the remote API on {host}:{port}: {e}")
            return
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()
        self.logger.info(f"Remote API listening on http://{host}:{port}/api/"
                         + (" (token required)" if _Handler.token else " (no token)"))

    def stop(self):
        if self._server:
            self._server.shutdown()
            self._server.server_close()
            self._server = None
        self._thread = None
