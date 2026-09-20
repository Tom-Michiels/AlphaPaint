"""Thin client for the daemon's remote API (see Daemon/lib/remote_api.py).

Only the standard library, so it runs under the system python3 as well - handy
because that one has OpenCV for looking at the photos.

    from plotter_api import Plotter

    with Plotter() as p:          # takes control, gives it back on exit
        p.home()
        p.move(x=100, y=100)
        p.pen_down()
        p.move(x=200, y=100, draw=True)
        photo = p.photo('test')
"""

import json
import os
import urllib.error
import urllib.request
from typing import Dict, Optional


class PlotterError(Exception):
    """The machine refused or could not carry out the request."""


class Plotter:
    def __init__(self, base: str = None, token: str = None, timeout: float = 300.0):
        self.base = (base or os.environ.get('ALPHAPAINT_API', 'http://127.0.0.1:8080')).rstrip('/')
        self.token = token if token is not None else os.environ.get('ALPHAPAINT_TOKEN', '')
        self.timeout = timeout

    # -- plumbing ---------------------------------------------------------

    def _call(self, path: str, body: Optional[Dict] = None) -> Dict:
        headers = {'Content-Type': 'application/json'}
        if self.token:
            headers['X-Api-Token'] = self.token
        request = urllib.request.Request(
            self.base + path,
            data=json.dumps(body).encode() if body is not None else None,
            headers=headers,
            method='POST' if body is not None else 'GET')
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                return json.loads(response.read() or '{}')
        except urllib.error.HTTPError as e:
            try:
                message = json.loads(e.read()).get('error', str(e))
            except Exception:
                message = str(e)
            raise PlotterError(message) from None
        except urllib.error.URLError as e:
            raise PlotterError(f"cannot reach the daemon at {self.base}: {e.reason}") from None

    def __enter__(self):
        self.take_control()
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            self.pen_up()
        except PlotterError:
            pass
        self.release_control()
        return False

    # -- control ----------------------------------------------------------

    def status(self) -> Dict:
        return self._call('/api/status')

    def take_control(self) -> Dict:
        return self._call('/api/control', {'mode': 'remote'})

    def release_control(self) -> Dict:
        return self._call('/api/control', {'mode': 'console'})

    def home(self) -> Dict:
        return self._call('/api/home', {})

    def stop(self) -> Dict:
        return self._call('/api/stop', {})

    def sync(self) -> Dict:
        return self._call('/api/sync', {})

    # -- motion -----------------------------------------------------------

    def move(self, x=None, y=None, z=None, feed=None, draw=False, wait=True) -> Dict:
        body = {'draw': draw, 'wait': wait}
        for key, value in (('x', x), ('y', y), ('z', z), ('feed', feed)):
            if value is not None:
                body[key] = value
        return self._call('/api/move', body)

    def draw_to(self, x=None, y=None, feed=None) -> Dict:
        return self.move(x=x, y=y, feed=feed, draw=True)

    def pen_up(self) -> Dict:
        return self._call('/api/pen', {'action': 'up'})

    def pen_down(self, z=None, feed=None) -> Dict:
        body = {'action': 'down'}
        if z is not None:
            body['z'] = z
        if feed is not None:
            body['feed'] = feed
        return self._call('/api/pen', body)

    def set_pen_z(self, z: float) -> Dict:
        return self._call('/api/pen/z', {'z': z})

    def pickup_pen(self, index: int) -> Dict:
        return self._call('/api/pen/pickup', {'index': index})

    def return_pen(self, index: int = None) -> Dict:
        return self._call('/api/pen/return', {} if index is None else {'index': index})

    # -- camera -----------------------------------------------------------

    def photo(self, name: str = None) -> Dict:
        return self._call('/api/photo', {} if name is None else {'name': name})

    def position(self) -> Dict:
        return self.status()['position']
