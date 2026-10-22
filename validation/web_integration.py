#!/usr/bin/env python3
"""Real C++ HTTP/WebSocket smoke test with a private synthetic supervisor."""
import argparse
import base64
import hashlib
import http.client as http_client
import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time

SOURCE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SOURCE/'web_panel'))
from supervisor_rpc import SupervisorRPC


class WebSocket:
    def __init__(self, port):
        self.socket = socket.create_connection(('127.0.0.1', port), timeout=2)
        self.buffer = bytearray()
        key = base64.b64encode(os.urandom(16)).decode()
        self.socket.sendall((f'GET /ws HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\n'
            f'Upgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n').encode())
        response = b''
        while b'\r\n\r\n' not in response:
            response += self.socket.recv(4096)
        header, tail = response.split(b'\r\n\r\n', 1)
        assert b'101' in header.split(b'\r\n')[0], header
        expected = base64.b64encode(hashlib.sha1((key+'258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest())
        assert expected in header
        self.buffer.extend(tail)

    def read(self, size):
        while len(self.buffer) < size:
            data = self.socket.recv(16384)
            if not data:
                raise EOFError('WebSocket closed')
            self.buffer.extend(data)
        out = bytes(self.buffer[:size]);del self.buffer[:size]
        return out

    def frame(self):
        first, second = self.read(2)
        size = second & 127
        if size == 126:
            size = struct.unpack('!H', self.read(2))[0]
        elif size == 127:
            size = struct.unpack('!Q', self.read(8))[0]
        assert size <= 2*1024*1024
        mask = self.read(4) if second & 128 else None
        data = self.read(size)
        if mask:
            data = bytes(value ^ mask[i % 4] for i, value in enumerate(data))
        return first & 15, data

    def send(self, value):
        payload = json.dumps(value).encode();mask = os.urandom(4)
        header = bytes([0x81, 0x80 | len(payload)]) if len(payload) < 126 else b'\x81\xfe'+struct.pack('!H', len(payload))
        self.socket.sendall(header+mask+bytes(value ^ mask[i % 4] for i, value in enumerate(payload)))

    def close(self):
        self.socket.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('binary', type=Path)
    args = parser.parse_args()
    if not args.binary.is_file():
        parser.error('Build the standalone C++ web executable first')
    with tempfile.TemporaryDirectory(prefix='hw-web-') as directory:
        root = Path(directory);maps = root/'maps';maps.mkdir()
        image = maps/'site.png'
        image.write_bytes(base64.b64decode('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+jC1sAAAAASUVORK5CYII='))
        mode = 'idle';calls = [];sequence = 0;escape = False
        def handle(request):
            nonlocal mode, sequence
            if request['operation'] == 'costmap':
                return {'available': False}
            if request['operation'] == 'snapshot':
                sequence += 1
                return {'state': {'mode': mode, 'demo': True},
                    'map': {'available': True, 'revision': 'fixture-map', 'image_url': '/api/map/image?revision=fixture-map'},
                    'image_path': str(root/'outside.png' if escape else image),
                    'cloud': {'sequence': sequence, 'frame': 'map', 'points': [[1., 2., 3.], [-1., 0., .5]]}}
            if request['operation'] == 'command':
                calls.append(request)
                if request['path'] == '/api/mode':
                    mode = request['body']['mode']
                    return {'mode': mode}
                raise RuntimeError('Fixture rejects this operation')
            raise ValueError('Unsupported operation')
        server = SupervisorRPC(root/'s.sock', handle)
        with socket.socket() as reserved:
            reserved.bind(('127.0.0.1', 0));port = reserved.getsockname()[1]
        def http(path, body=None, token='web-test-secret'):
            connection = http_client.HTTPConnection('127.0.0.1', port, timeout=3)
            headers = {'Authorization': 'Bearer '+token}
            if body is not None:
                headers['Content-Type'] = 'application/json'
            connection.request('POST' if body is not None else 'GET', path,
                body=None if body is None else json.dumps(body), headers=headers)
            reply = connection.getresponse();result = reply.status, {k.lower(): v for k, v in reply.getheaders()}, reply.read();connection.close()
            return result
        log = (root/'web.log').open('w+')
        process = subprocess.Popen([str(args.binary.resolve()), '--fixture', '--socket', str(server.path),
            '--assets', str(SOURCE/'web_panel'), '--maps-root', str(maps), '--port', str(port)],
            env={**os.environ, 'SCURM_WEB_TOKEN': 'web-test-secret'}, stdout=log, stderr=subprocess.STDOUT)
        peers = []
        try:
            deadline = time.monotonic()+10
            while True:
                try:
                    if http('/api/state')[0] == 200:
                        break
                except (OSError, http_client.HTTPException):
                    pass
                if process.poll() is not None or time.monotonic() > deadline:
                    log.seek(0);raise AssertionError('C++ web startup failed: '+log.read())
                time.sleep(.05)
            assert http('/')[0] == 200
            assert http('/stream.js')[0] == 200
            assert http('/server.py')[0] == 404
            assert http('/api/state', token='wrong')[0] == 401
            assert http('/api/mode', {'mode': 'mapping'}, token='wrong')[0] == 401 and not calls
            status, headers, body = http('/api/state')
            assert status == 200 and json.loads(body)['mode'] == 'idle'
            assert json.loads(body)['transport']['websocket']
            assert headers.get('cache-control') == 'no-store'
            assert http('/api/map/image?revision=fixture-map')[2] == image.read_bytes()
            assert http('/api/map/image?revision=old')[0] == 409
            assert http('/api/mode', {'mode': 'mapping'})[0] == 202
            assert len(calls) == 1 and mode == 'mapping'
            assert http('/api/navigation/cancel', {})[0] == 409
            peer = WebSocket(port);peers.append(peer)
            peer.socket.settimeout(.3)
            try:
                peer.frame();raise AssertionError('Unauthenticated stream leaked data')
            except socket.timeout:
                pass
            peer.socket.settimeout(2)
            peer.send({'type': 'auth', 'token': 'web-test-secret'})
            assert json.loads(peer.frame()[1])['type'] == 'ready'
            opcode, data = peer.frame();packet = json.loads(data)
            assert opcode == 1 and packet['type'] == 'state' and packet['cloud']
            opcode, data = peer.frame();assert opcode == 2 and data[:4] == b'SCPC'
            length = struct.unpack_from('<I', data, 4)[0];metadata = json.loads(data[8:8+length])
            assert metadata['count'] == 2 and metadata['sequence'] == packet['cloud_sequence']
            assert struct.unpack_from('<3f', data, 8+length) == (1., 2., 3.)
            peer.socket.settimeout(.35)
            try:
                peer.frame();raise AssertionError('Slow client received unbounded queued frames')
            except socket.timeout:
                pass
            peer.socket.settimeout(2);peer.send({'type': 'ack', 'serial': packet['serial']})
            assert json.loads(peer.frame()[1])['serial'] > packet['serial']
            bad = WebSocket(port);peers.append(bad);bad.send({'type': 'auth', 'token': 'wrong'})
            opcode, data = bad.frame();assert opcode == 8 and struct.unpack_from('!H', data)[0] == 1008
            (root/'outside.png').write_bytes(image.read_bytes());escape = True
            time.sleep(.3);assert http('/api/map/image?revision=fixture-map')[0] == 409
            server.close();time.sleep(1.3)
            assert http('/api/state')[0] == 503
            print('C++ web integration passed: HTTP/auth, command dispatch, map scope, binary WebSocket, ACK backpressure and supervisor loss')
        finally:
            for peer in peers:
                peer.close()
            if process.poll() is None:
                process.terminate()
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:process.kill();process.wait(timeout=2)
            if server.path.exists():
                server.close()
            log.close()


if __name__ == '__main__':
    main()
