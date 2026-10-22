import json
import os
from pathlib import Path
import socket
import tempfile
import threading
import unittest
from types import SimpleNamespace as NS
from supervisor_rpc import SupervisorRPC, dispatch


class SupervisorTests(unittest.TestCase):
    def test_private_socket_command_snapshot_and_no_arbitrary_operation(self):
        calls = []
        panel = NS(lock=threading.RLock(), navigation_costmap={'available': False},
            saved_map=lambda: ({'available': False}, None), state=lambda: {'mode': 'idle'},
            request_mode=lambda mode: calls.append(mode))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'supervisor.sock'
            server = SupervisorRPC(path, lambda request: dispatch(panel, request))
            def request(body):
                with socket.socket(socket.AF_UNIX) as client:
                    client.settimeout(1)
                    client.connect(str(path))
                    client.sendall(json.dumps(body).encode()+b'\n')
                    return json.loads(client.makefile('rb').readline())
            try:
                self.assertEqual(path.stat().st_mode & 0o777, 0o600)
                self.assertEqual(request({'operation': 'snapshot'})['body']['state']['mode'], 'idle')
                self.assertEqual(request({'operation': 'costmap'})['body'], {'available': False})
                result = request({'operation': 'command', 'path': '/api/mode', 'body': {'mode': 'mapping'}})
                self.assertEqual(result['status'], 200)
                self.assertEqual(calls, ['mapping'])
                result = request({'operation': 'command', 'path': '/shell', 'body': {}})
                self.assertEqual(result['status'], 400)
                self.assertEqual(calls, ['mapping'])
                with self.assertRaisesRegex(RuntimeError, 'already active'):
                    SupervisorRPC(path, lambda _: {})
            finally:
                server.close()
            self.assertFalse(path.exists())

    def test_occupied_file_is_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'supervisor.sock'
            path.write_text('existing data')
            with self.assertRaises(RuntimeError):
                SupervisorRPC(path, lambda _: {})
            self.assertEqual(path.read_text(), 'existing data')

    def test_invalid_request_length_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'s.sock'
            server = SupervisorRPC(path, lambda _: self.fail('Malformed request was dispatched'))
            try:
                with socket.socket(socket.AF_UNIX) as client:
                    client.settimeout(1);client.connect(str(path));client.sendall(b' '*16385+b'\n')
                    self.assertEqual(json.loads(client.makefile('rb').readline())['status'], 400)
            finally:
                server.close()


if __name__ == '__main__':
    unittest.main()
