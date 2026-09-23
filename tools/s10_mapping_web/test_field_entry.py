"""Homepage entry/auth regression, local HTTP only; no robot/backend calls."""
import hashlib
import json
import threading
import unittest
from http.server import ThreadingHTTPServer
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import HTTPCookieProcessor, ProxyHandler, Request, build_opener

import server as web


class FieldEntryTests(unittest.TestCase):
    def test_home_entry_keeps_auth_and_same_field_destination(self):
        with patch.object(web, 'config', dict(login_hash=hashlib.sha256(b'demo:entry-only').hexdigest())), \
             patch.object(web, 'call') as backend:
            http = ThreadingHTTPServer(('127.0.0.1', 0), web.Handler)
            threading.Thread(target=http.serve_forever, daemon=True).start()
            client = build_opener(ProxyHandler({}), HTTPCookieProcessor())
            base = f'http://127.0.0.1:{http.server_port}'

            def request(path, body=None):
                req = Request(base+path, data=json.dumps(body).encode() if body is not None else None,
                              headers={'Content-Type': 'application/json'})
                try:
                    response = client.open(req, timeout=3)
                except HTTPError as exc:
                    response = exc
                with response:
                    return response.code, response.read().decode(), response.url

            try:
                code, home, _ = request('/')
                self.assertEqual(code, 200)
                self.assertIn('id="fieldAssistant" class="field-button" href="/field"', home)
                self.assertIn('<strong>现场助手</strong>', home)
                self.assertIn('min-height:76px', home)
                for old_link in ('href="/localization"', 'href="/heightmap"', 'id="start"', 'id="save"'):
                    self.assertIn(old_link, home)

                code, locked, url = request('/field')
                self.assertEqual(code, 200)
                self.assertEqual(url, base+'/field')
                self.assertIn('id="login"', locked)
                self.assertNotIn('src="/field.js"', locked)
                self.assertIn("if(location.pathname!=='/')location.reload()", locked)
                self.assertEqual(request('/field.js')[0], 401)
                self.assertEqual(request('/phone/field/health')[0], 401)

                self.assertEqual(request('/phone/login', dict(username='demo', password='entry-only'))[0], 200)
                code, field, url = request('/field')
                self.assertEqual(code, 200)
                self.assertEqual(url, base+'/field')
                self.assertIn('src="/field.js"', field)
                self.assertNotIn('id="login"', field)
                self.assertEqual(request('/field.js')[0], 200)
                self.assertEqual(request('/phone/field/submit', {})[0], 403)
                backend.assert_not_called()
            finally:
                http.shutdown(); http.server_close()


if __name__ == '__main__':
    unittest.main(verbosity=2)
