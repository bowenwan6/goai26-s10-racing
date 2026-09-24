"""Local synthetic demo using installed Flask; no ROS or robot connection."""
import secrets
from pathlib import Path

from server import Recorder, create_app

if __name__ == '__main__':
    output = Path(__file__).resolve().parents[2] / 'out' / 's10-capture-demo'
    output.mkdir(parents=True, exist_ok=True)
    token_file = output / '.access-token'
    if not token_file.exists():
        token_file.write_text(secrets.token_urlsafe(24), encoding='utf-8')
    recorder = Recorder(output, demo=True)
    recorder.start_worker()
    try:
        create_app(recorder, token=token_file.read_text().strip()).run(
            host='127.0.0.1', port=8090, debug=False, use_reloader=False)
    finally:
        recorder.quit.set()
        with recorder.lock:
            if recorder.active:
                recorder._stop('interrupted', 'Local demo closed')
