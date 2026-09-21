#!/usr/bin/env python3
"""config/s10_params.yaml is the ONE parameter file. This tool keeps everything else in step with it.

  python3 tools/params.py sync            # regenerate config/nav.yaml, config/control.yaml, config/web_nav.json (every
                                          # start script runs this first, so the generated files can never drift)
  python3 tools/params.py check           # does the master still match the lock? (exit 0 yes, 1 no)
  python3 tools/params.py lock "<note>"   # record the current master as the approved one (after the operator agreed)
  python3 tools/params.py show [section]  # print the values (run | web | route_build | nav | control)
  python3 tools/params.py diff            # what changed since the lock (section.key: locked -> now)

The lock (config/s10_params.lock) holds the sha256 of the master, the approved values and a history of notes.
"""
import hashlib
import json
import os
import sys
import time

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MASTER = os.path.join(ROOT, "config", "s10_params.yaml")
LOCK = os.path.join(ROOT, "config", "s10_params.lock")
BANNER = ("# GENERATED from config/s10_params.yaml (section `%s`) by tools/params.py sync. DO NOT EDIT: this file is\n"
          "# overwritten at every start. Change the master file instead (it is LOCKED: operator approval needed).\n")


def load():
    with open(MASTER) as f:
        return yaml.safe_load(f)


def section_text(name):
    """The master's text of one top-level section, dedented by two spaces, comments kept. Everything inside a section
    is indented or blank; the first unindented line (the next section's header comment) ends it."""
    out, inside = [], False
    for line in open(MASTER).read().splitlines(True):
        if inside:
            if line.strip() and not line.startswith("  "):
                break
            out.append(line[2:] if line.startswith("  ") else line)
        elif line.rstrip() == name + ":":
            inside = True
    return "".join(out).rstrip("\n") + "\n"


def write_if_changed(path, text):
    try:
        if open(path).read() == text:
            return False
    except OSError:
        pass
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(text)
    os.replace(tmp, path)
    return True


def sync(quiet=False):
    doc = load()
    changed = []
    for name, target in (("nav", "nav.yaml"), ("control", "control.yaml")):
        text = BANNER % name + section_text(name)
        if yaml.safe_load(text) != doc[name]:
            sys.exit(f"params sync: the generated {target} would not equal section `{name}` of the master; nothing written")
        if write_if_changed(os.path.join(ROOT, "config", target), text):
            changed.append(target)
    web = dict(doc["web"])
    web["_generated_from"] = "config/s10_params.yaml section web: DO NOT EDIT"
    if write_if_changed(os.path.join(ROOT, "config", "web_nav.json"), json.dumps(web, ensure_ascii=False) + "\n"):
        changed.append("web_nav.json")
    if not quiet or changed:
        print("params: " + (("regenerated " + ", ".join(changed)) if changed else "generated files up to date") + " | " + status_line())
    return changed


def sha():
    return hashlib.sha256(open(MASTER, "rb").read()).hexdigest()


def read_lock():
    try:
        return json.load(open(LOCK))
    except (OSError, ValueError):
        return None


def flat(d, prefix=""):
    out = {}
    for k, v in (d or {}).items():
        if isinstance(v, dict):
            out.update(flat(v, prefix + str(k) + "."))
        else:
            out[prefix + str(k)] = v
    return out


def changes():
    lk = read_lock()
    if lk is None:
        return None
    a, b = flat(lk.get("values")), flat(load())
    return [(k, a.get(k, "<absent>"), b.get(k, "<absent>")) for k in sorted(set(a) | set(b)) if a.get(k, "<absent>") != b.get(k, "<absent>")]


def status_line():
    lk = read_lock()
    if lk is None:
        return "NOT LOCKED yet (tools/params.py lock \"<note>\")"
    if lk.get("sha256") == sha():
        return "parameters = locked version %s (%s)" % (lk["sha256"][:8], lk["history"][-1]["when"])
    ch = changes() or []
    if not ch:
        return "parameters: comments/layout changed since the lock %s, values identical" % lk["sha256"][:8]
    return "WARNING: %d parameter value(s) differ from the locked version %s: %s" % (
        len(ch), lk["sha256"][:8], "; ".join("%s: %s -> %s" % c for c in ch[:6]) + (" ..." if len(ch) > 6 else ""))


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "check"
    if cmd == "sync":
        sync(quiet="--quiet" in sys.argv)
    elif cmd == "check":
        print(status_line())
        lk = read_lock()
        sys.exit(0 if lk is not None and not changes() else 1)
    elif cmd == "diff":
        ch = changes()
        print("not locked yet" if ch is None else "\n".join("%s: %s -> %s" % c for c in ch) or "no value differs from the lock")
    elif cmd == "lock":
        note = " ".join(a for a in sys.argv[2:]).strip()
        if not note:
            sys.exit('lock needs a note: what changed and who approved it, e.g. lock "speed 1.0 -> 1.4, approved by Bowen"')
        lk = read_lock() or dict(history=[])
        lk.update(sha256=sha(), values=load())
        lk["history"].append(dict(when=time.strftime("%Y-%m-%d %H:%M"), sha256=lk["sha256"][:8], note=note))
        with open(LOCK, "w") as f:
            json.dump(lk, f, indent=1, ensure_ascii=False, default=str)
        sync(quiet=True)
        print("locked " + lk["sha256"][:8] + ": " + note)
    elif cmd == "show":
        doc = load()
        sec = sys.argv[2] if len(sys.argv) > 2 else None
        for k, v in sorted(flat(doc[sec] if sec else doc, (sec + ".") if sec else "").items()):
            print("%-52s %s" % (k, v))
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
