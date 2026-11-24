#!/usr/bin/env python3
"""Check the model examples in the documentation with the real RosTooling language server.

An example is a fenced code block preceded by an HTML comment:

    <!-- check: group=<name> file=<file name> [expect=<text>] -->

Blocks with the same group are written to one folder and validated together, so a
.rossystem example can refer to the .ros2 example it needs. Without `expect`, a block
must produce no ERROR and no WARNING. With `skip="<reason>"` the group is listed but not run (for a documented feature
that the shipped server does not accept yet; remove the attribute once it does).
With `expect=<text>`, the group must produce a
diagnostic that contains <text>, and the check fails if it does not (used for the
examples that show an error on purpose).

    python3 tools/check_examples.py --server /path/to/ipa-rostooling-studio/scripts/ros_server.py

The server script comes from the ipa-rostooling-studio repository; it finds a
RosTooling language server JAR and Java 21. Set ROSTOOLING_SERVER_SCRIPT to avoid
passing --server every time.
"""

import argparse
import importlib.util
import os
import re
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MARK = re.compile(r"<!--\s*check:\s*(?P<attrs>.*?)\s*-->\s*\n```[^\n]*\n(?P<body>.*?)\n```", re.S)
ATTR = re.compile(r'(\w+)=("[^"]*"|\S+)')


def collect():
    groups = {}
    for md in sorted(ROOT.rglob("*.md")):
        rel = md.relative_to(ROOT)
        if rel.parts[0] in ("_build", ".github", "tools", "venv", ".venv") or "old" in rel.parts:
            continue
        for m in MARK.finditer(md.read_text(encoding="utf-8")):
            attrs = {k: v.strip('"') for k, v in ATTR.findall(m.group("attrs"))}
            if "group" not in attrs or "file" not in attrs:
                sys.exit(f"{rel}: a check comment needs group= and file=: {m.group('attrs')}")
            key = f"{rel}::{attrs['group']}"
            g = groups.setdefault(key, {"files": {}, "expect": None, "skip": None})
            g["files"][attrs["file"]] = m.group("body") + "\n"
            if "expect" in attrs:
                g["expect"] = attrs["expect"]
            if "skip" in attrs:
                g["skip"] = attrs["skip"]
    return groups


def load_server(path):
    spec = importlib.util.spec_from_file_location("ros_server", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--server", default=os.environ.get("ROSTOOLING_SERVER_SCRIPT"))
    ap.add_argument("--list", action="store_true", help="only list the examples found")
    ap.add_argument("--include-skipped", action="store_true", help="also run the groups marked skip=")
    args = ap.parse_args()
    groups = collect()
    if args.list:
        for key, g in groups.items():
            print(key, sorted(g["files"]), "expect=" + g["expect"] if g["expect"] else "")
        return 0
    if not args.server:
        sys.exit("pass --server <path to ros_server.py> or set ROSTOOLING_SERVER_SCRIPT")
    server = load_server(args.server)
    ok, reason = server.preflight()
    if not ok:
        sys.exit(f"language server not available: {reason}")
    failures = 0
    skipped = 0
    with tempfile.TemporaryDirectory() as tmp:
        for key, g in groups.items():
            if g["skip"] and not args.include_skipped:
                print(f"skip {key} ({g['skip']})")
                skipped += 1
                continue
            folder = Path(tmp) / re.sub(r"\W+", "_", key)
            folder.mkdir()
            paths = []
            for name, body in g["files"].items():
                (folder / name).write_text(body, encoding="utf-8")
                paths.append(str(folder / name))
            result = server.validate_files(paths)
            if result["status"] != "OK":
                print(f"FAIL {key}: server {result['status']} {result['reason']}")
                failures += 1
                continue
            diags = [(Path(p).name, d) for p, ds in result["diagnostics"].items() for d in ds]
            bad = [(f, d) for f, d in diags if d["severity"] in ("ERROR", "WARNING")]
            if g["expect"]:
                if any(g["expect"] in d["message"] for _, d in diags):
                    print(f"ok   {key} (expected: {g['expect']})")
                else:
                    print(f"FAIL {key}: expected a diagnostic containing {g['expect']!r}, got {[d['message'] for _, d in diags]}")
                    failures += 1
            elif bad:
                print(f"FAIL {key}")
                for f, d in bad:
                    print(f"     {f}:{d['line']} {d['severity']} {d['message']}")
                failures += 1
            else:
                print(f"ok   {key}")
    print(f"{len(groups)} example groups, {failures} failed, {skipped} skipped")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
