#!/usr/bin/env python3
"""Static checks for every Node-RED flow in the repo: valid JSON, unique ids, no wire or
group/tab/broker reference to a missing node, every function node parses as JavaScript.
    python3 node-red/validate_flows.py
"""
import glob, json, os, subprocess, sys, tempfile
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
files = sorted(set(glob.glob(os.path.join(ROOT, '*.json')) + glob.glob(os.path.join(ROOT, 'node-red', '*.json')) +
                   glob.glob(os.path.join(ROOT, 'power-management', '**', 'node-red', '*.json'), recursive=True)))
bad = 0
for f in files:
    rel = os.path.relpath(f, ROOT)
    try:
        d = json.load(open(f))
    except ValueError as e:
        print('FAIL', rel, e); bad += 1; continue
    if not isinstance(d, list) or not all(isinstance(n, dict) and 'type' in n for n in d):
        continue                                   # not a flow export
    ids = [n['id'] for n in d]
    errs = [f'duplicate id {i}' for i in set(ids) if ids.count(i) > 1]
    idset = set(ids)
    for n in d:
        for out in n.get('wires', []):
            errs += [f"{n['id']} wires to missing {w}" for w in out if w not in idset]
        for ref in ('group', 'tab', 'broker', 'serialport', 'z'):
            if n['type'] in ('mqtt-broker', 'serial-port') and ref in ('broker', 'serialport'):
                continue                           # config nodes: host name / port path, not a reference
            if ref in n and n[ref] and n[ref] not in idset:
                errs.append(f"{n['id']} {ref} -> missing {n[ref]}")
        if n['type'] == 'function':
            with tempfile.NamedTemporaryFile('w', suffix='.js', delete=False) as t:
                t.write('async function __f(msg,node,context,flow,global,env){\n' + n.get('func', '') + '\n}\n')
            r = subprocess.run(['node', '--check', t.name], capture_output=True, text=True)
            os.unlink(t.name)
            if r.returncode:
                errs.append(f"function {n.get('name')} does not parse: {r.stderr.strip().splitlines()[-1]}")
    print(('OK  ' if not errs else 'FAIL') + f' {rel} ({len(d)} nodes)')
    for e in errs:
        print('     ', e)
    bad += bool(errs)
sys.exit(1 if bad else 0)
