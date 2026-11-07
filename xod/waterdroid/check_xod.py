#!/usr/bin/env python3
"""Syntax-check every XOD C++ node with g++ against a tiny stub of the XOD node API.
(The real check is opening the project in XOD IDE; this catches C++ mistakes on a PC.)
    python3 xod/waterdroid/check_xod.py
"""
import glob, json, os, subprocess, sys, tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOTS = [HERE, os.path.join(HERE, '..', '..', 'power-management', 'power-management', 'xod', 'power-management')]
CT = {'boolean': 'bool', 'number': 'Number', 'pulse': 'bool', 'string': 'XString'}
ok = True
for root in ROOTS:
    for xodp in sorted(glob.glob(os.path.join(root, '*', 'patch.xodp'))):
        spec = json.load(open(xodp))
        decl = ['#include <stdint.h>', 'typedef double Number;', 'struct CStringView { CStringView(const char*) {} };',
                'struct XString { XString(CStringView*) {} XString() {} };', 'struct Context {};',
                'static uint32_t millis() { return 0; }', 'template<class T> bool isInputDirty(Context) { return false; }',
                'static void setTimeout(Context, uint32_t) {}', '#define INPUT 0', '#define FALLING 2',
                'static void pinMode(int, int) {}', 'static int digitalPinToInterrupt(int p) { return p; }',
                'static void attachInterrupt(int, void (*)(), int) {}', '#include <math.h>', 'static bool isTimedOut(Context) { return false; }']
        for n in spec['nodes']:
            t = n['type'].rsplit('/', 1)[1]
            if t.startswith('input-') or t.startswith('output-'):
                d, k = t.split('-', 1)
                decl.append(f"struct {d}_{n['label']} {{ typedef {CT[k]} T; }};")
        decl += ['template<class P> typename P::T getValue(Context) { return typename P::T(); }',
                 'template<class P, class V> void emitValue(Context, V) {}', '#define node struct XodNode']
        body = open(os.path.join(os.path.dirname(xodp), 'patch.cpp')).read().rstrip()
        body = body[:-1] + '};' if body.endswith('}') else body      # XOD's `node {}` needs no ';'
        src = '\n'.join(decl) + '\n' + body
        src += '\nint main() { XodNode n; Context c; n.evaluate(c); return 0; }\n'
        with tempfile.NamedTemporaryFile('w', suffix='.cpp', delete=False) as f:
            f.write(src)
        r = subprocess.run(['g++', '-std=c++11', '-Wall', '-Wno-unused-variable', '-Wno-unused-function',
                            '-fsyntax-only', f.name], capture_output=True, text=True)
        name = os.path.basename(os.path.dirname(xodp))
        print(('OK   ' if r.returncode == 0 else 'FAIL ') + name)
        if r.returncode:
            ok = False
            print(r.stderr[:2000])
        os.unlink(f.name)
sys.exit(0 if ok else 1)
