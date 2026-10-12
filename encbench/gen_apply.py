#!/usr/bin/env python3
"""Generate simdjson on-demand decoders that apply a JSON merge-patch delta
straight into the EmcStat object-API types, from the same .fbs schema."""
import sys
sys.argv, files = sys.argv[:1], sys.argv[1:]
import re

def parse(path):
    tables, structs, enums = {}, {}, set()
    cur = None
    for line in open(path):
        line = line.split('//')[0].strip()
        m = re.match(r'(table|struct)\s+(\w+)', line)
        if m:
            cur = (m.group(1), m.group(2), [])
            continue
        m = re.match(r'enum\s+(\w+)', line)
        if m:
            enums.add(m.group(1)); cur = None; continue
        if line.startswith('}'):
            if cur:
                (tables if cur[0] == 'table' else structs)[cur[1]] = cur[2]
            cur = None; continue
        m = re.match(r'(\w+)\s*:\s*([\w\[\]]+)', line)
        if m and cur:
            cur[2].append((m.group(1), m.group(2)))
    return tables, structs, enums

T, S, E = {}, {}, set()
for p in files:
    t, s, e = parse(p); T.update(t); S.update(s); E |= e

SIGNED = {'byte', 'short', 'int', 'long', 'int8', 'int16', 'int32', 'int64'}
UNSIGNED = {'ubyte', 'ushort', 'uint', 'ulong', 'uint8', 'uint16', 'uint32', 'uint64'}

def scalar(ty, val):
    if ty in ('double', 'float', 'float32', 'float64'): return f'double({val}.get_double())'
    if ty == 'bool': return f'bool({val}.get_bool())'
    if ty in SIGNED: return f'int64_t({val}.get_int64())'
    if ty in UNSIGNED: return f'uint64_t({val}.get_uint64())'
    if ty == 'string': return f'std::string(std::string_view({val}.get_string()))'
    raise SystemExit(f'unhandled type {ty}')

out = ['#pragma once', '#include <simdjson.h>', '#include <charconv>', '#include "gen/emc_ws_generated.h"',
       'namespace EMC {', 'namespace od = simdjson::ondemand;']
for name in T:
    out.append(f'inline void apply_json(od::object o, {name}T &v);')
for name, fields in S.items():
    out.append(f'inline void apply_json(od::object o, {name} &v) {{')
    out.append('  for (auto f : o) { std::string_view k = f.unescaped_key(); od::value x = f.value();')
    for i, (fn, ty) in enumerate(fields):
        kw = 'if' if i == 0 else 'else if'
        out.append(f'    {kw} (k == "{fn}") v.mutate_{fn}({scalar(ty, "x")});')
    out.append('  }')
    out.append('}')
for name, fields in T.items():
    out.append(f'inline void apply_json(od::object o, {name}T &v) {{')
    out.append('  for (auto f : o) { std::string_view k = f.unescaped_key(); od::value x = f.value();')
    for i, (fn, ty) in enumerate(fields):
        kw = 'if' if i == 0 else 'else if'
        if ty.startswith('['):
            inner = ty[1:-1]
            if inner in T:
                out.append(f'    {kw} (k == "{fn}") {{ for (auto e : x.get_object()) {{ std::string_view ik = e.unescaped_key(); size_t i = 0;')
                out.append(f'        std::from_chars(ik.data(), ik.data() + ik.size(), i);')
                out.append(f'        if (i >= v.{fn}.size()) v.{fn}.resize(i + 1);')
                out.append(f'        if (!v.{fn}[i]) v.{fn}[i] = std::make_unique<{inner}T>();')
                out.append(f'        apply_json(e.value().get_object(), *v.{fn}[i]); }} }}')
            else:
                out.append(f'    {kw} (k == "{fn}") {{ v.{fn}.clear(); for (auto e : x.get_array()) {{ od::value y = e.value(); v.{fn}.push_back({scalar(inner, "y")}); }} }}')
        elif ty in T:
            out.append(f'    {kw} (k == "{fn}") {{ if (!v.{fn}) v.{fn} = std::make_unique<{ty}T>(); apply_json(x.get_object(), *v.{fn}); }}')
        elif ty in S:
            out.append(f'    {kw} (k == "{fn}") {{ if (!v.{fn}) v.{fn} = std::make_unique<{ty}>(); apply_json(x.get_object(), *v.{fn}); }}')
        elif ty in E:
            out.append(f'    {kw} (k == "{fn}") v.{fn} = static_cast<{ty}>(int64_t(x.get_int64()));')
        else:
            out.append(f'    {kw} (k == "{fn}") v.{fn} = {scalar(ty, "x")};')
    out.append('  }')
    out.append('}')
out.append('}')
print('\n'.join(out))
