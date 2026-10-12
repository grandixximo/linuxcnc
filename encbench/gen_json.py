#!/usr/bin/env python3
"""Generate nlohmann::json converters for the EmcStat object-API types
from emc_stat.fbs / emc_common.fbs, so the JSON encoder is schema-driven
exactly like the FlatBuffers one."""
import re, sys

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
for p in sys.argv[1:]:
    t, s, e = parse(p); T.update(t); S.update(s); E |= e

out = ['#pragma once', '#include <nlohmann/json.hpp>', '#include "gen/emc_ws_generated.h"', 'namespace EMC {']
for name, fields in S.items():
    body = ', '.join(f'{{"{f}", v.{f}()}}' for f, _ in fields)
    out.append(f'inline void to_json(nlohmann::json &j, const {name} &v) {{ j = nlohmann::json{{{body}}}; }}')
for name in T:
    out.append(f'inline void to_json(nlohmann::json &j, const {name}T &v);')
for name, fields in T.items():
    out.append(f'inline void to_json(nlohmann::json &j, const {name}T &v) {{')
    out.append('  j = nlohmann::json::object();')
    for f, ty in fields:
        if ty.startswith('['):
            inner = ty[1:-1]
            if inner in T:
                out.append(f'  {{ auto &a = j["{f}"] = nlohmann::json::array(); for (auto &e : v.{f}) {{ nlohmann::json x; if (e) to_json(x, *e); a.push_back(std::move(x)); }} }}')
            else:
                out.append(f'  j["{f}"] = v.{f};')
        elif ty in T:
            out.append(f'  if (v.{f}) to_json(j["{f}"], *v.{f}); else j["{f}"] = nullptr;')
        elif ty in S:
            out.append(f'  if (v.{f}) to_json(j["{f}"], *v.{f}); else j["{f}"] = nullptr;')
        elif ty in E:
            out.append(f'  j["{f}"] = static_cast<int>(v.{f});')
        else:
            out.append(f'  j["{f}"] = v.{f};')
    out.append('}')
# field-by-field diff on the native object-API types
for name in T:
    out.append(f'inline void diff_json(nlohmann::json &j, const {name}T &a, const {name}T &b);')
for name, fields in S.items():
    cond = ' || '.join(f'a.{f}() != b.{f}()' for f, _ in fields)
    out.append(f'inline void diff_json(nlohmann::json &j, const {name} &a, const {name} &b) {{')
    for f, _ in fields:
        out.append(f'  if (a.{f}() != b.{f}()) j["{f}"] = b.{f}();')
    out.append('}')
for name, fields in T.items():
    out.append(f'inline void diff_json(nlohmann::json &j, const {name}T &a, const {name}T &b) {{')
    for f, ty in fields:
        if ty.startswith('['):
            inner = ty[1:-1]
            if inner in T:
                out.append(f'  {{ nlohmann::json o = nlohmann::json::object(); size_t n = b.{f}.size();')
                out.append(f'    for (size_t i = 0; i < n; ++i) {{ nlohmann::json x = nlohmann::json::object();')
                out.append(f'      if (i < a.{f}.size() && a.{f}[i] && b.{f}[i]) diff_json(x, *a.{f}[i], *b.{f}[i]); else if (b.{f}[i]) to_json(x, *b.{f}[i]);')
                out.append(f'      if (!x.empty()) o[std::to_string(i)] = std::move(x); }}')
                out.append(f'    if (!o.empty()) j["{f}"] = std::move(o); }}')
            else:
                out.append(f'  if (a.{f} != b.{f}) j["{f}"] = b.{f};')
        elif ty in T or ty in S:
            out.append(f'  if (a.{f} && b.{f}) {{ nlohmann::json x = nlohmann::json::object(); diff_json(x, *a.{f}, *b.{f}); if (!x.empty()) j["{f}"] = std::move(x); }}')
            out.append(f'  else if (b.{f}) to_json(j["{f}"], *b.{f}); else if (a.{f}) j["{f}"] = nullptr;')
        elif ty in E:
            out.append(f'  if (a.{f} != b.{f}) j["{f}"] = static_cast<int>(b.{f});')
        else:
            out.append(f'  if (a.{f} != b.{f}) j["{f}"] = b.{f};')
    out.append('}')
out.append('}')
print('\n'.join(out))

