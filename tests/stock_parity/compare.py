#!/usr/bin/env python3
"""Compare play_launch's SystemModel against the stock oracle's output.

usage: compare.py <model.yaml> <stock.json> [--anon]

Both sides are normalised to {fqn: entity} where entity carries kind, pkg,
exec/plugin, container, params (effective, typed), remaps (ordered), args,
ros_args, env (set/unset). Differences are printed; exit 1 if any.
"""

import fnmatch
import json
import re
import sys

import re
import yaml


class ModelLoader(yaml.SafeLoader):
    """The model is written by serde_yaml (YAML 1.2): `1e-6` is a float there."""


ModelLoader.add_implicit_resolver(
    'tag:yaml.org,2002:float',
    re.compile(r'^[-+]?(\.[0-9]+|[0-9]+(\.[0-9]*)?)([eE][-+]?[0-9]+)$'),
    list('-+0123456789.'))

ANON_RE = re.compile(r'_\d+_\d+$')


def runtime_typed(v):
    """What play_launch's spawner makes of a model value (node_cmdline.rs
    str_to_yaml): it stringifies, then re-types by shape."""
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        s = repr(v) if isinstance(v, float) else str(v)
    elif isinstance(v, list):
        return [runtime_typed(x) for x in v]
    elif v is None:
        return None
    else:
        s = str(v)
    # A YAML single-quoted value is a string the parser kept from re-typing.
    if len(s) >= 2 and s[0] == "'" and s[-1] == "'":
        return s[1:-1].replace("''", "'")
    if s in ('true', 'True'):
        return True
    if s in ('false', 'False'):
        return False
    try:
        return int(s)
    except ValueError:
        pass
    if ('.' in s or 'e' in s or 'E' in s):
        try:
            return float(s)
        except ValueError:
            pass
    t = s.strip()
    if t.startswith('[') and t.endswith(']'):
        inner = t[1:-1].strip()
        if not inner:
            return []
        out = []
        for item in inner.split(','):
            item = item.strip()
            if len(item) >= 2 and item[0] == item[-1] and item[0] in '\'"':
                out.append(item[1:-1])
            else:
                out.append(runtime_typed(item))
        return out
    return s


def canon(v):
    """Canonical comparable form of a parameter value."""
    if type(v).__name__ == 'array':
        v = list(v)
    if isinstance(v, bool):
        return ('bool', v)
    if isinstance(v, int):
        return ('int', v)
    if isinstance(v, float):
        return ('float', v)
    if isinstance(v, (list, tuple)):
        return ('list', tuple(canon(x) for x in v))
    if v is None:
        return ('none',)
    return ('str', str(v))


def flatten(prefix, d, out):
    for k, v in d.items():
        key = f'{prefix}.{k}' if prefix else str(k)
        if isinstance(v, dict):
            flatten(key, v, out)
        else:
            out[key] = v


def node_matches(key, name, fqn):
    key = str(key)
    if key in ('/**', '**'):
        return True
    cands = {name, fqn, fqn.lstrip('/')}
    if key in cands:
        return True
    k = key if key.startswith('/') else '/' + key
    # wildcard keys
    if '*' in k:
        pat = k.replace('/**/', '/*/')
        return fnmatch.fnmatch(fqn, pat) or fnmatch.fnmatch(fqn, k)
    return False


def params_from_yaml_text(text, name, fqn):
    out = {}
    try:
        data = yaml.safe_load(text) or {}
    except Exception as e:  # noqa
        return {'__unparseable__': str(e)}
    for key, body in data.items():
        if not isinstance(body, dict):
            continue
        if 'ros__parameters' in body and node_matches(key, name, fqn):
            flatten('', body['ros__parameters'] or {}, out)
        else:
            # namespace nesting: {ns: {name: {ros__parameters}}}
            for k2, b2 in body.items():
                if isinstance(b2, dict) and 'ros__parameters' in b2:
                    if node_matches(f'{key}/{k2}', name, fqn):
                        flatten('', b2['ros__parameters'] or {}, out)
    return out


def parse_param_value(s):
    try:
        return yaml.safe_load(s)
    except Exception:
        return s


# ---------------------------------------------------------------- stock side

def stock_entities(path):
    d = json.load(open(path))
    ents = {}
    for p in d['processes']:
        cmd = p['cmd']
        files = {f['path']: f['content'] for f in p['param_files']}
        if p['kind'] == 'process':
            name = None
            fqn = None
        else:
            fqn = p['fqn']
        kind = p['kind']
        e = {'kind': {'node': 'node', 'lifecycle_node': 'node', 'container': 'container',
                      'process': 'exec'}[kind]}
        if kind == 'process':
            e['cmd'] = cmd
            # name for an executable: stock names it by its `name` attr; we key
            # by the joined command since there is no fqn
            key = 'exec:' + ' '.join(cmd)
        else:
            e['pkg'] = p['package']
            e['exec'] = p['executable']
            if '--ros-args' in cmd:
                i = cmd.index('--ros-args')
                pre, ros = cmd[1:i], cmd[i + 1:]
            else:
                pre, ros = cmd[1:], []
            e['args'] = pre
            remaps, ros_args, params = [], [], {}
            node_name = fqn.rsplit('/', 1)[-1]
            j = 0
            ns = ''
            while j < len(ros):
                t = ros[j]
                if t in ('-r', '--remap') and j + 1 < len(ros):
                    a, _, b = ros[j + 1].partition(':=')
                    if a == '__node':
                        node_name = b
                    elif a == '__ns':
                        ns = b
                    else:
                        remaps.append((a, b))
                    j += 2
                elif t == '--params-file' and j + 1 < len(ros):
                    params.update(params_from_yaml_text(files.get(ros[j + 1]) or '', node_name, fqn))
                    j += 2
                elif t in ('-p', '--param') and j + 1 < len(ros):
                    a, _, b = ros[j + 1].partition(':=')
                    params[a] = parse_param_value(b)
                    j += 2
                elif t in ('--ros-args', '--'):
                    j += 1
                else:
                    ros_args.append(t)
                    j += 1
            e['remaps'] = remaps
            e['ros_args'] = ros_args
            e['params'] = {k: canon(v) for k, v in params.items()}
            key = fqn.replace('<node_namespace_unspecified>', '')
            if not key.startswith('/'):
                key = '/' + key
        e['env'] = dict(p['env_diff'])
        e['env_unset'] = sorted(p['env_unset'])
        ents.setdefault(key, []).append(e)
    for l in d['loads']:
        ns = l['namespace'] or ''
        if ns in ('', '/'):
            fqn = '/' + l['name']
        else:
            fqn = ns.rstrip('/') + '/' + l['name']
        e = {
            'kind': 'composable',
            'pkg': l['package'],
            'plugin': l['plugin'],
            'container': l['target'] if l['target'].startswith('/') else '/' + l['target'],
            'remaps': [tuple(r.split(':=', 1)) for r in l['remaps']],
            'params': {k: canon(v) for k, v in l['params']},
            'extra_args': {k: canon(v) for k, v in l['extra_args']},
        }
        ents.setdefault(fqn, []).append(e)
    return ents


# ---------------------------------------------------------------- model side

def model_entities(path):
    m = yaml.load(open(path), Loader=ModelLoader)
    nodes = (m.get('structure') or {}).get('nodes') or {}
    ents = {}
    for fqn, n in nodes.items():
        if n.get('is_container'):
            kind = 'container'
        elif n.get('plugin'):
            kind = 'composable'
        elif n.get('raw_cmd') is not None and not n.get('pkg'):
            kind = 'exec'
        else:
            kind = 'node'
        e = {'kind': kind}
        params = {}
        for f in n.get('params_files') or []:
            params.update(params_from_yaml_text(f, n.get('node_name') or fqn.rsplit('/', 1)[-1], fqn))
        for k, v in (n.get('params') or {}).items():
            if v is None:
                continue
            # The model types scalars itself; a Str stays a string at spawn
            # unless it is a list's flow text (the model has no array type).
            if isinstance(v, str) and not (v.startswith('[') and v.endswith(']')):
                params[k] = v
            else:
                params[k] = runtime_typed(v)
        if kind == 'exec':
            e['cmd'] = n.get('raw_cmd')
            key = 'exec:' + ' '.join(n.get('raw_cmd') or [])
        else:
            key = fqn
            e['pkg'] = n.get('pkg')
            if kind == 'composable':
                e['plugin'] = n.get('plugin')
                c = n.get('container') or ''
                e['container'] = c if c.startswith('/') else '/' + c
                e['extra_args'] = {k: canon(runtime_typed(v)) for k, v in (n.get('extra_args') or {}).items()}
            else:
                e['exec'] = n.get('exec')
                e['args'] = n.get('args') or []
                e['ros_args'] = n.get('ros_args') or []
            e['remaps'] = [(r['from'], r['to']) for r in (n.get('remaps') or [])]
            e['params'] = {k: canon(v) for k, v in params.items()}
        env = {}
        for kv in n.get('env') or []:
            if BASE_ENV.get(kv['name']) == kv['value']:
                env.pop(kv['name'], None)
                continue
            env[kv['name']] = kv['value']
        if kind != 'composable':
            e['env'] = env
            e['env_unset'] = sorted(n.get('env_unset') or [])
        e['_raw'] = n
        ents.setdefault(key, []).append(e)
    return ents


BASE_ENV = {}


def anon_key(k, e=None):
    k = ANON_RE.sub('_ANON', k)
    if e is not None and e[0].get('exec') and (
            k.endswith('/<node_name_unspecified>') or re.search(r'/[^/]+-\d+$', k)):
        k = k.rsplit('/', 1)[0] + '/<unnamed:' + str(e[0]['exec']) + '>'
    return k


def main():
    model, stock = sys.argv[1], sys.argv[2]
    global BASE_ENV
    BASE_ENV = json.load(open(stock)).get('base_env') or {}
    s = stock_entities(stock)
    m = model_entities(model)
    s2, m2 = {}, {}
    for k, v in s.items():
        for e in v:
            s2.setdefault(anon_key(k, [e]), []).append(e)
    for k, v in m.items():
        for e in v:
            m2.setdefault(anon_key(k, [e]), []).append(e)
    s, m = s2, m2
    diffs = 0
    for k in sorted(set(s) | set(m)):
        if k not in m:
            print(f'MISSING in play_launch: {k}  {s[k][0]["kind"]}')
            diffs += 1
            continue
        if k not in s:
            print(f'EXTRA in play_launch:   {k}  {m[k][0]["kind"]}')
            diffs += 1
            continue
        if len(s[k]) != len(m[k]):
            print(f'COUNT {k}: stock {len(s[k])} play_launch {len(m[k])}')
            diffs += 1
        se, me = s[k][0], m[k][0]
        for f in sorted(set(se) | set(me)):
            if f.startswith('_'):
                continue
            sv, mv = se.get(f), me.get(f)
            if f == 'remaps':
                sv = [tuple(x) for x in (sv or [])]
                mv = [tuple(x) for x in (mv or [])]
            if f == 'args' and me['kind'] == 'node':
                # the model keeps args as one list; stock splits the same way
                pass
            if sv != mv:
                if f == 'params' and isinstance(sv, dict) and isinstance(mv, dict):
                    for pk in sorted(set(sv) | set(mv)):
                        if sv.get(pk) != mv.get(pk):
                            print(f'DIFF {k} param {pk}: stock={sv.get(pk)} play_launch={mv.get(pk)}')
                            diffs += 1
                    continue
                print(f'DIFF {k} {f}: stock={sv!r} play_launch={mv!r}')
                diffs += 1
    print(f'-- {diffs} difference(s); stock {sum(map(len, s.values()))} entities, '
          f'play_launch {sum(map(len, m.values()))}')
    sys.exit(1 if diffs else 0)


if __name__ == '__main__':
    main()
