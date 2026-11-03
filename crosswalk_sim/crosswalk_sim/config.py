"""Scenario configuration: default.yaml <- scenario file <- command line overrides."""

import copy
import os
import shlex

import yaml


def _deep_merge(base, extra):
    out = copy.deepcopy(base)
    for k, v in (extra or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def resolve_scenario_path(scenario, scenario_dir):
    """`scenario` is a name from config/scenarios (e.g. 'hard') or a path to a YAML file."""
    if not scenario:
        return None
    if os.path.isfile(scenario):
        return scenario
    candidate = os.path.join(scenario_dir, scenario if scenario.endswith('.yaml') else scenario + '.yaml')
    if os.path.isfile(candidate):
        return candidate
    available = sorted(f[:-5] for f in os.listdir(scenario_dir) if f.endswith('.yaml'))
    raise FileNotFoundError(f"Scenario '{scenario}' not found. Available: {available} or a path to a YAML file.")


def apply_overrides(cfg, overrides):
    """overrides: 'a.b=1 c=[1,2] d.e=true' (space or ';' separated, YAML values)."""
    if not overrides:
        return cfg
    cfg = copy.deepcopy(cfg)
    for item in shlex.split(overrides.replace(';', ' ')):
        if '=' not in item:
            raise ValueError(f"Bad override '{item}', expected key.sub=value")
        key, value = item.split('=', 1)
        node = cfg
        parts = key.strip().split('.')
        for p in parts[:-1]:
            node = node.setdefault(p, {})
            if not isinstance(node, dict):
                raise ValueError(f"Override '{key}': '{p}' is not a section")
        node[parts[-1]] = yaml.safe_load(value)
    return cfg


def load_config(scenario_dir, scenario='default', overrides='', seed=None):
    with open(os.path.join(scenario_dir, 'default.yaml')) as f:
        cfg = yaml.safe_load(f) or {}
    path = resolve_scenario_path(scenario, scenario_dir)
    if path and os.path.basename(path) != 'default.yaml':
        with open(path) as f:
            cfg = _deep_merge(cfg, yaml.safe_load(f) or {})
    cfg = apply_overrides(cfg, overrides)
    if seed not in (None, ''):
        cfg['seed'] = int(seed)
    return cfg
