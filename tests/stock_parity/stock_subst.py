#!/usr/bin/env python3
"""Perform substitution strings with STOCK launch.

usage: stock_subst.py [-c name=value ...] [-e NAME=value ...] -- STRING [STRING ...]
Prints repr(result) or the exception for each string.
"""
import os
import sys

from launch.frontend.parse_substitution import parse_substitution
from launch.launch_context import LaunchContext
from launch.utilities import perform_substitutions


def main():
    args = sys.argv[1:]
    configs = {}
    while args and args[0] in ('-c', '-e'):
        flag, kv = args[0], args[1]
        k, _, v = kv.partition('=')
        if flag == '-c':
            configs[k] = v
        else:
            os.environ[k] = v
        args = args[2:]
    if args and args[0] == '--':
        args = args[1:]
    ctx = LaunchContext()
    ctx.launch_configurations.update(configs)
    for s in args:
        try:
            subs = parse_substitution(s)
            print(repr(s), '->', repr(perform_substitutions(ctx, subs)))
        except Exception as e:  # noqa
            print(repr(s), '-> ERROR', type(e).__name__, str(e).splitlines()[0][:200])


if __name__ == '__main__':
    main()
