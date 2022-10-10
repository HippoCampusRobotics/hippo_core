#!/usr/bin/env python3
import xacro
import sys
import argparse


def parse_mappings(args):
    mapping = {}
    if args:
        for arg in args:
            s = arg.split('=')
            if len(s) > 1:
                mapping[s[0]] = s[1]
    return mapping


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--mappings',
        nargs='+',
        help='space separated list of mappings in the form of arg_name=value')
    parser.add_argument('--input', required=True)
    args = parser.parse_args()
    mappings = parse_mappings(args.mappings)
    doc = xacro.process_file(args.input, mappings=mappings)
    sys.stdout.write(doc.toxml())
    return 0


if __name__ == "__main__":
    main()
