#!/usr/bin/env python3
import yaml
import os
import sys
import argparse
from typing import Tuple


def float_representer(dumper, value):
    text = '{0:.3f}'.format(value)
    return dumper.represent_scalar(u'tag:yaml.org,2002:float', text)


def generate_even_grid(size: Tuple[int, int], offset: Tuple[float, float,
                                                            float],
                       distance_between_tags: Tuple[float, float], tag_size):
    data = {}
    data['tag_poses'] = []
    tag_id = 0
    for row in range(size[0]):
        for col in range(size[1]):
            x = col * distance_between_tags[0] + offset[0]
            y = row * distance_between_tags[1] + offset[1]
            z = offset[2]
            data['tag_poses'].append({
                'frame_id': 'map',
                'id': tag_id,
                'size': tag_size,
                'x': x,
                'y': y,
                'z': z,
                'R': 0.0,
                'P': 0.0,
                'Y': 0.0,
            })
            tag_id += 1
    return data


def generate_tag_standalone(n_tags: int, size: float):
    data = {}
    data['standalone_tags'] = []
    for i in range(n_tags):
        data['standalone_tags'].append({'id': i, 'size': size})
    return data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--tag-size', default=0.04, help='Tag size in meters.')
    parser.add_argument('--grid-size',
                        nargs=2,
                        default=[23, 14],
                        type=int,
                        help='Number of rows and cols of the tag grid.')
    parser.add_argument('--offset',
                        nargs=3,
                        default=[0.2, 0.2, -1.0],
                        type=float,
                        help='Offset of the tag grid relative to the origin')
    parser.add_argument('--distance',
                        nargs=2,
                        default=[0.2, 0.2],
                        type=float,
                        help='Distance between tags in x- and y-direction')
    parser.add_argument('--out-dir',
                        required=True,
                        help='Output directory of the generated files.')
    args = parser.parse_args()

    yaml.add_representer(float, float_representer)
    data = generate_even_grid(args.grid_size, args.offset, args.distance,
                              args.tag_size)
    if args.out_dir == '-':
        yaml.dump(data, sys.stdout, default_flow_style=True, width=float('inf'))
    else:
        filename = 'tag_poses.yaml'
        filepath = os.path.join(args.out_dir, filename)
        with open(filepath, 'w') as f:
            yaml.dump(data, f)
            print(f'Created file [{filepath}]')

        filename = 'tags_standalone.yaml'
        filepath = os.path.join(args.out_dir, filename)
        with open(filepath, 'w') as f:
            n = args.grid_size[0] * args.grid_size[1]
            # correct for difference in physical tag size and tag without border
            data = generate_tag_standalone(n, args.tag_size * 8 / 10)
            yaml.dump(data, f)
            print(f'Created file [{filepath}]')


if __name__ == '__main__':
    main()
