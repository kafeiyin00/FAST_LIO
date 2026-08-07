#!/usr/bin/env python3

import argparse
import json
import os
from pathlib import Path
import shutil
import tempfile


def timestamp_key(path):
    try:
        return tuple(int(value) for value in path.stem.split('_'))
    except ValueError as exc:
        raise ValueError('Unexpected timestamp filename: ' + path.name) from exc


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-root', required=True, type=Path,
                        help='Directory containing route_1 ... route_5')
    parser.add_argument('--submap-dir', required=True, type=Path)
    parser.add_argument('--odom-dir', required=True, type=Path)
    parser.add_argument('--transforms', type=Path,
                        help='Existing all_transforms.txt to annotate')
    parser.add_argument('--annotate-only', action='store_true',
                        help='Reuse route_ranges.json and only annotate transforms')
    return parser.parse_args()


def copy_outputs(input_root, submap_dir, odom_dir):
    submap_dir.mkdir(parents=True, exist_ok=True)
    odom_dir.mkdir(parents=True, exist_ok=True)
    if any(submap_dir.glob('*.pcd')) or any(odom_dir.glob('*.odom')):
        raise RuntimeError('Aggregate output directories must not contain PCD/odom files')

    route_ranges = []
    next_index = 0
    for route_id in range(1, 6):
        frames_dir = input_root / ('route_%d' % route_id) / 'frames'
        pcd_files = sorted(frames_dir.glob('*.pcd'), key=timestamp_key)
        if not pcd_files:
            raise RuntimeError('No submaps found in ' + str(frames_dir))

        route_start = next_index
        for pcd_path in pcd_files:
            global_odom = frames_dir / ('global_' + pcd_path.stem + '.odom')
            if not global_odom.is_file():
                raise RuntimeError('Missing matching global odom: ' + str(global_odom))
            shutil.copy2(str(pcd_path), str(submap_dir / ('%d.pcd' % next_index)))
            shutil.copy2(str(global_odom), str(odom_dir / ('%d.odom' % next_index)))
            next_index += 1

        route_ranges.append({
            'route': route_id,
            'start_index': route_start,
            'end_index': next_index - 1,
            'submap_count': next_index - route_start,
        })

    metadata = {
        'total_submaps': next_index,
        'routes': route_ranges,
    }
    metadata_path = submap_dir / 'route_ranges.json'
    with open(metadata_path, 'w') as handle:
        json.dump(metadata, handle, indent=2)
        handle.write('\n')
    return metadata


def annotate_transforms(transforms_path, metadata):
    if not transforms_path.is_file():
        raise RuntimeError('Transform file does not exist: ' + str(transforms_path))
    original = transforms_path.read_text()
    marker = '# Submap index ranges by route (ascending route order):\n'
    if marker in original:
        raise RuntimeError('Transform file is already annotated: ' + str(transforms_path))

    lines = [marker]
    for route in metadata['routes']:
        lines.append(
            '# Route {route}: {start_index}-{end_index} ({submap_count} submaps)\n'.format(
                **route
            )
        )
    annotation = ''.join(lines)

    separator = '########################################\n'
    insertion = original.find(separator)
    if insertion == -1:
        raise RuntimeError('Could not find transform header separator')
    updated = original[:insertion] + annotation + original[insertion:]

    descriptor, temporary_name = tempfile.mkstemp(
        prefix=transforms_path.name + '.', dir=str(transforms_path.parent)
    )
    try:
        with os.fdopen(descriptor, 'w') as handle:
            handle.write(updated)
        os.replace(temporary_name, str(transforms_path))
    finally:
        if os.path.exists(temporary_name):
            os.unlink(temporary_name)


def main():
    args = parse_args()
    if args.annotate_only:
        if args.transforms is None:
            raise RuntimeError('--annotate-only requires --transforms')
        metadata_path = args.submap_dir / 'route_ranges.json'
        with open(metadata_path) as handle:
            metadata = json.load(handle)
    else:
        metadata = copy_outputs(args.input_root, args.submap_dir, args.odom_dir)
    if args.transforms is not None:
        annotate_transforms(args.transforms, metadata)
    print(json.dumps(metadata, indent=2))


if __name__ == '__main__':
    main()
