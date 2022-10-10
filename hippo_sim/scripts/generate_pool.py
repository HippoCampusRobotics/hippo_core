#!/usr/bin/env python3
import xacro
import xml.etree.ElementTree as ET
import sys
import os
import yaml
import argparse
from ament_index_python.packages import get_package_share_path
import subprocess
import shlex
import hashlib
import glob


def create_base_model():
    sdf = ET.Element('sdf', {'version': '1.6'})
    model = ET.SubElement(sdf, 'model', {'name': 'apriltag_grid'})
    static = ET.SubElement(model, 'static')
    static.text = 'true'
    pose = ET.SubElement(model, 'pose')
    pose.text = '0 0 0 0 0 0'
    return sdf, model


def offset_pose(x, y, z, R, P, Y, model):
    pose = model.find('pose')
    if not pose:
        pose = ET.SubElement(model, 'pose')
        pose.text = f'{x} {y} {z} {R} {P} {Y}'
    else:
        tmp = pose.text.split(' ')
        pose.text = f'{x+tmp[0]} {y+tmp[1]} {z+tmp[2]} {R+tmp[3]} {P+tmp[4]} {Y+tmp[5]}'
    return model


def clean_model(model):
    link = model.find('link')
    if link:
        inertial = link.find('inertial')
        if inertial:
            link.remove(inertial)
    return model


def generate_hashes(paths):
    hashes = []
    n = len(paths)
    for i, path in enumerate(paths):
        with open(path, 'rb') as f:
            file_hash = hashlib.blake2b()
            while chunk := f.read(8192):
                file_hash.update(chunk)
            hashes.append(file_hash.digest())
    return hashes


def generate_urdfs(xacro_path, tag_poses, output_dir):
    paths = []
    for tag_data in tag_poses['tag_poses']:
        name = f'apriltag_{tag_data["id"]}'
        mappings = {
            'size_x': str(tag_data['size']),
            'size_y': str(tag_data['size']),
            'tag_id': str(tag_data['id'])
        }
        doc = xacro.process_file(xacro_path, mappings=mappings)
        urdf = doc.toxml()
        tag_sdf = ET.fromstring(urdf)
        tag_sdf.set('name', name)
        xml_string = ET.tostring(tag_sdf).decode()
        filepath = os.path.join(output_dir, name)
        with open(filepath, 'w') as f:
            f.write(xml_string)
        paths.append(filepath)
    paths = sorted(paths)
    hashes = generate_hashes(paths)
    return [{'path': p, 'hash': h} for p, h in zip(paths, hashes)]


def urdf_changed(old_urdf, new_urdf):
    if len(old_urdf) != len(new_urdf):
        return True
    for i, urdf in enumerate(new_urdf):
        if old_urdf[i]['hash'] != new_urdf[i]['hash']:
            return True
    return False


def get_old_urdf(path):
    files = glob.glob(path)
    old_urdf_files = sorted(files)
    old_hashes = generate_hashes(old_urdf_files)
    return [{'path': p, 'hash': h} for p, h in zip(old_urdf_files, old_hashes)]


def need_to_rebuild(old_urdf, new_urdf, cache_sdf):
    if urdf_changed(old_urdf, new_urdf):
        return True
    if not os.path.exists(cache_sdf):
        return True
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--tag-poses', required=True)
    parser.add_argument('--input-file', required=False)
    parser.add_argument('--force', action='store_true')
    args = parser.parse_args()

    cache_dir = os.path.join('/tmp', 'apriltag_generation')

    cache_sdf = os.path.join(cache_dir, 'apriltag.sdf')

    if args.input_file:
        xacro_path = args.input_file
    else:
        package_path = get_package_share_path('hippo_sim')
        xacro_path = package_path / 'models/apriltag/urdf/apriltag.xacro'

    tag_poses = yaml.safe_load(args.tag_poses)

    if not os.path.exists(cache_dir):
        os.mkdir(cache_dir)

    p = f'{cache_dir}/apriltag_*'
    files = glob.glob(p)
    old_urdf = get_old_urdf(p)
    if files:
        subprocess.Popen(['rm'] + files).wait()

    base_sdf, base_model = create_base_model()
    urdf = generate_urdfs(xacro_path=xacro_path,
                          tag_poses=tag_poses,
                          output_dir=cache_dir)
    if not args.force:
        if not need_to_rebuild(old_urdf, urdf, cache_sdf):
            with open(cache_sdf, 'r') as f:
                sys.stdout.write(f.read())
            return
    cmds = [shlex.split(f'ign sdf -p {x["path"]}') for x in urdf]
    p = [
        subprocess.Popen(x, stdout=subprocess.PIPE, universal_newlines=True)
        for x in cmds
    ]
    tag_models = [
        clean_model(ET.fromstring(x.communicate()[0]).find('model')) for x in p
    ]
    for i, tag_model in enumerate(tag_models):
        d = tag_poses['tag_poses'][i]
        offset_pose(d['x'], d['y'], d['z'], d['R'], d['P'], d['Y'], tag_model)
        base_model.append(tag_model)
    with open(cache_sdf, 'w') as f:
        f.write(ET.tostring(base_sdf).decode())
    sys.stdout.write(ET.tostring(base_sdf).decode())


if __name__ == '__main__':
    main()
