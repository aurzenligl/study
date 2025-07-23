#!/usr/bin/env python

import argparse
import subprocess
from pathlib import Path


# aurzenligl@aurzbox:/mnt/stor4/kazik-backup/work$ tune2fs -l ../home-disk.img | grep Block
# Block count:              50283264
# Block size:               4096
# Blocks per group:         32768
BLOCK_SIZE = 4096


def get_inode(imgpath, address) -> str:
    lines = []
    lines.append(f'open {imgpath}')
    lines.append(f'icheck {address}')
    text = ''.join((l + '\n' for l in lines))
    proc = subprocess.Popen('debugfs', stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    output = proc.communicate(text.encode())[0].decode()
    proc.wait()
    inode = output.strip().splitlines()[-2].split('\t')[1]
    return inode


def get_fpath(imgpath, inode) -> str:
    lines = []
    lines.append(f'open {imgpath}')
    lines.append(f'ncheck {inode}')
    text = ''.join((l + '\n' for l in lines))
    proc = subprocess.Popen('debugfs', stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    output = proc.communicate(text.encode())[0].decode()
    proc.wait()
    fpath = output.strip().splitlines()[-2].split('\t')[1]
    return fpath


def parse_opts():
    parser = argparse.ArgumentParser(prog='ddrescue-mapper')
    parser.add_argument('imgfile')
    parser.add_argument('mapfile')
    return parser.parse_args()


def main():
    opts = parse_opts()
    lines = Path(opts.mapfile).read_text().splitlines()
    corrupt_offsets = [int(l.split()[0], 16) // BLOCK_SIZE for l in lines if l.endswith('-')]

    print(f'Number of corrupted files: {len(corrupt_offsets)}')
    fpaths = set()
    for offset in corrupt_offsets:
        inode = get_inode(opts.imgfile, offset)
        if inode != '<block not found>':
            fpath = get_fpath(opts.imgfile, inode)
            fpaths.add(fpath)
            print(fpath)

    print(f'')
    print(f'Unique corrupted files: {len(fpaths)}')
    for p in sorted(fpaths):
        print(p)


if __name__ == '__main__':
    main()
