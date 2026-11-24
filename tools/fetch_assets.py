#!/usr/bin/env python3
"""Fetch only required meshes from pinned upstream archives and verify SHA-256."""
import hashlib
import io
import json
from pathlib import Path
import tarfile
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]


def matches(path, digest):
    return path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest() == digest


def main():
    manifest = json.loads((ROOT/'assets/manifest.json').read_text())
    groups = {}
    for row in manifest['files']:
        if not matches(ROOT/row['destination'], row['sha256']):
            groups.setdefault((row['repository'], row['commit']), []).append(row)
    for (repository, commit), rows in groups.items():
        owner_repo = repository.removeprefix('https://github.com/')
        url = f'https://codeload.github.com/{owner_repo}/tar.gz/{commit}'
        print(f'Downloading {repository} at {commit}', flush=True)
        with urlopen(url, timeout=120) as response:
            archive_bytes = response.read()
        with tarfile.open(fileobj=io.BytesIO(archive_bytes), mode='r:gz') as archive:
            for row in rows:
                member = next(m for m in archive.getmembers() if '/'.join(m.name.split('/')[1:]) == row['upstream_path'])
                payload = archive.extractfile(member).read()
                if hashlib.sha256(payload).hexdigest() != row['sha256']:
                    raise RuntimeError(f'Hash mismatch: {row["upstream_path"]}')
                destination = ROOT/row['destination']
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_bytes(payload)
    for row in manifest['files']:
        if not matches(ROOT/row['destination'], row['sha256']):
            raise RuntimeError(f'Missing or modified asset: {row["destination"]}')
    print(f'Verified {len(manifest["files"])} upstream meshes.')


if __name__ == '__main__':
    main()
