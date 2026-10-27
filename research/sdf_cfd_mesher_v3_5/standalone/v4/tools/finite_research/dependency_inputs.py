"""Declare exact dependency inputs; metadata only, no package imports or grants.

The output is a plain absolute-path/SHA map for a fresh finite request or
control manifest. The receiver must hold and authenticate the merged inputs
before importing them. This builder does not supply that runtime custody.
"""
from pathlib import Path
import argparse
import hashlib
import json


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def union_checked(*maps):
    """Preserve insertion order, refuse conflicting duplicates, check bytes."""
    merged = {}
    for values in maps:
        if not isinstance(values, dict):
            raise ValueError('DependencyMapRequired')
        for raw, digest in values.items():
            path = Path(raw)
            if (not path.is_absolute() or not isinstance(digest, str)
                    or len(digest) != 64 or any(c not in '0123456789abcdef' for c in digest)):
                raise ValueError('ExactAbsoluteDependencyIdentityRequired', raw)
            name = str(path.resolve(strict=True))
            if name in merged and merged[name] != digest:
                raise ValueError('ConflictingDependencyDigest', name)
            if sha(name) != digest:
                raise ValueError('ChangedDependencyInput', name)
            merged.setdefault(name, digest)
    return merged


def build(base_path, observed_paths, package_roots):
    """Old observations are provenance, never retrospective authentication."""
    references = []
    maps = []
    for path, field in [(base_path, None), *[(p, 'files') for p in observed_paths]]:
        path = Path(path).resolve(strict=True)
        before = sha(path)
        document = json.loads(path.read_text(encoding='utf8'))
        values = document if field is None else document[field]
        if sha(path) != before:
            raise ValueError('ChangedDependencyReference', str(path))
        maps.append(values)
        references.append(dict(path=str(path), sha256=before, field=field, entries=len(values)))
    packages = []
    for root in package_roots:
        root = Path(root).resolve(strict=True)
        if not root.is_dir():
            raise ValueError('InstalledPackageDirectoryRequired', str(root))
        files = sorted(p for p in root.rglob('*') if p.is_file())
        if not files:
            raise ValueError('EmptyDependencyPackage', str(root))
        values = {str(p.resolve(strict=True)): sha(p) for p in files}
        if sorted(p for p in root.rglob('*') if p.is_file()) != files:
            raise ValueError('ChangedPackageMembership', str(root))
        maps.append(values)
        packages.append(dict(root=str(root), files=len(files),
            logical_bytes=sum(p.stat().st_size for p in files),
            native_files=sum(p.suffix.lower() in ('.dll', '.pyd') for p in files)))
    merged = union_checked(*maps)
    provenance = dict(schema='DeclaredDependencyInputs/1', references=references,
        packages=packages, declared_files=len(merged), imported_packages=False,
        runtime_custody=False, scientific_acceptance=False,
        scope='Current exact file declarations only. Receiver must merge consistently, hold and verify before use; failed historical observations remain failed.')
    return merged, provenance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', required=True)
    parser.add_argument('--observed', action='append', default=[])
    parser.add_argument('--package-root', action='append', default=[])
    parser.add_argument('--output', required=True)
    parser.add_argument('--provenance', required=True)
    args = parser.parse_args()
    output = Path(args.output).resolve()
    provenance_path = Path(args.provenance).resolve()
    if output == provenance_path or output.exists() or provenance_path.exists():
        raise ValueError('FreshDistinctDependencyOutputsRequired')
    values, provenance = build(args.base, args.observed, args.package_root)
    with output.open('x', encoding='utf8', newline='\n') as stream:
        json.dump(values, stream, indent=2)
        stream.write('\n')
    provenance['output'] = dict(path=str(output), sha256=sha(output))
    with provenance_path.open('x', encoding='utf8', newline='\n') as stream:
        json.dump(provenance, stream, indent=2, sort_keys=True)
        stream.write('\n')
    print(json.dumps(dict(files=len(values), output_sha256=sha(output),
        provenance_sha256=sha(provenance_path), runtime_custody=False)))


if __name__ == '__main__':
    main()
