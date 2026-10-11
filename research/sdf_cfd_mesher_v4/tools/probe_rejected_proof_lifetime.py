"""Measure retirement of an actual rejected candidate, without meshing changes."""
import json
import shutil
import sys
from pathlib import Path

import psutil
from mixed_front_fixture_provider import release_rejected_proofs
from run_geometry_family import OUT, g
from sdf_cfd_mesher_v4.common.authority import file_hash, identity


def run(source):
    source = Path(source)
    folder = OUT / 'proof_lifetime_probe'
    folder.mkdir(exist_ok=True)
    preserved = folder / 'rejected_proposal.json'
    if not preserved.exists():
        shutil.copyfile(source, preserved)
    digest = file_hash(preserved)
    with preserved.open() as stream:
        proposal = json.load(stream)
    certificate = proposal['check']['certificate']
    invariant = {k: v for k, v in certificate.items() if k != 'proofs'}
    invariant_hash = identity(invariant)
    process = psutil.Process()
    before = process.memory_info().rss
    record = release_rejected_proofs(proposal)
    after = process.memory_info().rss
    assert record is not None and identity(certificate) == invariant_hash
    record.update(source_sha256=digest, source_bytes=preserved.stat().st_size,
                  preserved_source=str(preserved.relative_to(OUT)),
                  rss_before=before, rss_after=after, rss_released=before-after,
                  peak_rss_bytes=process.memory_info().peak_wset,
                  failed_witnesses_and_summary_exact=True,
                  scope='Isolated real rejected-candidate payload. Not a full-run peak reduction measurement.')
    g.save(OUT / 'PROOF_LIFETIME_PROBE.json', record)
    print(json.dumps(record), flush=True)


if __name__ == '__main__':
    run(sys.argv[1])
