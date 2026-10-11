"""Regression for the actual retained UTF-8 evidence; no geometry execution."""
import tempfile
import unittest
from pathlib import Path
from run_live_sf2_surface import read,sha,V4

class EncodingTests(unittest.TestCase):
    def test_retained_manifest_bytes_unchanged(self):
        path=V4/'results/sf2_coupled_policy/EVIDENCE.json'
        before=path.read_bytes()
        self.assertEqual(sha(path),'9fcf6201afb4639176bc4da216ba7f3f0033f5ad24fc34f3b2a419e588418e41')
        with self.assertRaises(UnicodeDecodeError):before.decode('cp1252')
        result=read(path)
        self.assertIn('?? \uf05c',result['preservation']['initial_status'])
        self.assertEqual(path.read_bytes(),before)
    def test_invalid_utf8_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'invalid.json';path.write_bytes(b'{"x":"\xff"}')
            with self.assertRaises(UnicodeDecodeError):read(path)

if __name__=='__main__':unittest.main(verbosity=2)
