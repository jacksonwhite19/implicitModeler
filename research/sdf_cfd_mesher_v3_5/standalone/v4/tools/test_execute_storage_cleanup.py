"""Only disposable test files. Never operates on a scientific target."""
import ctypes
from pathlib import Path
import tempfile
import unittest
from execute_storage_cleanup import (checked_paths, deletion_handle, delete_held,
                                     directory_guard, same_metadata)
from path_preserving_deduplication import metadata, file_hash, handle_hash
from immutable_state_access import guard


class CleanupChecks(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name).resolve()
        self.target = self.root/'copy.bin'
        self.canonical = self.root/'original.bin'
        self.target.write_bytes(b'bounded disposable duplicate test\x00\xff')
        self.canonical.write_bytes(self.target.read_bytes())
    def tearDown(self):
        self.tmp.cleanup()
    def test_exact_handle_retirement_preserves_canonical(self):
        expected = metadata(self.target)
        sha = file_hash(self.canonical)
        with directory_guard(self.root), guard(self.canonical), deletion_handle(self.target) as h:
            same_metadata(metadata(self.target), expected)
            self.assertEqual(handle_hash(h), sha)
            delete_held(h)
        self.assertFalse(self.target.exists())
        self.assertEqual(file_hash(self.canonical), sha)
    def test_writer_and_replacement_denied(self):
        with deletion_handle(self.target):
            with self.assertRaises(OSError):
                self.target.write_bytes(b'altered')
            with self.assertRaises(OSError):
                self.target.rename(self.root/'other.bin')
        self.assertTrue(self.target.exists())
    def test_guarded_survivor_cannot_be_deleted(self):
        with guard(self.canonical):
            with self.assertRaises(OSError):
                with deletion_handle(self.canonical):
                    self.fail('GuardBypassed')
    def test_changed_metadata_rejected(self):
        old = metadata(self.target)
        self.target.write_bytes(b'altered')
        with self.assertRaises(ValueError):
            same_metadata(metadata(self.target), old)
    def test_shared_target_rejected(self):
        import os
        os.link(self.target, self.root/'alias.bin')
        actual = metadata(self.target)
        with self.assertRaises(ValueError):
            same_metadata(actual, actual)
    def test_outside_and_root_rejected(self):
        for path in (self.root, self.root.parent):
            with self.assertRaises(ValueError):
                checked_paths(path, self.root)
        self.assertEqual(checked_paths(self.target, self.root), self.target)
    def test_directory_guard_blocks_rename(self):
        child = self.root/'folder'; child.mkdir()
        with directory_guard(child):
            with self.assertRaises(OSError):
                child.rename(self.root/'renamed')


if __name__ == '__main__':
    unittest.main()
