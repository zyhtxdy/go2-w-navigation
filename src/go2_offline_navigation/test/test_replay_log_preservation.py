"""Filesystem-only checks; importing this helper does not initialize ROS."""
import os
from pathlib import Path
import stat
import tarfile

import pytest

from go2_offline_navigation.replay_probe import preserve_fastlio_logs


LOG_NAMES = ('pos_log.txt', 'mat_pre.txt', 'mat_out.txt', 'imu.txt', 'dbg.txt',
             'fast_lio_time_log.csv')


@pytest.mark.parametrize('name', LOG_NAMES)
def test_original_bytes_mode_and_nanosecond_mtime_are_restored(tmp_path, name):
    directory, output = tmp_path / 'Log', tmp_path / 'results'
    directory.mkdir()
    output.mkdir()
    path = directory / name
    original = b'original debug contents\x00\xff\n'
    path.write_bytes(original)
    path.chmod(0o640)
    atime_ns, mtime_ns = 1600000000000123456, 1600000001000654321
    os.utime(path, ns=(atime_ns, mtime_ns))
    before = path.stat()

    with preserve_fastlio_logs(directory, output) as state:
        assert not state['restored']
        assert state['directory'] == str(directory)
        assert set(state['protected_files']) == set(LOG_NAMES)
        path.write_bytes(b'test replay replaced/truncated original logs\n')
        path.chmod(0o600)
        os.utime(path, ns=(atime_ns + 1000000000, mtime_ns + 1000000000))

    restored = path.stat()
    assert state['restored'] is True
    assert stat.S_IMODE(restored.st_mode) == stat.S_IMODE(before.st_mode)
    assert restored.st_mtime_ns == before.st_mtime_ns
    assert restored.st_atime_ns == before.st_atime_ns
    assert path.read_bytes() == original
    with tarfile.open(output / 'fastlio_logs_before.tar.gz', 'r:gz') as archive:
        assert archive.getnames() == [name]
        assert archive.extractfile(name).read() == original


def test_exception_restores_existing_files_and_removes_new_fastlio_logs(tmp_path):
    directory, output = tmp_path / 'Log', tmp_path / 'results'
    directory.mkdir()
    output.mkdir()
    original = directory / 'pos_log.txt'
    original.write_bytes(b'before failure\n')
    original.chmod(0o644)
    os.utime(original, ns=(1600000000000000000, 1600000001000000000))
    before = original.stat()

    with pytest.raises(RuntimeError, match='intentional replay failure'):
        with preserve_fastlio_logs(directory, output) as state:
            original.write_bytes(b'damaged by a failed replay\n')
            original.chmod(0o600)
            (directory / 'imu.txt').write_bytes(b'created only by this replay\n')
            raise RuntimeError('intentional replay failure')

    assert state['restored'] is True
    assert original.read_bytes() == b'before failure\n'
    assert stat.S_IMODE(original.stat().st_mode) == stat.S_IMODE(before.st_mode)
    assert original.stat().st_mtime_ns == before.st_mtime_ns
    assert not (directory / 'imu.txt').exists()


def test_fastlio_logs_absent_before_replay_are_removed_afterwards(tmp_path):
    directory, output = tmp_path / 'Log', tmp_path / 'results'
    directory.mkdir()
    output.mkdir()
    with preserve_fastlio_logs(directory, output) as state:
        for name in LOG_NAMES:
            (directory / name).write_text('new temporary log\n')
    assert state['restored'] is True
    assert not any((directory / name).exists() for name in LOG_NAMES)
    with tarfile.open(output / 'fastlio_logs_before.tar.gz', 'r:gz') as archive:
        assert archive.getnames() == []


def test_other_files_and_subdirectories_are_not_restored_or_deleted(tmp_path):
    directory, output = tmp_path / 'Log', tmp_path / 'results'
    directory.mkdir()
    output.mkdir()
    other = directory / 'operator_notes.txt'
    other.write_text('outside FAST-LIO debug log set\n')
    subdirectory = directory / 'unrelated'
    subdirectory.mkdir()
    nested = subdirectory / 'imu.txt'
    nested.write_text('same filename in a different directory\n')
    with preserve_fastlio_logs(directory, output):
        other.write_text('intentional change to unrelated notes\n')
        (directory / 'new_unrelated.txt').write_text('keep this file\n')
        nested.write_text('intentional nested change\n')
    assert other.read_text() == 'intentional change to unrelated notes\n'
    assert (directory / 'new_unrelated.txt').read_text() == 'keep this file\n'
    assert nested.read_text() == 'intentional nested change\n'
    with tarfile.open(output / 'fastlio_logs_before.tar.gz', 'r:gz') as archive:
        assert archive.getnames() == []


def test_none_directory_reports_unavailable_without_creating_files(tmp_path):
    output = tmp_path / 'not_created'
    with preserve_fastlio_logs(None, output) as state:
        assert state == dict(directory=None, restored=False, protected_files=[])
    assert state == dict(directory=None, restored=False, protected_files=[])
    assert not output.exists()
    assert list(tmp_path.iterdir()) == []
