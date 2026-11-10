"""Build fixed terrain from the exact PCD used by the keyframe database."""
import argparse
from collections import Counter
import csv
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import yaml

from .map_io import database_signature, read_points, sha256


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('database', help='Exported go2_relocalization database with map.pcd')
    parser.add_argument('output', help='New navigation-map directory; never overwrites')
    parser.add_argument('--recover-sparse-ground-layers', action='store_true',
                        help='Recover a majority observed ground layer after a failed sparse-cell fit')
    parser.add_argument('--terrain-params', default=str(Path(
        get_package_share_directory('terrain_costmap')) / 'config/terrain_costmap.yaml'))
    args = parser.parse_args()
    database, output = Path(args.database).resolve(), Path(args.output).resolve()
    if output.exists():
        parser.error('Output already exists; choose a new directory')
    before = database_signature(database)
    params = yaml.safe_load(Path(args.terrain_params).read_text())['terrain_costmap']['ros__parameters']
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix='.' + output.name + '_', dir=output.parent))
    try:
        binary = Path(get_package_prefix('go2_offline_navigation')) / 'lib/go2_offline_navigation/terrain_map_builder'
        subprocess.run([str(binary), '--ros-args', '--params-file', args.terrain_params,
                        '-p', 'input_map:=' + str(database / 'map.pcd'),
                        '-p', 'recover_sparse_ground_layers:=' +
                        str(args.recover_sparse_ground_layers).lower(),
                        '-p', 'output_directory:=' + str(stage)], check=True)
        after = database_signature(database)
        if before != after:
            raise ValueError('Localization database changed while building the map')
        files = {}
        for name, allow_empty in (('support.csv', False), ('obstacles.csv', True)):
            files[name] = dict(sha256=sha256(stage / name),
                               points=len(read_points(stage / name, allow_empty)))
        with (stage / 'hazard_sources.csv').open() as stream:
            source_counts = Counter(row['source'] for row in csv.DictReader(stream))
        if set(source_counts) - {'measured', 'synthetic'} or \
                sum(source_counts.values()) != files['obstacles.csv']['points']:
            raise ValueError('Hazard source diagnostics do not match exported obstacles')
        diagnostic_files = {name: dict(sha256=sha256(stage / name))
                            for name in ('hazard_sources.csv', 'terrain_tiles.csv', 'terrain_cells.csv')}
        metadata = dict(format_version=1, frame_id='map', units='metres',
                        database_signature=before, files=files,
                        diagnostic_files=diagnostic_files,
                        hazard_sources={name: source_counts[name] for name in ('measured', 'synthetic')},
                        tile_diagnostics_scope='Cell counts include overlapping fitting halos; '
                                               'owned export counts are disjoint',
                        terrain_parameters=params,
                        synthetic_obstacle_clearance_radius_used=0.0,
                        sparse_ground_layer_recovery_used=args.recover_sparse_ground_layers,
                        height_reference='Unmodified database map coordinates',
                        coverage='Observed supported cells only; gaps are not filled')
        (stage / 'navigation_map.json').write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2) + '\n')
        # No other writer may replace this directory during normal single-user export.
        if output.exists():
            raise FileExistsError(output)
        os.rename(stage, output)
        print('Saved navigation map:', output)
        print('Support points:', files['support.csv']['points'],
              'Static hazard points:', files['obstacles.csv']['points'])
        print('Hazard sources:', metadata['hazard_sources'])
    finally:
        shutil.rmtree(stage, ignore_errors=True)
