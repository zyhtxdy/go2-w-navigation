from pathlib import Path

import pytest

from semantic_navigator.semantic_map import (
    PoseCandidate,
    SemanticEntity,
    is_cancel_command,
    load_semantic_map,
    rank_poses,
    resolve_entity,
)


def _entity(entity_id, names, poses=None):
    return SemanticEntity(
        entity_id=entity_id,
        names=tuple(names),
        description='',
        poses=tuple(
            poses or [PoseCandidate(0.0, 0.0, 0.0, 0.0)]
        ),
    )


def test_longest_alias_wins_over_shorter_contained_alias():
    entities = [
        _entity('door', ['门']),
        _entity('meeting_room', ['会议室', '小会议室']),
    ]
    result = resolve_entity('请带我去小会议室', entities)
    assert result is not None
    assert result.entity.entity_id == 'meeting_room'
    assert result.alias == '小会议室'


def test_fuzzy_english_target_is_supported():
    entities = [_entity('meeting_room', ['meeting room'])]
    result = resolve_entity('navigate to meting room', entities, 0.60)
    assert result is not None
    assert result.entity.entity_id == 'meeting_room'


def test_pose_candidates_are_ranked_from_current_position():
    poses = [
        PoseCandidate(5.0, 0.0, 0.0, 0.0),
        PoseCandidate(1.0, 0.0, 0.0, 0.0),
    ]
    ranked = rank_poses(poses, (0.0, 0.0))
    assert ranked[0].x == 1.0


def test_cancel_commands():
    assert is_cancel_command('停止导航')
    assert is_cancel_command('cancel')
    assert not is_cancel_command('去充电站')


def test_map_loader_skips_disabled_entities(tmp_path: Path):
    map_file = tmp_path / 'semantic.yaml'
    map_file.write_text(
        """
semantic_map:
  frame_id: map
  locations:
    - id: disabled
      enabled: false
      poses: [{x: 0, y: 0}]
    - id: office
      names: [办公室]
      poses:
        - {x: 1.0, y: 2.0, yaw: 1.57}
""",
        encoding='utf-8',
    )
    semantic_map = load_semantic_map(str(map_file))
    assert semantic_map.frame_id == 'map'
    assert len(semantic_map.entities) == 1
    assert semantic_map.entities[0].entity_id == 'office'
    assert semantic_map.entities[0].poses[0].z == 0.0


def test_duplicate_ids_are_rejected(tmp_path: Path):
    map_file = tmp_path / 'bad.yaml'
    map_file.write_text(
        """
locations:
  - {id: same, poses: [{x: 0, y: 0}]}
  - {id: same, poses: [{x: 1, y: 1}]}
""",
        encoding='utf-8',
    )
    with pytest.raises(ValueError, match='duplicate'):
        load_semantic_map(str(map_file))
