"""Semantic-map loading, validation, target matching and pose ranking."""

from dataclasses import dataclass
from difflib import SequenceMatcher
import math
from pathlib import Path
import unicodedata
from typing import Iterable, List, Optional, Sequence, Tuple

import yaml


@dataclass(frozen=True)
class PoseCandidate:
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class SemanticEntity:
    entity_id: str
    names: Tuple[str, ...]
    description: str
    poses: Tuple[PoseCandidate, ...]


@dataclass(frozen=True)
class MatchResult:
    entity: SemanticEntity
    alias: str
    score: float


@dataclass(frozen=True)
class SemanticMap:
    frame_id: str
    entities: Tuple[SemanticEntity, ...]


_INTENT_PREFIXES = (
    '请带我去',
    '带我前往',
    '导航到',
    '移动到',
    '前往',
    '带我去',
    '请去',
    '到',
    '去',
    'pleasegotothe',
    'navigateto',
    'takemeto',
    'goto',
)


def normalize_text(text: str) -> str:
    """Normalize Chinese/Latin commands while preserving semantic characters."""
    folded = unicodedata.normalize('NFKC', text).casefold()
    return ''.join(
        character
        for character in folded
        if not character.isspace()
        and not unicodedata.category(character).startswith(('P', 'S'))
    )


def target_text(command: str) -> str:
    normalized = normalize_text(command)
    for prefix in sorted(_INTENT_PREFIXES, key=len, reverse=True):
        if normalized.startswith(prefix) and len(normalized) > len(prefix):
            return normalized[len(prefix):]
    return normalized


def _as_float(mapping, key: str, context: str, default=None) -> float:
    if key not in mapping:
        if default is not None:
            return float(default)
        raise ValueError(f'{context} is missing required field {key!r}')
    try:
        value = float(mapping[key])
    except (TypeError, ValueError) as error:
        raise ValueError(f'{context}.{key} must be numeric') from error
    if not math.isfinite(value):
        raise ValueError(f'{context}.{key} must be finite')
    return value


def load_semantic_map(path: str) -> SemanticMap:
    map_path = Path(path).expanduser()
    if not map_path.is_file():
        raise ValueError(f'semantic map file does not exist: {map_path}')
    with map_path.open('r', encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}

    root = document.get('semantic_map', document)
    if not isinstance(root, dict):
        raise ValueError('semantic_map must be a mapping')
    frame_id = str(root.get('frame_id', 'map')).strip()
    if not frame_id:
        raise ValueError('semantic_map.frame_id cannot be empty')
    raw_locations = root.get('locations', [])
    if not isinstance(raw_locations, list):
        raise ValueError('semantic_map.locations must be a list')

    entities: List[SemanticEntity] = []
    seen_ids = set()
    for index, raw_entity in enumerate(raw_locations):
        context = f'semantic_map.locations[{index}]'
        if not isinstance(raw_entity, dict):
            raise ValueError(f'{context} must be a mapping')
        if not bool(raw_entity.get('enabled', True)):
            continue
        entity_id = str(raw_entity.get('id', '')).strip()
        if not entity_id:
            raise ValueError(f'{context}.id cannot be empty')
        if entity_id in seen_ids:
            raise ValueError(f'duplicate semantic entity id: {entity_id}')
        seen_ids.add(entity_id)

        raw_names = raw_entity.get('names', [])
        if not isinstance(raw_names, list):
            raise ValueError(f'{context}.names must be a list')
        names = tuple(
            dict.fromkeys(
                name
                for name in (
                    str(value).strip()
                    for value in [entity_id, *raw_names]
                )
                if name
            )
        )
        if not any(normalize_text(name) for name in names):
            raise ValueError(f'{context} has no usable names')

        raw_poses = raw_entity.get('poses', [])
        if not isinstance(raw_poses, list) or not raw_poses:
            raise ValueError(f'{context}.poses must contain at least one pose')
        poses = []
        for pose_index, raw_pose in enumerate(raw_poses):
            pose_context = f'{context}.poses[{pose_index}]'
            if not isinstance(raw_pose, dict):
                raise ValueError(f'{pose_context} must be a mapping')
            poses.append(
                PoseCandidate(
                    x=_as_float(raw_pose, 'x', pose_context),
                    y=_as_float(raw_pose, 'y', pose_context),
                    z=_as_float(raw_pose, 'z', pose_context, 0.0),
                    yaw=_as_float(raw_pose, 'yaw', pose_context, 0.0),
                )
            )
        entities.append(
            SemanticEntity(
                entity_id=entity_id,
                names=names,
                description=str(raw_entity.get('description', '')).strip(),
                poses=tuple(poses),
            )
        )
    return SemanticMap(frame_id=frame_id, entities=tuple(entities))


def resolve_entity(
    command: str,
    entities: Iterable[SemanticEntity],
    fuzzy_threshold: float = 0.62,
) -> Optional[MatchResult]:
    """Resolve an instruction, preferring exact and longest substring aliases."""
    normalized_command = normalize_text(command)
    normalized_target = target_text(command)
    if not normalized_target:
        return None

    exact = []
    contained = []
    fuzzy = []
    for entity in entities:
        for alias in entity.names:
            normalized_alias = normalize_text(alias)
            if not normalized_alias:
                continue
            if normalized_target == normalized_alias:
                exact.append((len(normalized_alias), entity, alias))
            elif normalized_alias in normalized_command:
                contained.append((len(normalized_alias), entity, alias))
            else:
                score = SequenceMatcher(
                    None, normalized_target, normalized_alias
                ).ratio()
                if score >= fuzzy_threshold:
                    fuzzy.append((score, len(normalized_alias), entity, alias))

    if exact:
        _, entity, alias = max(exact, key=lambda item: item[0])
        return MatchResult(entity=entity, alias=alias, score=1.0)
    if contained:
        length, entity, alias = max(contained, key=lambda item: item[0])
        coverage = length / max(1, len(normalized_command))
        return MatchResult(
            entity=entity,
            alias=alias,
            score=0.90 + min(0.09, 0.09 * coverage),
        )
    if fuzzy:
        score, _, entity, alias = max(
            fuzzy, key=lambda item: (item[0], item[1])
        )
        return MatchResult(entity=entity, alias=alias, score=score)
    return None


def rank_poses(
    poses: Sequence[PoseCandidate],
    current_xy: Optional[Tuple[float, float]],
) -> List[PoseCandidate]:
    if current_xy is None:
        return list(poses)
    current_x, current_y = current_xy
    return sorted(
        poses,
        key=lambda pose: math.hypot(
            pose.x - current_x, pose.y - current_y
        ),
    )


def is_cancel_command(command: str) -> bool:
    normalized = normalize_text(command)
    return normalized in {
        '取消',
        '取消导航',
        '停止',
        '停止导航',
        '停下',
        'cancel',
        'stop',
        'stopnavigation',
    }
