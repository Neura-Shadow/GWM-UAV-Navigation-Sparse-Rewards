"""P3 immutable evidence helpers: hashing, strict JSON and exclusive publication.

Pure standard library. No runtime starts on import. The function names and
semantics match those consumed by the separately retained MC-trace modules
(`digest`, `strict_json`, `atomic_json(exclusive=...)`, `tree_manifest`).
"""
import hashlib
import json
import os
from pathlib import Path
import uuid


def digest(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def digest_bytes(raw):
    return hashlib.sha256(raw).hexdigest()


def _pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate_json_key:" + key)
        result[key] = value
    return result


def _constant(value):
    raise ValueError("non_json_constant:" + value)


def strict_loads(text):
    return json.loads(text, object_pairs_hook=_pairs, parse_constant=_constant)


def strict_json(path):
    return strict_loads(Path(path).read_text())


def strict_jsonl(path):
    """Every line must be complete; a partial trailing record is never accepted."""
    text = Path(path).read_text()
    if text and not text.endswith("\n"):
        raise ValueError("partial_jsonl:" + str(path))
    return [strict_loads(line) for line in text.splitlines()]


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def canonical_sha256(value):
    return hashlib.sha256(canonical(value).encode()).hexdigest()


def atomic_json(path, value, exclusive=False):
    """Fsynced publication; exclusive mode can never replace an existing record."""
    path = Path(path)
    raw = json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n"
    temporary = path.with_name(path.name + ".tmp-" + uuid.uuid4().hex)
    with temporary.open("x") as stream:
        stream.write(raw)
        stream.flush()
        os.fsync(stream.fileno())
    try:
        if exclusive:
            os.link(temporary, path)
        else:
            os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def tree_manifest(directory):
    directory = Path(directory)
    return {p.relative_to(directory).as_posix(): digest(p) for p in sorted(directory.rglob("*"))
            if p.is_file() and "__pycache__" not in p.parts and p.suffix != ".pyc"}
