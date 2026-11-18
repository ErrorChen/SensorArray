#!/usr/bin/env python3
"""Archive explicitly selected, already closed evidence; retain all originals.

The caller must establish closure. No device access, deletion or overwrite is
performed. Existing gzip files are verified rather than rewritten. Every gzip
is decompressed and byte/hash-compared with the unchanged original afterward.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
from pathlib import Path


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording", action="append", required=True, type=Path)
    parser.add_argument("--evidence", action="append", default=[], type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError("index exists; use a new label")
    paths = sorted(set(args.recording))
    if any(not p.is_file() or p.suffix == ".gz" for p in paths):
        raise ValueError("recordings must name original regular files")
    evidence = [{"path": str(p), "sha256": sha(p.read_bytes()), "bytes": p.stat().st_size}
                for p in args.evidence]
    rows = []
    for path in paths:
        original = path.read_bytes()
        before = sha(original)
        target = Path(str(path)+".gz")
        created = not target.exists()
        if created:
            compressed = gzip.compress(original, compresslevel=6, mtime=0)
            with target.open("xb") as stream:
                stream.write(compressed)
        compressed = target.read_bytes()
        restored = gzip.decompress(compressed)
        if restored != original or sha(restored) != before:
            raise ValueError("gzip roundtrip mismatch: "+str(path))
        if sha(path.read_bytes()) != before:
            raise ValueError("original changed during archive: "+str(path))
        rows.append({"path": str(path), "raw_bytes": len(original), "raw_sha256": before,
                     "gzip": str(target), "gzip_bytes": len(compressed), "gzip_sha256": sha(compressed),
                     "gzip_created": created, "roundtrip_verified": True, "original_retained": True})
    for item in evidence:
        if sha(Path(item["path"]).read_bytes()) != item["sha256"]:
            raise ValueError("evidence changed during archive: "+item["path"])
    result = {"schema_version": 1, "status": "VERIFIED", "files": rows, "evidence": evidence,
              "file_count": len(rows), "total_original_bytes": sum(r["raw_bytes"] for r in rows),
              "total_gzip_bytes": sum(r["gzip_bytes"] for r in rows),
              "closure": "caller selected already closed recordings; no active device access",
              "original_files_modified_or_deleted": False, "hardware_actions": [],
              "generator": {"path": str(Path(__file__)), "sha256": sha(Path(__file__).read_bytes())}}
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2); stream.write("\n")
    print(json.dumps({"status": result["status"], "files": len(rows),
                      "raw_bytes": result["total_original_bytes"], "gzip_bytes": result["total_gzip_bytes"],
                      "original_retained": True}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
