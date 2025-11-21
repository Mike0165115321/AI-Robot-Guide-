#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Merge `details` from details.json/details.jsonl into nan_data_compiled.json.

Usage:
  python merge_details.py details.json nan_data_compiled.json
  python merge_details.py details.jsonl nan_data_compiled.json --out merged.json --mode replace

Modes:
  - fill     (default) update only when target details is missing or empty
  - replace  overwrite target details regardless of current value
  - append   append new detail items (deduplicated by (heading, content))
"""

import argparse
import json
import os
from typing import Dict, List, Any


def load_details_map(path: str) -> Dict[str, List[Dict[str, Any]]]:
    """
    Load details from JSON (array) or JSON Lines into a mapping: id -> details list.
    Expected item shape: {"id": "...", "details": [ { "heading": "...", "content": "..." }, ... ]}
    """
    def is_valid_detail_list(v: Any) -> bool:
        if not isinstance(v, list):
            return False
        for it in v:
            if not isinstance(it, dict):
                return False
            if "heading" not in it or "content" not in it:
                return False
        return True

    mapping: Dict[str, List[Dict[str, Any]]] = {}

    with open(path, "r", encoding="utf-8") as f:
        raw = f.read().lstrip("\ufeff")  # strip BOM if present

    try:
        data = json.loads(raw)
        if not isinstance(data, list):
            raise ValueError("Details file must be a JSON array or JSON Lines.")
        items = data
    except json.JSONDecodeError:
        # Try JSON Lines
        items = []
        for line in raw.splitlines():
            line = line.strip()
            if not line:
                continue
            items.append(json.loads(line))

    for obj in items:
        if not isinstance(obj, dict):
            continue
        _id = obj.get("id")
        det = obj.get("details")
        if isinstance(_id, str) and is_valid_detail_list(det):
            mapping[_id] = det

    return mapping


def load_compiled(path: str) -> List[Dict[str, Any]]:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
        if not isinstance(data, list):
            raise ValueError("nan_data_compiled.json must contain a JSON array.")
        return data


def save_json(path: str, data: Any) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def append_merge(existing: List[Dict[str, Any]], incoming: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    seen = {(d.get("heading"), d.get("content")) for d in existing if isinstance(d, dict)}
    merged = list(existing)
    for d in incoming:
        key = (d.get("heading"), d.get("content"))
        if key not in seen:
            merged.append(d)
            seen.add(key)
    return merged


def main():
    parser = argparse.ArgumentParser(description="Merge details into nan_data_compiled.json")
    parser.add_argument("details_path", help="Path to details.json or details.jsonl")
    parser.add_argument("compiled_path", help="Path to nan_data_compiled.json")
    parser.add_argument("--out", default=None, help="Output path (default: overwrite compiled or create .bak)")
    parser.add_argument(
        "--mode",
        choices=["fill", "replace", "append"],
        default="fill",
        help="Merge mode: fill (default), replace, append"
    )
    args = parser.parse_args()

    details_map = load_details_map(args.details_path)
    compiled = load_compiled(args.compiled_path)

    updated = 0
    skipped = 0
    missing = 0

    for item in compiled:
        _id = item.get("id")
        if not isinstance(_id, str):
            skipped += 1
            continue

        incoming = details_map.get(_id)
        if incoming is None:
            missing += 1
            continue

        current = item.get("details")
        if args.mode == "fill":
            if not current:
                item["details"] = incoming
                updated += 1
            else:
                skipped += 1
        elif args.mode == "replace":
            item["details"] = incoming
            updated += 1
        elif args.mode == "append":
            if not isinstance(current, list):
                current = []
            item["details"] = append_merge(current, incoming)
            updated += 1

    out_path = args.out
    if out_path is None:
        # backup then overwrite
        bak_path = args.compiled_path + ".bak"
        if not os.path.exists(bak_path):
            save_json(bak_path, compiled)
        # Save merged to original path
        out_path = args.compiled_path

    save_json(out_path, compiled)

    # Print a concise summary
    print(f"Merged details: {updated} updated, {skipped} skipped, {missing} ids not found in details map")
    print(f"Output written to: {out_path}")


if __name__ == "__main__":
    main()
