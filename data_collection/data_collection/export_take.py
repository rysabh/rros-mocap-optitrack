import csv
import os
from pathlib import Path
from typing import Any, Dict, List, Set

import yaml


def _load_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data if isinstance(data, dict) else {}


def _extract_rigid_bodies(mocap_yaml: Dict[str, Any]) -> Dict[int, List[float]]:
    bodies: Dict[int, List[float]] = {}
    for rb in mocap_yaml.get("rigid_bodies", []) or []:
        rb_id = int(rb.get("id", 0))
        pose = rb.get("pose_stamped", {}).get("pose", {})
        pos = pose.get("position", {})
        ori = pose.get("orientation", {})
        bodies[rb_id] = [
            float(pos.get("x", 0.0)),
            float(pos.get("y", 0.0)),
            float(pos.get("z", 0.0)),
            float(ori.get("w", 0.0)),
            float(ori.get("x", 0.0)),
            float(ori.get("y", 0.0)),
            float(ori.get("z", 0.0)),
        ]
    return bodies


def export_take(take_dir: Path, output_csv: Path) -> None:
    samples_path = take_dir / "samples.csv"
    if not samples_path.exists():
        raise FileNotFoundError(f"Missing samples.csv at {samples_path}")

    rows: List[Dict[str, Any]] = []
    all_rb_ids: Set[int] = set()

    with samples_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        base_fields = list(reader.fieldnames or [])

        for row in reader:
            mocap_rel = row.get("mocap_yaml_path", "")
            if mocap_rel:
                mocap_path = take_dir / mocap_rel
                if mocap_path.exists():
                    mocap_yaml = _load_yaml(mocap_path)
                    rigid_bodies = _extract_rigid_bodies(mocap_yaml)
                    row["_rigid_bodies"] = rigid_bodies
                    all_rb_ids.update(rigid_bodies.keys())
            rows.append(row)

    rb_fields: List[str] = []
    for rb_id in sorted(all_rb_ids):
        rb_fields.extend(
            [
                f"rb_{rb_id}_x",
                f"rb_{rb_id}_y",
                f"rb_{rb_id}_z",
                f"rb_{rb_id}_qw",
                f"rb_{rb_id}_qx",
                f"rb_{rb_id}_qy",
                f"rb_{rb_id}_qz",
            ]
        )

    out_fields = [f for f in base_fields if f != "mocap_yaml_path"] + rb_fields
    output_csv.parent.mkdir(parents=True, exist_ok=True)

    with output_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=out_fields)
        writer.writeheader()

        for row in rows:
            rigid_bodies: Dict[int, List[float]] = row.pop("_rigid_bodies", {}) or {}
            out_row = {k: row.get(k, "") for k in out_fields}

            for rb_id, values in rigid_bodies.items():
                (
                    out_row[f"rb_{rb_id}_x"],
                    out_row[f"rb_{rb_id}_y"],
                    out_row[f"rb_{rb_id}_z"],
                    out_row[f"rb_{rb_id}_qw"],
                    out_row[f"rb_{rb_id}_qx"],
                    out_row[f"rb_{rb_id}_qy"],
                    out_row[f"rb_{rb_id}_qz"],
                ) = values

            writer.writerow(out_row)


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Export a recorded take into a single flattened CSV.")
    parser.add_argument("--take-dir", required=True, help="Path to a take directory (contains samples.csv).")
    parser.add_argument(
        "--output-csv",
        default="",
        help="Output CSV path (default: <take-dir>/export.csv).",
    )
    args = parser.parse_args()

    take_dir = Path(os.path.expanduser(args.take_dir)).resolve()
    output_csv = (
        Path(os.path.expanduser(args.output_csv)).resolve()
        if args.output_csv
        else (take_dir / "export.csv")
    )

    export_take(take_dir=take_dir, output_csv=output_csv)
    print(f"Wrote {output_csv}")


if __name__ == "__main__":
    main()

