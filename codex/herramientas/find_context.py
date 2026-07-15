#!/usr/bin/env python3
import argparse
import json
import re
from pathlib import Path

try:
    import yaml
except ImportError:
    yaml = None


def load_yaml(path: Path):
    if yaml is None:
        raise RuntimeError("PyYAML is required to run this script. Install it with pip install pyyaml")
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def read_text(path: Path):
    try:
        return path.read_text(encoding="utf-8")
    except Exception:
        return ""


def collect_package_summaries(packages_dir: Path, summary_name: str):
    summaries = []
    if not packages_dir.exists():
        return summaries
    for pkg_dir in packages_dir.iterdir():
        if not pkg_dir.is_dir():
            continue
        summary_path = pkg_dir / summary_name
        if summary_path.exists():
            summaries.append(summary_path)
    return summaries


def match_text(text: str, query: re.Pattern):
    return bool(query.search(text))


def find_matches(paths, query_pattern):
    matches = []
    for path in paths:
        text = read_text(path)
        if match_text(text, query_pattern):
            first_line = next((line.strip() for line in text.splitlines() if line.strip()), "")
            matches.append({"path": str(path), "title": first_line})
    return matches


def build_quick_matches(index, query_pattern):
    matches = []
    if "package_map" in index:
        package_map = Path(index["package_map"])
        if package_map.exists():
            matches.extend(find_matches([package_map], query_pattern))
    if "read_order" in index:
        for item in index["read_order"]:
            path = Path(item)
            if path.exists():
                matches.extend(find_matches([path], query_pattern))
    return matches


def collect_subfase_files(subfases_dir: Path):
    return sorted([p for p in subfases_dir.glob("*.md") if p.is_file()])


def main():
    parser = argparse.ArgumentParser(description="Find the most likely context files for a query using CODEX_INDEX.yaml and short summaries.")
    parser.add_argument("query", nargs="+", help="Search query terms")
    parser.add_argument("--index", default="codex/contexto/CODEX_INDEX.yaml", help="Path to CODEX_INDEX.yaml")
    parser.add_argument("--deep", action="store_true", help="Search all markdown files under codex/contexto and codex/pipeline if no short matches are found")
    parser.add_argument("--json", action="store_true", help="Output results as JSON")
    args = parser.parse_args()

    query = " ".join(args.query)
    pattern = re.compile(re.escape(query), re.IGNORECASE)

    index_path = Path(args.index)
    if not index_path.exists():
        raise FileNotFoundError(f"Index file not found: {index_path}")

    index = load_yaml(index_path)
    package_dir = Path(index.get("packages_dir", "codex/contexto/paquetes/"))
    summary_name = index.get("short_summary_name", "00_summary.md")
    subfases_dir = Path(index.get("subfases_dir", "codex/pipeline/fase_1_sparse_global/subfases/"))

    summary_files = collect_package_summaries(package_dir, summary_name)
    matches = find_matches(summary_files, pattern)

    if not matches:
        matches = build_quick_matches(index, pattern)

    if not matches and args.deep:
        all_md = list(Path("codex/contexto").rglob("*.md")) + list(Path("codex/pipeline").rglob("*.md"))
        matches = find_matches(all_md, pattern)

    if not args.json:
        if matches:
            print(f"Found {len(matches)} candidate file(s) for query: '{query}'\n")
            for m in matches:
                print(f"- {m['path']}")
                if m["title"]:
                    print(f"  title: {m['title']}")
        else:
            print(f"No short-context matches found for '{query}'.")
            print("Try --deep to search all markdown files, or refine the query.")
    else:
        print(json.dumps({"query": query, "matches": matches}, indent=2, ensure_ascii=False))

if __name__ == "__main__":
    main()
