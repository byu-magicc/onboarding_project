#!/usr/bin/env python3
import json
import re
import sys
from pathlib import Path

RESULTS_PATH = Path("docs/leaderboard_results.json")
README_PATH = Path("README.md")

URL_PATTERN = re.compile(r"^https?://[^\s]+$")
START_MARKER = "<!-- LEADERBOARD:START -->"
END_MARKER = "<!-- LEADERBOARD:END -->"


def validate_entry(entry, index):
    required_fields = ["name", "minimum_dist_meters", "time_s", "github", "video"]

    for field in required_fields:
        if field not in entry:
            raise ValueError(f"Entry {index}: missing required field '{field}'")

    if not isinstance(entry["name"], str) or not entry["name"]:
        raise ValueError(f"Entry {index}: 'name' must be a non-empty string")

    if (
        not isinstance(entry["minimum_dist_meters"], (int, float))
        or entry["minimum_dist_meters"] <= 0
    ):
        raise ValueError(
            f"Entry {index}: 'minimum_dist_meters' must be a positive number"
        )

    if not isinstance(entry["time_s"], (int, float)) or entry["time_s"] <= 0:
        raise ValueError(f"Entry {index}: 'time_s' must be a positive number")

    if not URL_PATTERN.match(entry["github"]):
        raise ValueError(f"Entry {index}: invalid GitHub URL '{entry['github']}'")

    if not URL_PATTERN.match(entry["video"]):
        raise ValueError(f"Entry {index}: invalid YouTube URL '{entry['video']}'")


def compute_score(min_dist, time):
    return min_dist / time


def main():
    try:
        with RESULTS_PATH.open() as f:
            data = json.load(f)
    except json.JSONDecodeError as e:
        print(f"Invalid JSON format: {e}")
        sys.exit(1)

    if not isinstance(data, list):
        print("'results.json' must contain a list of entries.")
        sys.exit(1)

    # Validate entries
    for i, entry in enumerate(data, start=1):
        validate_entry(entry, i)

    # Keep only best entry per name (highest minimum_dist_meters, then lowest time_s)
    best_entries = {}
    for entry in data:
        name = entry["name"].lower()
        current_best = best_entries.get(name)
        if current_best is None:
            best_entries[name] = entry
        else:
            new_score = compute_score(entry["minimum_dist_meters"], entry["time_s"])
            old_score = compute_score(
                current_best["minimum_dist_meters"], current_best["time_s"]
            )
            if new_score > old_score:
                best_entries[name] = entry

    # Sort: highest minimum distance first, then lowest time_s
    sorted_entries = sorted(
        best_entries.values(), key=lambda x: (-x["minimum_dist_meters"], x["time_s"])
    )

    # Generate Markdown
    leaderboard_lines = [
        START_MARKER,
        "<!-- The leaderboard below is automatically generated. Do not edit manually. -->",
        "| Rank | Name | Minimum distance from walls (m) | Best time (s) | GitHub | Video Link |",
        "|------|------|------------------|----------------|---------|----------|",
    ]

    for i, entry in enumerate(sorted_entries, start=1):
        leaderboard_lines.append(
            f"| {i} | {entry['name']} | "
            f"{entry['minimum_dist_meters']} | {entry['time_s']} | "
            f"[repo]({entry['github']}) | [video]({entry['video']}) |"
        )

    leaderboard_lines.append("")
    leaderboard_lines.append(END_MARKER)
    leaderboard_text = "\n".join(leaderboard_lines)

    # Read existing README
    text = README_PATH.read_text()

    # Replace leaderboard section
    if START_MARKER not in text or END_MARKER not in text:
        print(f"README.md must contain '{START_MARKER}' and '{END_MARKER}' markers.")
        sys.exit(1)

    updated_text = re.sub(
        f"{START_MARKER}.*?{END_MARKER}", leaderboard_text, text, flags=re.DOTALL
    )

    README_PATH.write_text(updated_text)
    print(f"Leaderboard updated with {len(sorted_entries)} unique participants.")


if __name__ == "__main__":
    try:
        main()
    except ValueError as e:
        print(f"Validation error: {e}")
        sys.exit(1)
