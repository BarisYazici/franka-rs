#!/usr/bin/env python3
"""Build the guide and overlay its public landing page; optionally include local videos."""

import argparse
from html import escape
from pathlib import Path
import re
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[1]
VIDEOS = ("architecture-overview.mp4", "bridge-demo.mp4", "rerun-demo.mp4")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--videos-dir", type=Path, help="Optional directory of demo videos")
    args = parser.parse_args()
    subprocess.run(["mdbook", "build", str(ROOT / "docs/book")], check=True)
    destination = ROOT / "target/book"
    source = ROOT / "docs/site"
    shutil.copytree(source, destination / "site", dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns("README.md", "index.html"))
    media = destination / "site/media"
    media.mkdir(parents=True, exist_ok=True)
    for name in VIDEOS:
        (media / name).unlink(missing_ok=True)
    if args.videos_dir:
        for name in VIDEOS:
            video = args.videos_dir / name
            preferred = {
                "architecture-overview.mp4": "architecture-overview-slow.mp4",
                "bridge-demo.mp4": "bridge-demo-short.mp4",
            }.get(name)
            if preferred and (args.videos_dir / preferred).is_file():
                video = args.videos_dir / preferred
            if video.is_file():
                shutil.copy2(video, media / name)
            else:
                print(f"Optional video not found: {video}")
    # Make a clean deployment work without JavaScript or requests for absent videos.
    def optional_video(match):
        block = match.group(0)
        src = re.search(r'<source\s+src="([^"]+)"', block)
        if src and (destination / src[1]).is_file():
            return block
        poster = re.search(r'poster="([^"]+)"', block)
        alt = re.search(r'data-alt="([^"]+)"', block)
        if not poster:
            raise ValueError("An optional video needs a poster")
        return f'<img src="{poster[1]}" alt="{escape(alt[1] if alt else "Architecture overview")}">'

    html = re.sub(r"<video\b.*?</video>", optional_video,
                  (source / "index.html").read_text(), flags=re.DOTALL)
    (destination / "index.html").write_text(html)
    (destination / ".nojekyll").touch()
    print(f"Site built: {destination}")
    print("Preview: python3 -m http.server 8080 --directory target/book")


if __name__ == "__main__":
    main()
