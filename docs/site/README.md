# Website

The landing page helps a new user choose a setup. The mdBook contains the installation
guides and technical reference. They build into one site, with existing book URLs intact.

From the repository root, with Python 3 and mdBook 0.5.4 installed:

```sh
python3 tools/build-site.py
python3 -m http.server 8080 --directory target/book
```

Open <http://localhost:8080>. No Node.js packages or frontend build tools are required.
The API reference is built separately by the docs workflow. To include it locally:

```sh
cargo doc --no-deps -p franka-rs
mkdir -p target/book/api
cp -a target/doc/. target/book/api/
```

## Demo videos

Videos are optional local assets and are not committed. To include them in a preview:

```sh
python3 tools/build-site.py --videos-dir /path/to/videos
```

The build accepts `architecture-overview.mp4`, `bridge-demo.mp4`, and `rerun-demo.mp4`.
If `architecture-overview-slow.mp4` is present, it is used in place of the original overview.
This reading-paced export plays at 0.75× the original speed (about 89 seconds instead of 67).
The original video is preserved. To create the slower copy without re-encoding its silent video:

```sh
ffmpeg -n -itsscale 1.3333333333333333 -i /path/to/videos/architecture-overview.mp4 \
  -map 0:v:0 -c:v copy -an -movflags +faststart /path/to/videos/architecture-overview-slow.mp4
```

If `bridge-demo-short.mp4` is present, it replaces the left “From targets to motion” video.
This cut ends at 0:29 and preserves the original `bridge-demo.mp4`:

```sh
ffmpeg -n -i /path/to/videos/bridge-demo.mp4 -t 29 -map 0:v:0 \
  -c:v libx264 -preset fast -crf 18 -an -movflags +faststart /path/to/videos/bridge-demo-short.mp4
```

Poster images and the text explanation remain available without the videos. A clean
GitHub Actions build does not include these local videos. Hosting them publicly is a
separate publishing step; add approved media to the build input when ready.

## Editing

- `index.html`: use cases, setup choices, and links into the guide.
- `site.css`: layout, responsive styles, and reduced-motion preferences.
- `site.js`: progressive enhancements; the content remains readable without JavaScript.
- `../book/src/getting-started/`: installation instructions and ecosystem overview.

Links on the landing page resolve from the site root, not from `site/`. The build replaces
mdBook's generated `index.html`; the book introduction remains at `introduction.html`.
The existing Pages workflow builds this site but only deploys the `main` branch.

## Pi hardware guide

The Pi setup is a four-step route starting at `getting-started/raspberry-pi.html`.
Curated renders, GLB models and printable/editable downloads live in
`docs/book/src/assets/pi5/`. See its `SOURCES.txt`, retained Raspberry Pi license, and
`manifest.json`. The print bundle contains geometry, not machine-specific G-code.

`site/pi-viewer.html` loads the local GLBs with vendored Three.js 0.147.0; no CDN
connection or build package manager is required. The Three.js MIT notice is retained
in `docs/site/vendor/three/LICENSE`. No private source directory is served or copied
by the build script.
