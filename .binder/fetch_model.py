"""Download only the pinned FR3 model at image-build time, retaining its license."""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parent
DEST = ROOT / 'assets'
REVISION = 'feadf76d42f8a2162426f7d226a3b539556b3bf5'

def git(*args):
    subprocess.run(['git', '-C', str(DEST), *args], check=True)

DEST.mkdir(exist_ok=True)
if not (DEST / '.git').is_dir():
    git('init', '--quiet')
    git('remote', 'add', 'origin', 'https://github.com/google-deepmind/mujoco_menagerie.git')
git('config', 'remote.origin.promisor', 'true')
git('config', 'remote.origin.partialclonefilter', 'blob:none')
git('fetch', '--depth=1', '--filter=blob:none', 'origin', REVISION)
git('sparse-checkout', 'init', '--cone')
git('sparse-checkout', 'set', 'franka_fr3_v2')
git('checkout', '--detach', REVISION)
assert (DEST / 'franka_fr3_v2/fr3v2.xml').is_file()
print('FR3 model prepared:', REVISION)
