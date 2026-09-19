from . import OculusReader  # vendoring edit -- see reader.py's header comment
from .fetch_apk import fetch  # vendoring edit -- the APK is fetched, not committed

def main(argv=None):
    import argparse

    parser = argparse.ArgumentParser(description='Utility to manage teleoperation APK. Installs APK if no arguments are provided.')
    parser.add_argument("--reinstall", action="store_true", help='reinstalls APK from the default path')
    parser.add_argument("--uninstall", action="store_true", help='uninstalls APK')
    args = parser.parse_args(argv)

    # vendoring edit: the pinned build from the user's data dir, downloaded once and
    # verified every time; refused on a hash mismatch
    apk = None if args.uninstall else fetch()

    reader = OculusReader(run=False)

    if args.reinstall:
        reader.install(APK_path=apk, reinstall=True)
    elif args.uninstall:
        reader.uninstall()
    else:
        reader.install(APK_path=apk)
    print('Done.')

if __name__ == "__main__":
    main()
