"""PlatformIO pre-build hook: ensure config/vision_profiles_generated.h exists.

Runs at project-configuration time for every environment (REQ-DER item 19).
Never overwrites an operator-calibrated header - it only generates the
canonical seed table when the file is missing, then validates structure.

A failure here must never break a build: the firmware falls back to the
built-in profile table in config/vision_profiles.h when the header is absent,
so problems are reported as warnings instead of hard errors.
"""

import os
import subprocess
import sys

try:
    Import("env")  # noqa: F821  (provided by PlatformIO SCons)
except NameError:
    # Allow direct execution for smoke-testing outside PlatformIO.
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--project_dir", default="..")
    args = parser.parse_args()
    env = None
    PROJECT_DIR = os.path.abspath(args.project_dir)
else:
    PROJECT_DIR = env.subst("$PROJECT_DIR")

SCRIPT = os.path.join(PROJECT_DIR, "scripts", "gen_vision_profiles.py")


def _run(args_list):
    return subprocess.call([sys.executable] + args_list)


def main():
    try:
        rc = _run([SCRIPT, "--if-missing"])
        if rc == 0:
            rc = _run([SCRIPT, "--check"])
    except Exception as exc:  # offline / permission issues must not kill builds
        print("[PIO_EXTRA][WARN] gen_vision_profiles.py could not run: %s" % exc)
        print("[PIO_EXTRA][WARN] Continuing with built-in vision profiles.")
        return


main()
