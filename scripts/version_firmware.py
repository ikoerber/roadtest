"""Name and export the ROADTEST firmware from the central version macro."""

from pathlib import Path
import re
import shutil

Import("env")


project_dir = Path(env.subst("$PROJECT_DIR"))
version_header = project_dir / "src" / "hardware_config.h"
version_source = version_header.read_text(encoding="utf-8")
version_match = re.search(
    r'^#define\s+ROADTEST_FIRMWARE_VERSION\s+"([0-9]+\.[0-9]+\.[0-9]+)"',
    version_source,
    re.MULTILINE,
)

if version_match is None:
    raise RuntimeError(
        "ROADTEST_FIRMWARE_VERSION fehlt oder ist keine dreiteilige Version"
    )

firmware_version = version_match.group(1)
program_name = f"roadtest_{firmware_version}"
firmware_file_name = f"{program_name}.bin"

env.Replace(PROGNAME=program_name)


def export_versioned_firmware(target, source, env):
    built_firmware = Path(env.subst("$BUILD_DIR")) / firmware_file_name
    if not built_firmware.is_file():
        raise RuntimeError(
            f"Erzeugte Firmwaredatei fehlt: {built_firmware}"
        )
    exported_firmware = project_dir / firmware_file_name
    shutil.copy2(built_firmware, exported_firmware)
    print(f"Exported versioned firmware: {exported_firmware}")


# Der Alias wird auch bei inkrementellen Builds zuverlässig nach Abschluss
# des vollständigen Firmware-Builds ausgeführt. Eine Aktion direkt am
# umbenannten .bin-Ziel wurde von PlatformIO nicht in jedem Lauf ausgelöst.
env.AddPostAction("buildprog", export_versioned_firmware)
print(f"ROADTEST firmware version: {firmware_version} ({firmware_file_name})")
