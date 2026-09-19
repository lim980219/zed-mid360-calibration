#!/usr/bin/env python3
"""Build a source-only GitHub ZIP. Does not include bags or private exports."""
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[1]
ROOT_FILES = (
    "README.md", "LICENSE", "CHANGELOG.md", "CONTRIBUTING.md",
    ".gitignore", ".gitattributes", "CMakeLists.txt", "package.xml",
    "setup.py", "setup.cfg", "pyproject.toml", "requirements-dev.txt",
)
PATTERNS = (
    "zed_mid360_calibration/*.py", "config/*.yaml", "scripts/*.py",
    "tests/*.py", "docs/*.md", ".github/workflows/*.yml",
)


def main():
    paths = {ROOT / name for name in ROOT_FILES}
    for pattern in PATTERNS:
        paths.update(ROOT.glob(pattern))
    for path in paths:
        if not path.is_file() or path.is_symlink():
            raise ValueError("Expected regular release file: " + str(path))
        path.resolve().relative_to(ROOT)
    destination = ROOT / "dist"
    destination.mkdir(exist_ok=True)
    archive = destination / "zed_mid360_calibration-github.zip"
    with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as output:
        for path in sorted(paths):
            name = path.relative_to(ROOT).as_posix()
            # Portable UTF-8/LF source text, without Windows BOM.
            payload = path.read_text(encoding="utf-8-sig").replace("\r\n", "\n").encode("utf-8")
            info = zipfile.ZipInfo(name)
            info.create_system = 3
            info.external_attr = (0o100755 if name.startswith("scripts/") else 0o100644) << 16
            info.compress_type = zipfile.ZIP_DEFLATED
            output.writestr(info, payload)
    print("{} ({} files)".format(archive, len(paths)))


if __name__ == "__main__":
    main()

