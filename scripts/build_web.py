"""Prepares the web page: packs the Python package and the examples into web/ (generated files, not committed).

You normally do not need to run this by hand: `python scripts/serve.py` does it every time the page asks for the files.

Usage:  python scripts/build_web.py
"""
import shutil
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
WEB = ROOT / "web"


def build() -> None:
    with zipfile.ZipFile(WEB / "vdcalc.zip", "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted((ROOT / "src" / "vdcalc").glob("*.py")):
            z.write(f, f"vdcalc/{f.name}")
    examples = WEB / "examples"
    examples.mkdir(exist_ok=True)
    shutil.copy(ROOT / "data" / "examples" / "mx5_nd2.json", examples / "mx5_nd2.json")
    shutil.copy(ROOT / "data" / "circuits" / "example_circuit.csv", examples / "example_circuit.csv")


if __name__ == "__main__":
    build()
    print("web/vdcalc.zip and web/examples/ ready")
