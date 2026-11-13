"""수업 로봇 모델(ASMR2026 Humanoid v1: MJCF + STL)을 내려받아 assets/에 푼다.

    python fetch_model.py

출처: https://snu-srbl.github.io/asmr2026-docs/simulation/robot-model/  (Robot Model v1, 2026-10-16)
수업 자료(All rights reserved)라서 공개 저장소에는 올리지 않는다 (.gitignore).
"""
import io
import sys
import urllib.request
import zipfile
from pathlib import Path

URL = "https://snu-srbl.github.io/asmr2026-docs/assets/downloads/asmr2026-humanoid-v1.zip"
ASSETS = Path(__file__).resolve().parent / "assets"
TOP = "asmr2026-humanoid-v1/"


def fetch(progress=print):
    progress(f"내려받는 중: {URL}")
    with urllib.request.urlopen(URL, timeout=60) as r:
        blob = r.read()
    progress(f"{len(blob) / 1e6:.1f} MB 받음, 푸는 중...")
    ASSETS.mkdir(parents=True, exist_ok=True)
    root = ASSETS.resolve()
    with zipfile.ZipFile(io.BytesIO(blob)) as z:
        for info in z.infolist():
            name = info.filename
            if not name.startswith(TOP) or ".." in Path(name).parts or Path(name).is_absolute():
                continue
            dest = (ASSETS / name).resolve()
            if root not in dest.parents:
                continue
            if info.is_dir():
                dest.mkdir(parents=True, exist_ok=True)
                continue
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes(z.read(info))
    xml = ASSETS / TOP / "mjcf" / "asmr2026_humanoid.xml"
    if not xml.exists():
        raise RuntimeError("압축 안에 mjcf/asmr2026_humanoid.xml이 없습니다")
    progress(f"완료: {xml}")
    return xml


if __name__ == "__main__":
    try:
        fetch()
    except Exception as e:  # noqa: BLE001
        sys.exit(f"실패: {e}")
