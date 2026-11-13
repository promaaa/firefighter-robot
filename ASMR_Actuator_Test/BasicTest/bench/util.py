"""입력 프롬프트, 데이터 저장 경로, CSV 기록."""
import csv
import json
import time
from pathlib import Path

DATA_ROOT = Path(__file__).resolve().parent.parent / "data"


def data_dir(act_id):
    d = DATA_ROOT / f"ID{act_id}"
    d.mkdir(parents=True, exist_ok=True)
    return d


def stamp():
    return time.strftime("%Y%m%d_%H%M%S")


def ask(prompt, default=""):
    s = input(prompt).strip()
    return s if s else default


def ask_float(prompt):
    """숫자 입력. 비우면 None."""
    while True:
        s = input(prompt).strip()
        if not s:
            return None
        try:
            return float(s)
        except ValueError:
            print("  숫자를 입력하거나 비워 두세요.")


def confirm(prompt):
    """Enter면 진행, q면 중단."""
    s = input(prompt + " [Enter=진행, q=중단] ").strip().lower()
    if s == "q":
        raise KeyboardInterrupt
    return True


def run(main):
    """스크립트 진입점. 안전 중단(RuntimeError)은 traceback 없이 이유만 출력."""
    try:
        main()
    except RuntimeError as e:
        raise SystemExit(f"\n중단: {e}")
    except (KeyboardInterrupt, EOFError):
        raise SystemExit("\n중단됨.")


def save_json(path, obj):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2, ensure_ascii=False, default=str)
    print(f"저장: {path}")


def append_csv(path, row):
    """dict 한 줄을 CSV에 덧붙임 (처음이면 헤더 작성)."""
    new = not Path(path).exists()
    with open(path, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(row.keys()))
        if new:
            w.writeheader()
        w.writerow(row)
    print(f"기록: {path}")


def write_log(path, meta, columns, rows):
    """첫 줄은 '# {meta json}', 다음 줄부터 CSV."""
    with open(path, "w", newline="", encoding="utf-8") as f:
        f.write("# " + json.dumps(meta, ensure_ascii=False, default=str) + "\n")
        w = csv.writer(f)
        w.writerow(columns)
        w.writerows(rows)
    print(f"저장: {path}")


def read_log(path):
    """write_log 형식 -> (meta, {열 이름: list[float]})."""
    with open(path, encoding="utf-8") as f:
        first = f.readline()
        meta = json.loads(first[1:]) if first.startswith("#") else {}
        if not first.startswith("#"):
            f.seek(0)
        reader = csv.DictReader(f)
        cols = {k: [] for k in reader.fieldnames}
        for r in reader:
            for k in cols:
                v = r[k]
                cols[k].append(float(v) if v not in ("", "None") else float("nan"))
    return meta, cols
