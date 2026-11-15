#!/usr/bin/env python3
"""ソースファイルの文字コードを検査する。

規則(.editorconfig と同じ):
  - .hpp / .cpp は UTF-8 (BOM 付き)、改行は CRLF
  - 上記以外の文字コード(CP932 や UTF-16)は不可

使い方:
  python tools/check_encoding.py          # 検査だけ。違反があれば終了コード 1
  python tools/check_encoding.py --fix    # 違反を修正する(UTF-8 BOM + CRLF に変換)
"""
import argparse
import pathlib
import sys

BOM = b"\xef\xbb\xbf"
EXTENSIONS = {".hpp", ".cpp", ".h", ".cc"}
ROOTS = ("include", "src", "tests", "examples")


def decode(raw: bytes):
    """BOM を除いた中身を文字列にする。どの文字コードだったかも返す。"""
    if raw.startswith(BOM):
        return raw[len(BOM):].decode("utf-8"), "utf-8-bom"
    if raw.startswith((b"\xff\xfe", b"\xfe\xff")):
        return raw.decode("utf-16"), "utf-16"
    try:
        return raw.decode("utf-8"), "utf-8"
    except UnicodeDecodeError:
        return raw.decode("cp932"), "cp932"


def main() -> int:
    sys.stdout.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--fix", action="store_true", help="違反を修正する")
    args = parser.parse_args()

    repo = pathlib.Path(__file__).resolve().parent.parent
    bad = 0
    for root in ROOTS:
        for path in sorted((repo / root).rglob("*")):
            if path.suffix not in EXTENSIONS or not path.is_file():
                continue
            raw = path.read_bytes()
            text, enc = decode(raw)
            has_lf_only = b"\n" in raw.replace(b"\r\n", b"")
            problems = []
            if enc != "utf-8-bom":
                problems.append(f"文字コード={enc} (UTF-8 BOM にする)")
            if has_lf_only:
                problems.append("改行が LF のみ (CRLF にする)")
            if not problems:
                continue

            rel = path.relative_to(repo)
            if args.fix:
                normalized = text.replace("\r\n", "\n").replace("\n", "\r\n")
                path.write_bytes(BOM + normalized.encode("utf-8"))
                print(f"fixed  {rel}: {', '.join(problems)}")
            else:
                bad += 1
                print(f"NG     {rel}: {', '.join(problems)}")

    if bad:
        print(f"\n{bad} file(s) violate the encoding rule. Run with --fix.")
        return 1
    print("encoding OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
