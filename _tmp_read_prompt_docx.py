"""Extrae texto de un .docx sin python-docx (ZIP + regex sobre document.xml)."""
import re
import sys
import zipfile
from pathlib import Path


def docx_to_text(path: Path) -> str:
    with zipfile.ZipFile(path) as z:
        raw = z.read("word/document.xml").decode("utf-8")
    # Cada párrafo (y celdas de tabla suelen ser w:p dentro de w:tc)
    blocks = re.split(r"</w:p>", raw)
    lines: list[str] = []
    for block in blocks:
        parts = re.findall(r"<w:t[^>]*>([^<]*)</w:t>", block)
        line = "".join(parts).replace("\xa0", " ").strip()
        if line:
            lines.append(line)
    return "\n".join(lines)


if __name__ == "__main__":
    p = Path(
        sys.argv[1]
        if len(sys.argv) > 1
        else r"C:\Users\r6957\Downloads\prompt_maestro_xgboost_atlas.docx"
    )
    if not p.is_file():
        print("NO_FILE", p)
        sys.exit(1)
    text = docx_to_text(p)
    out = Path(__file__).resolve().parent / "_tmp_prompt_maestro_xgboost_extracted.txt"
    out.write_text(text, encoding="utf-8")
    print(out)
    print("---LEN---", len(text))
