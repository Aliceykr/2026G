from pathlib import Path
import json
import re

import win32com.client
from pypdf import PdfReader


ROOT = Path(__file__).resolve().parent
OUT = ROOT / "tmp" / "extracted"
OUT.mkdir(parents=True, exist_ok=True)


def clean(text: str) -> str:
    text = text.replace("\r\x07", "\n").replace("\x07", "\t").replace("\r", "\n")
    text = re.sub(r"[ \t]+\n", "\n", text)
    text = re.sub(r"\n{3,}", "\n\n", text)
    return text.strip()


records = []
word = win32com.client.DispatchEx("Word.Application")
word.Visible = False
word.DisplayAlerts = 0
try:
    for path in sorted(ROOT.iterdir()):
        if path.suffix.lower() not in {".doc", ".docx"}:
            continue
        doc = None
        try:
            doc = word.Documents.Open(str(path), ReadOnly=True, AddToRecentFiles=False)
            text = clean(doc.Content.Text)
            out = OUT / f"{path.name}.txt"
            out.write_text(text, encoding="utf-8")
            records.append({"file": path.name, "type": "word", "chars": len(text), "output": str(out)})
        except Exception as exc:
            records.append({"file": path.name, "type": "word", "error": str(exc)})
        finally:
            if doc is not None:
                doc.Close(False)
finally:
    word.Quit()

for path in sorted(ROOT.glob("*.pdf")):
    try:
        reader = PdfReader(str(path))
        pages = []
        for i, page in enumerate(reader.pages, 1):
            pages.append(f"\n===== PAGE {i} =====\n{page.extract_text() or ''}")
        text = clean("\n".join(pages))
        out = OUT / f"{path.name}.txt"
        out.write_text(text, encoding="utf-8")
        records.append({"file": path.name, "type": "pdf", "pages": len(reader.pages), "chars": len(text), "output": str(out)})
    except Exception as exc:
        records.append({"file": path.name, "type": "pdf", "error": str(exc)})

(OUT / "index.json").write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")
print(json.dumps(records, ensure_ascii=False, indent=2))
