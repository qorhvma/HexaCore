"""MkDocs 훅: 문서 안의 짧은 표기를 실제 내용으로 바꾼다.

- ``](repo:경로)``  → 저장소의 파일·폴더 링크 (mkdocs.yml의 repo_url 기준)
- ``{{ repo_url }}`` → mkdocs.yml의 repo_url (코드 블록 안에서도)
- ``{{ bom }}``     → hardware/bom/BOM.csv를 표로 (BOM을 두 곳에 따로 적지 않기 위해)
"""

from __future__ import annotations

import csv
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BOM = ROOT / "hardware/bom/BOM.csv"
REPO_LINK = re.compile(r"\]\(repo:([^)#\s]*)(#[^)\s]*)?\)")


def _repo_url(config, path: str) -> str:
    base = config["repo_url"].rstrip("/")
    kind = "tree" if path == "" or path.endswith("/") or (ROOT / path).is_dir() else "blob"
    if not (ROOT / path).exists():
        raise FileNotFoundError(f"repo: 링크가 가리키는 파일이 없습니다: {path}")
    return f"{base}/{kind}/main/{path}"


def _bom_table() -> str:
    with BOM.open(encoding="utf-8") as f:
        rows = list(csv.reader(f))
    esc = lambda c: c.replace("|", "\\|")  # noqa: E731
    lines = ["| " + " | ".join(rows[0]) + " |", "|" + "---|" * len(rows[0])]
    lines += ["| " + " | ".join(esc(c) for c in r) + " |" for r in rows[1:]]
    return "\n".join(lines)


def on_page_markdown(markdown, page, config, files):
    markdown = REPO_LINK.sub(lambda m: f"]({_repo_url(config, m.group(1))}{m.group(2) or ''})", markdown)
    markdown = markdown.replace("{{ repo_url }}", config["repo_url"].rstrip("/"))
    if "{{ bom }}" in markdown:
        markdown = markdown.replace("{{ bom }}", _bom_table())
    return markdown
