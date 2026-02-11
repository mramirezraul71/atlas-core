"""Extractors: text from page, links, etc. (used with navigator)."""
from __future__ import annotations

from typing import Any, Dict, List


def extract_links_from_html(html: str, base_url: str = "") -> List[Dict[str, str]]:
    """Simple link extraction from HTML (href)."""
    import re
    links = []
    for m in re.finditer(r'<a\s+[^>]*href\s*=\s*["\']([^"\']+)["\']', html, re.I):
        href = m.group(1).strip()
        if href.startswith("#") or href.startswith("javascript:"):
            continue
        if base_url and not href.startswith("http"):
            from urllib.parse import urljoin
            href = urljoin(base_url, href)
        links.append({"href": href, "text": ""})
    return links[:100]


def extract_text_from_html(html: str) -> str:
    """Strip tags and return text (simple)."""
    import re
    text = re.sub(r"<script[^>]*>.*?</script>", "", html, flags=re.DOTALL | re.I)
    text = re.sub(r"<style[^>]*>.*?</style>", "", text, flags=re.DOTALL | re.I)
    text = re.sub(r"<[^>]+>", " ", text)
    text = re.sub(r"\s+", " ", text).strip()
    return text[:10000]
