#!/usr/bin/env python3
"""Write the browser examples' index page from the gallery's card table.

examples/example_gallery.c lists every example with a category and a short
description (categories[] and cards[]).  The web build has no gallery
executable, so this script turns the same table into a static index.html
that links to example_<name>.html.

    make_index.py <example_gallery.c> <index.html> [<example targets>]

The optional third argument, a comma-separated list of the example targets
(example_<name>), is checked against the cards in both directions, as the
gallery's headless self-check does for the native executables.
"""

import html
import re
import sys


def parse_gallery(source):
    categories_match = re.search(
        r"static const char \*categories\[\]\s*=\s*\{(.*?)\};", source, re.S)
    cards_match = re.search(
        r"static const card_t cards\[\]\s*=\s*\{(.*?)\n\};", source, re.S)
    if not categories_match or not cards_match:
        raise ValueError("categories[] or cards[] not found")
    categories = re.findall(r'"((?:[^"\\]|\\.)*)"', categories_match.group(1))
    cards = [
        (name, int(category), text)
        for name, category, text in re.findall(
            r'\{\s*"([a-z0-9_]+)"\s*,\s*(\d+)\s*,\s*"((?:[^"\\]|\\.)*)"\s*\}',
            cards_match.group(1))
    ]
    if not categories or not cards:
        raise ValueError("empty categories[] or cards[]")
    for name, category, _ in cards:
        if not 0 <= category < len(categories):
            raise ValueError(f"card {name}: category {category} out of range")
    return categories, cards


PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>IVP examples</title>
<style>
  :root {{
    --bg: #f6f7f9;
    --card: #ffffff;
    --border: #dde1e6;
    --text: #1d2128;
    --muted: #5d6570;
    --accent: #2f6fd6;
  }}
  @media (prefers-color-scheme: dark) {{
    :root {{
      --bg: #1d2128;
      --card: #262b33;
      --border: #363c46;
      --text: #e6e8eb;
      --muted: #9aa3ad;
      --accent: #7fb2ff;
    }}
  }}
  * {{ box-sizing: border-box; }}
  body {{
    margin: 0;
    padding: 24px 16px 48px;
    background: var(--bg);
    color: var(--text);
    font: 15px/1.5 system-ui, -apple-system, "Segoe UI", sans-serif;
  }}
  .page {{ max-width: 1040px; margin: 0 auto; }}
  h1 {{ font-size: 24px; margin: 0 0 4px; }}
  .intro {{ color: var(--muted); margin: 0 0 24px; }}
  h2 {{ font-size: 16px; margin: 28px 0 10px; color: var(--muted); font-weight: 600; }}
  ul {{
    list-style: none;
    margin: 0;
    padding: 0;
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(240px, 1fr));
    gap: 10px;
  }}
  a {{
    display: block;
    height: 100%;
    padding: 12px 14px;
    background: var(--card);
    border: 1px solid var(--border);
    border-radius: 8px;
    color: inherit;
    text-decoration: none;
  }}
  a:hover, a:focus-visible {{ border-color: var(--accent); }}
  .name {{ display: block; font-weight: 600; color: var(--accent); }}
  .text {{ display: block; color: var(--muted); font-size: 14px; margin-top: 2px; }}
</style>
</head>
<body>
<div class="page">
<h1>IVP examples</h1>
<p class="intro">The libivp physics examples, compiled to WebAssembly and drawn with WebGL2.
Each one runs in its own page.</p>
{sections}
</div>
</body>
</html>
"""


def render(categories, cards):
    sections = []
    for index, category in enumerate(categories):
        items = []
        for name, card_category, text in cards:
            if card_category != index:
                continue
            items.append(
                f'  <li><a href="example_{name}.html">'
                f'<span class="name">{html.escape(name.replace("_", " "))}</span>'
                f'<span class="text">{html.escape(text)}</span></a></li>')
        if items:
            sections.append(f"<h2>{html.escape(category)}</h2>\n<ul>\n"
                            + "\n".join(items) + "\n</ul>")
    return PAGE.format(sections="\n".join(sections))


def check_targets(cards, targets):
    """Messages for cards without a target and targets without a card."""
    names = {name for name, _, _ in cards}
    built = {t[len("example_"):] for t in targets if t.startswith("example_")}
    problems = [f"card {n} has no example_{n} target" for n in sorted(names - built)]
    problems += [f"example_{n} has no card in example_gallery.c" for n in sorted(built - names)]
    return problems


def main(argv):
    if len(argv) not in (3, 4):
        sys.stderr.write(__doc__)
        return 2
    with open(argv[1], encoding="utf-8") as f:
        categories, cards = parse_gallery(f.read())
    if len(argv) == 4:
        problems = check_targets(cards, [t for t in argv[3].split(",") if t])
        for problem in problems:
            sys.stderr.write(f"make_index.py: {problem}\n")
        if problems:
            return 1
    with open(argv[2], "w", encoding="utf-8") as f:
        f.write(render(categories, cards))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
