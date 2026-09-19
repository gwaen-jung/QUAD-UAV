#!/usr/bin/env python3
"""Generate the HTML asset included by the ESP32 firmware."""

Import("env")

from pathlib import Path

project_dir = Path(env["PROJECT_DIR"])
html_path = project_dir / "src" / "index.html"
header_path = project_dir / "src" / "index_html.h"
html = html_path.read_text(encoding="utf-8")
header = "#pragma once\n\nstatic const char index_html[] PROGMEM = R\"rawliteral(\n"
header += html
header += "\n)rawliteral\";\n"
header_path.write_text(header, encoding="utf-8")
print(f"[web_builder.py] generated {header_path}")
