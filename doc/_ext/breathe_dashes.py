# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only
# License-Filename: LICENSE
# ------------------------------------------------------------------------------
"""Repair the dash placeholder Breathe leaves in rendered API documentation.

Doxygen applies typographic substitution to its input, so a ``--`` written in
a doc comment reaches the XML as an ``<ndash/>`` element rather than as text.
Breathe (4.36) turns that element into the *literal string* ``&#8212;`` and
uses it as a marker while rendering block quotes, where a trailing dash means
"attribution" -- see ``visit_docblockquote`` in breathe/renderer. It strips
the marker there, and only there: in ordinary prose the placeholder survives
into the doctree, gets HTML-escaped on the way out, and the published page
shows a bare ``&#8212;`` in the middle of a sentence.

Hemerion's doc comments use ``--`` as a parenthetical dash throughout, so
without this every API page would carry dozens of them. Replacing the marker
after Breathe has finished with it is safe: any occurrence still present at
``doctree-read`` time is one Breathe did not consume.
"""

from __future__ import annotations

from docutils import nodes

#: The placeholder Breathe substitutes for Doxygen's <ndash/> and <mdash/>.
PLACEHOLDER = "&#8212;"

#: What to show instead. Doxygen maps ``--`` to an en dash and ``---`` to an
#: em dash, but Breathe collapses both onto one placeholder, so the
#: distinction is not recoverable here. Hemerion's prose uses ``--`` as a
#: parenthetical break, which is an em dash's job.
REPLACEMENT = "—"


def _repair_dashes(app, doctree):
    for node in list(doctree.findall(nodes.Text)):
        text = node.astext()
        if PLACEHOLDER in text:
            node.parent.replace(node, nodes.Text(text.replace(PLACEHOLDER, REPLACEMENT)))


def setup(app):
    app.connect("doctree-read", _repair_dashes)
    return {
        "version": "1.0",
        "parallel_read_safe": True,
        "parallel_write_safe": True,
    }
