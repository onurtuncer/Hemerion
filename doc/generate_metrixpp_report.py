# ------------------------------------------------------------------------------
# Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
# ------------------------------------------------------------------------------
"""Render the Metrix++ cyclomatic complexity report as a Sphinx page.

Called from doc/conf.py, so every documentation build -- the CMake docs
target, a bare sphinx-build, and .github/workflows/deploy-docs.yml -- measures
the current sources and publishes the numbers alongside the rest of the docs.
The trees analysed and the complexity limit mirror
.github/workflows/metrixpp.yml, which uploads the same data as a raw
build artifact.

Metrix++ is informational here. The hard CI gate for AV Rule 3 is clang-tidy's
readability-function-size (CyclomaticComplexityThreshold=20, see .clang-tidy);
Metrix++ counts differently and is kept as a cross-check.

If Metrix++ is not installed, or a Metrix++ run fails, a placeholder page is
written instead so that the documentation build never breaks over a report.
"""

import csv
import os
import shutil
import subprocess
import sys

# Same trees .github/workflows/metrixpp.yml points `metrix++ collect` at.
ANALYSIS_TREES = ("modules", "apps", "bsp", "sim", "tests", "examples")

# Matches --max-limit in the workflow and CyclomaticComplexityThreshold in
# .clang-tidy.
COMPLEXITY_LIMIT = 20

# Functions at or above this are listed individually; below it there are
# several hundred and the distribution table says everything useful.
LISTING_THRESHOLD = 10

# Upper bound (inclusive) of each distribution bucket, None meaning "no bound".
DISTRIBUTION_BUCKETS = (
    ("0", 0),
    ("1-2", 2),
    ("3-5", 5),
    ("6-10", 10),
    ("11-20", 20),
    ("over 20", None),
)

_HEADER = """.. ------------------------------------------------------------------------------
.. Project: Hemerion
.. Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
..
.. SPDX-License-Identifier: GPL-3.0-only
.. License-Filename: LICENSE
.. ------------------------------------------------------------------------------

.. DO NOT EDIT -- written by doc/generate_metrixpp_report.py on every docs build.

.. _metrixpp_report:

Cyclomatic complexity report
============================
"""


def _metrixpp_command():
    """Return the argv prefix that runs Metrix++, or None if it is missing."""
    executable = shutil.which("metrix++")
    if executable is not None:
        return [executable]

    # Installed as a library but with its console script outside PATH, which is
    # the normal situation for a Windows user-site pip install.
    try:
        import metrixpp  # noqa: F401
    except ImportError:
        return None
    return [sys.executable, "-c", "from metrixpp.metrixpp import start; start()"]


def _metrixpp_version():
    try:
        from importlib.metadata import version

        return version("metrixpp")
    except Exception:  # pragma: no cover - version is cosmetic
        return None


def _run(command, cwd):
    """Run Metrix++ and return its stdout, or None if the run failed.

    Metrix++ logs to stderr and writes report data to stdout, so stdout is
    usable as-is.
    """
    try:
        completed = subprocess.run(
            command,
            cwd=cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
        )
    except OSError as exc:
        print("Metrix++ report: could not run %s (%s)" % (command[0], exc))
        return None

    if completed.returncode != 0:
        print("Metrix++ report: %s exited %d" % (command[1], completed.returncode))
        print(completed.stderr.strip()[-2000:])
        return None
    return completed.stdout


def _collect(repo_root, work_dir):
    """Measure the sources, returning the per-function rows Metrix++ exports."""
    command = _metrixpp_command()
    if command is None:
        print("Metrix++ report: metrixpp is not installed, writing a placeholder")
        return None

    trees = [t for t in ANALYSIS_TREES if os.path.isdir(os.path.join(repo_root, t))]
    if not trees:
        return None

    os.makedirs(work_dir, exist_ok=True)
    db_file = os.path.join(work_dir, "metrixpp.db")
    # Metrix++ updates an existing database incrementally; start clean so the
    # published page never carries rows for files that have since been deleted.
    if os.path.exists(db_file):
        os.remove(db_file)

    collected = _run(
        command
        + [
            "collect",
            "--db-file=" + db_file,
            "--std.code.complexity.cyclomatic",
            "--std.code.lines.code",
            "--",
        ]
        + list(trees),
        cwd=repo_root,
    )
    if collected is None:
        return None

    exported = _run(command + ["export", "--db-file=" + db_file], cwd=repo_root)
    if exported is None:
        return None

    functions = []
    for row in csv.DictReader(exported.splitlines()):
        # Only function regions carry a cyclomatic number; files, namespaces
        # and class bodies come back with the column empty.
        complexity = row.get("std.code.complexity:cyclomatic")
        if row.get("type") != "function" or not complexity:
            continue
        path = row["file"]
        if path.startswith("./"):
            path = path[2:]
        functions.append(
            {
                "path": path,
                "name": row["region"],
                "line": row["line start"],
                "complexity": int(complexity),
                "lines": int(row.get("std.code.lines:code") or 0),
            }
        )
    return functions


def _subsystem(path):
    """Group key for a source path: two levels deep, e.g. ``modules/sensors``."""
    parts = path.split("/")
    if len(parts) > 2:
        return "/".join(parts[:2])
    return parts[0]


def _list_table(headers, rows, widths):
    lines = [".. list-table::", "   :header-rows: 1"]
    lines.append("   :widths: " + " ".join(str(w) for w in widths))
    lines.append("")
    for record in [headers] + rows:
        for index, cell in enumerate(record):
            lines.append(("   * - " if index == 0 else "     - ") + str(cell))
    lines.append("")
    return lines


def _preamble(version):
    tool = "Metrix++" if version is None else "Metrix++ %s" % version
    trees = ["``%s/``" % tree for tree in ANALYSIS_TREES]
    trees = ", ".join(trees[:-1]) + " and " + trees[-1]
    return [
        "",
        "%s measures every function in %s each time" % (tool, trees),
        "this documentation is built, so the numbers below describe the sources as",
        "they are published.",
        "",
        "This report is informational. The hard CI gate for AV Rule 3 is clang-tidy's",
        "``readability-function-size`` with ``CyclomaticComplexityThreshold=%d`` (see" % COMPLEXITY_LIMIT,
        "``.clang-tidy``); Metrix++ counts branches differently and is kept as a",
        "cross-check at the same threshold.",
        "",
    ]


def _render(functions, version):
    lines = [_HEADER]
    lines.extend(_preamble(version))

    over_limit = [f for f in functions if f["complexity"] > COMPLEXITY_LIMIT]
    total = sum(f["complexity"] for f in functions)
    worst = max(functions, key=lambda f: f["complexity"])

    lines.append("Summary")
    lines.append("-------")
    lines.append("")
    lines.extend(
        _list_table(
            ("Measure", "Value"),
            [
                ("Functions measured", len(functions)),
                ("Mean cyclomatic complexity", "%.2f" % (float(total) / len(functions))),
                (
                    "Highest cyclomatic complexity",
                    "%d (``%s``, %s)" % (worst["complexity"], worst["name"], worst["path"]),
                ),
                ("Functions over the limit of %d" % COMPLEXITY_LIMIT, len(over_limit)),
                ("Total code lines measured", sum(f["lines"] for f in functions)),
            ],
            widths=(45, 55),
        )
    )

    lines.append("Distribution")
    lines.append("------------")
    lines.append("")
    rows = []
    lower = 0
    for label, upper in DISTRIBUTION_BUCKETS:
        if upper is None:
            count = len([f for f in functions if f["complexity"] >= lower])
        else:
            count = len([f for f in functions if lower <= f["complexity"] <= upper])
            lower = upper + 1
        rows.append((label, count, "%.1f %%" % (100.0 * count / len(functions))))
    lines.extend(
        _list_table(("Cyclomatic complexity", "Functions", "Share"), rows, widths=(40, 30, 30))
    )

    lines.append("Most complex functions")
    lines.append("----------------------")
    lines.append("")
    listed = sorted(
        [f for f in functions if f["complexity"] >= LISTING_THRESHOLD],
        key=lambda f: (-f["complexity"], f["path"], f["name"]),
    )
    if listed:
        lines.append(
            "Every function scoring %d or above, worst first. Rows over the limit of"
            % LISTING_THRESHOLD
        )
        lines.append("%d are the ones worth splitting up." % COMPLEXITY_LIMIT)
        lines.append("")
        lines.extend(
            _list_table(
                ("Complexity", "Code lines", "Function", "Location"),
                [
                    (
                        f["complexity"],
                        f["lines"],
                        "``%s``" % f["name"],
                        "``%s:%s``" % (f["path"], f["line"]),
                    )
                    for f in listed
                ],
                widths=(12, 12, 26, 50),
            )
        )
    else:
        lines.append(
            "No function scores %d or above." % LISTING_THRESHOLD,
        )
        lines.append("")

    lines.append("By subsystem")
    lines.append("------------")
    lines.append("")
    groups = {}
    for function in functions:
        groups.setdefault(_subsystem(function["path"]), []).append(function)
    rows = []
    for name in sorted(groups):
        members = groups[name]
        rows.append(
            (
                "``%s``" % name,
                len(members),
                "%.2f" % (float(sum(m["complexity"] for m in members)) / len(members)),
                max(m["complexity"] for m in members),
                len([m for m in members if m["complexity"] > COMPLEXITY_LIMIT]),
            )
        )
    lines.extend(
        _list_table(
            ("Subsystem", "Functions", "Mean", "Max", "Over limit"),
            rows,
            widths=(40, 15, 15, 15, 15),
        )
    )

    return "\n".join(lines) + "\n"


def _render_placeholder():
    lines = [_HEADER]
    lines.extend(_preamble(None))
    lines.append(".. warning::")
    lines.append("")
    lines.append("   This build of the documentation was made without Metrix++, so the")
    lines.append("   report is empty. Install it with ``pip install metrixpp`` (it is")
    lines.append("   listed in ``doc/requirements.txt``) and build the docs again, or read")
    lines.append("   the ``metrixpp-report`` artifact of the Metrix++ workflow run.")
    lines.append("")
    return "\n".join(lines) + "\n"


def write_report(repo_root, out_path, work_dir):
    """Write the report page, falling back to a placeholder page.

    Returns True when the page holds real measurements.
    """
    functions = _collect(repo_root, work_dir)
    if functions:
        page = _render(functions, _metrixpp_version())
    else:
        page = _render_placeholder()

    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(out_path, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(page)
    print(
        "Metrix++ report: wrote %s (%s)"
        % (out_path, "%d functions" % len(functions) if functions else "placeholder")
    )
    return bool(functions)


if __name__ == "__main__":
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    write_report(
        repo_root=root,
        out_path=os.path.join(root, "doc", "generated", "metrixpp_report.rst"),
        work_dir=os.path.join(root, "build", "metrixpp"),
    )
