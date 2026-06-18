#!/usr/bin/env python3
"""Reject trailing // comments that push a line past the 80-column limit.

clang-format (kernel style, ReflowComments: false) cannot move a trailing
comment onto its own line, so when a short statement carries a long trailing
comment it either splits the statement or leaves the line over-length. Our
convention is to instead put the comment on its own line(s) ABOVE the
statement, separated by a blank line. This check enforces that: a line wider
than the limit whose overflow comes from a trailing // comment is an error.

Standalone comment lines (// in column 0..) are left alone -- the kernel style
does not reflow comments, and those are not what triggers the bad split.
"""
import sys

LIMIT = 80
TAB = 8


def visual_width(s):
    """Width of a line with tabs expanded to the next multiple of TAB."""
    col = 0
    for ch in s:
        if ch == "\t":
            col += TAB - (col % TAB)
        else:
            col += 1
    return col


def trailing_comment_index(line):
    """Char index of the first real `//` comment, or None.

    Skips `//` that appears inside a string or char literal so that e.g.
    "http://example" is not mistaken for a comment.
    """
    quote = None
    i, n = 0, len(line)
    while i < n:
        ch = line[i]
        if quote:
            if ch == "\\":
                i += 2
                continue
            if ch == quote:
                quote = None
            i += 1
            continue
        if ch in "\"'":
            quote = ch
            i += 1
            continue
        if ch == "/" and i + 1 < n and line[i + 1] == "/":
            return i
        i += 1
    return None


def violations(path):
    out = []
    with open(path, errors="replace") as f:
        for lineno, raw in enumerate(f, 1):
            line = raw.rstrip("\n")
            if visual_width(line) <= LIMIT:
                continue
            ci = trailing_comment_index(line)
            if ci is None:
                continue  # over-length, but not because of a comment
            if line[:ci].strip() == "":
                continue  # standalone comment line -> allowed
            out.append((lineno, visual_width(line)))
    return out


def main(argv):
    files = argv[1:]
    bad = 0
    for path in files:
        for lineno, width in violations(path):
            bad += 1
            print(f"{path}:{lineno}: trailing comment exceeds "
                  f"{LIMIT} columns (line is {width}); "
                  f"move the comment onto its own line above the statement")
    if bad:
        print(f"\n{bad} trailing-comment violation(s). "
              f"See tools/check-trailing-comments.py.", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
