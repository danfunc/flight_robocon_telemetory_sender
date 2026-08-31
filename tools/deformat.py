#!/usr/bin/env python3
"""
deformat.py - Revert format-only diffs (e.g. clang-format noise) while preserving semantic changes.
Designed for C++ codebases (.cpp / .hpp) with Japanese comments.
"""

import argparse
from dataclasses import dataclass
from enum import Enum
import os
from pathlib import Path
import re
import subprocess
import sys
from typing import List, Optional, Tuple


CPP_TOKEN_REGEX = re.compile(
    r"""
    (?P<RAW_STR>R"(?P<delim>[a-zA-Z0-9_]{0,16})\(.*?\)(?P=delim)")
    |(?P<STR>"(?:\\.|[^"\\])*")
    |(?P<CHAR>'(?:\\.|[^'\\])*')
    |(?P<LINE_COMMENT>//[^\n]*)
    |(?P<BLOCK_COMMENT>/\*[\s\S]*?\*/)
    |(?P<NUMBER>(?:0[xX][0-9a-fA-F']+(?:\.[0-9a-fA-F']*)?(?:[pP][+-]?[0-9']+)?|[0-9']+(?:\.[0-9']*)?(?:[eE][+-]?[0-9']+)?)[a-zA-Z]*)
    |(?P<IDENT>[a-zA-Z_][a-zA-Z0-9_]*)
    |(?P<OP>::|->\*|->|\+\+|--|<<=|>>=|<=|>=|==|!=|&&|\|\||\+=|-=|\*=|/=|%=|&=|\|=|\^=|<<|>>|<=>|\#\#|[{}\[\]();,?:.~!%^&*+\-/|<>=#\\])
    |(?P<WS>\s+)
    |(?P<MISC>.)
    """,
    re.VERBOSE | re.DOTALL,
)


def _clean_comment_text(comment_str: str) -> str:
    if comment_str.startswith("//"):
        text = comment_str[2:]
    elif comment_str.startswith("/*") and comment_str.endswith("*/"):
        text = comment_str[2:-2]
        lines = [re.sub(r"^\s*\*\s?", "", l) for l in text.splitlines()]
        text = "".join(lines)
    else:
        text = comment_str
    return re.sub(r"\s+", "", text)


def tokenize_and_normalize(text: str) -> List[Tuple[str, str]]:
    tokens: List[Tuple[str, str]] = []
    pending_comments: List[str] = []

    def flush_comments():
        if pending_comments:
            merged = "".join(pending_comments)
            if merged:
                tokens.append(("COMMENT", merged))
            pending_comments.clear()

    for m in CPP_TOKEN_REGEX.finditer(text):
        kind = m.lastgroup
        val = m.group()

        if kind in ("LINE_COMMENT", "BLOCK_COMMENT"):
            cleaned = _clean_comment_text(val)
            if cleaned:
                pending_comments.append(cleaned)
        elif kind == "WS":
            continue
        else:
            flush_comments()
            tokens.append((kind or "MISC", val))

    flush_comments()
    return tokens


class HunkCategory(Enum):
    FORMAT_ONLY = "整形のみ"
    SEMANTIC = "意味あり"
    MIXED = "混在"


@dataclass
class Hunk:
    old_start: int
    old_count: int
    new_start: int
    new_count: int
    lines: List[Tuple[str, str]]

    @property
    def deleted_lines(self) -> List[str]:
        return [line for op, line in self.lines if op == "-"]

    @property
    def added_lines(self) -> List[str]:
        return [line for op, line in self.lines if op == "+"]

    @property
    def worktree_lines(self) -> List[str]:
        return [line for op, line in self.lines if op in (" ", "+")]

    @property
    def head_lines(self) -> List[str]:
        return [line for op, line in self.lines if op in (" ", "-")]


@dataclass
class FileDiff:
    old_path: str
    new_path: str
    hunks: List[Hunk]


HUNK_HEADER_RE = re.compile(r"^@@ -(\d+)(?:,(\d+))? \+(\d+)(?:,(\d+))? @@")


def parse_git_diff(diff_text: str) -> List[FileDiff]:
    files: List[FileDiff] = []
    current_file: Optional[FileDiff] = None
    current_hunk: Optional[Hunk] = None

    for line in diff_text.splitlines():
        if line.startswith("diff --git "):
            current_hunk = None
            parts = line.split(" ")
            old_p = parts[2][2:] if parts[2].startswith("a/") else parts[2]
            new_p = parts[3][2:] if parts[3].startswith("b/") else parts[3]
            current_file = FileDiff(old_path=old_p, new_path=new_p, hunks=[])
            files.append(current_file)
            continue

        if current_file is None:
            continue

        if line.startswith("@@ "):
            m = HUNK_HEADER_RE.match(line)
            if m:
                old_start = int(m.group(1))
                old_count = int(m.group(2)) if m.group(2) is not None else 1
                new_start = int(m.group(3))
                new_count = int(m.group(4)) if m.group(4) is not None else 1
                current_hunk = Hunk(
                    old_start=old_start,
                    old_count=old_count,
                    new_start=new_start,
                    new_count=new_count,
                    lines=[],
                )
                current_file.hunks.append(current_hunk)
            continue

        if current_hunk is not None:
            if line.startswith(("+", "-", " ")):
                current_hunk.lines.append((line[0], line[1:]))

    return files


def classify_hunk(hunk: Hunk) -> HunkCategory:
    del_lines = hunk.deleted_lines
    add_lines = hunk.added_lines

    if not del_lines or not add_lines:
        return HunkCategory.SEMANTIC

    del_text = "\n".join(del_lines)
    add_text = "\n".join(add_lines)

    del_tokens = tokenize_and_normalize(del_text)
    add_tokens = tokenize_and_normalize(add_text)

    if del_tokens == add_tokens:
        return HunkCategory.FORMAT_ONLY

    del_stripped = {re.sub(r"\s+", "", l) for l in del_lines if re.sub(r"\s+", "", l)}
    add_stripped = {re.sub(r"\s+", "", l) for l in add_lines if re.sub(r"\s+", "", l)}
    has_line_format_overlap = bool(del_stripped & add_stripped)

    del_token_set = set(del_tokens)
    add_token_set = set(add_tokens)
    has_token_overlap = bool(del_token_set & add_token_set)

    if has_line_format_overlap or (has_token_overlap and len(del_tokens) > 1):
        return HunkCategory.MIXED

    return HunkCategory.SEMANTIC


def revert_format_hunks(
    repo_root: Path, file_diff: FileDiff, classifications: List[HunkCategory]
) -> Tuple[bool, str]:
    target_path = repo_root / file_diff.new_path
    if not target_path.exists():
        return False, f"File does not exist: {target_path}"

    format_items = [
        (hunk, cat)
        for hunk, cat in zip(file_diff.hunks, classifications)
        if cat == HunkCategory.FORMAT_ONLY
    ]
    if not format_items:
        return True, "No format-only hunks to revert."

    try:
        with open(target_path, "r", encoding="utf-8", errors="surrogateescape") as f:
            file_lines = f.read().splitlines(keepends=True)
    except Exception as e:
        return False, f"Failed to read {target_path}: {e}"

    format_items.sort(key=lambda item: item[0].new_start, reverse=True)

    for hunk, _ in format_items:
        start_idx = hunk.new_start - 1
        count = hunk.new_count

        current_slice = [l.rstrip("\r\n") for l in file_lines[start_idx : start_idx + count]]
        expected_slice = hunk.worktree_lines

        if current_slice != expected_slice:
            return False, (
                f"Safety check failed at line {hunk.new_start} in {file_diff.new_path}. "
                f"File content diverged from diff expectation."
            )

        newline = "\r\n" if file_lines and file_lines[0].endswith("\r\n") else "\n"
        replacement_lines = [l + newline for l in hunk.head_lines]

        file_lines[start_idx : start_idx + count] = replacement_lines

    temp_path = target_path.with_suffix(target_path.suffix + ".deformat_tmp")
    try:
        with open(temp_path, "w", encoding="utf-8", errors="surrogateescape") as f:
            f.writelines(file_lines)
        os.replace(temp_path, target_path)
    except Exception as e:
        if temp_path.exists():
            temp_path.unlink()
        return False, f"Failed to write updated file: {e}"

    return True, f"Successfully reverted {len(format_items)} format hunks."


def main():
    parser = argparse.ArgumentParser(
        description="Revert format-only diffs to HEAD while preserving semantic changes."
    )
    parser.add_argument("--repo", type=Path, default=Path.cwd())
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--paths", nargs="*", default=[])
    args = parser.parse_args()

    repo_root = args.repo.resolve()
    if not (repo_root / ".git").exists():
        print(f"Error: {repo_root} is not a git repository.", file=sys.stderr)
        sys.exit(1)

    cmd = ["git", "diff", "--no-color", "--no-ext-diff", "-U3", "HEAD"]
    if args.paths:
        cmd.append("--")
        cmd.extend(args.paths)

    try:
        res = subprocess.run(cmd, cwd=repo_root, capture_output=True, text=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error executing git diff: {e.stderr}", file=sys.stderr)
        sys.exit(1)

    diff_text = res.stdout
    if not diff_text.strip():
        print("Worktree is clean. Nothing to do.")
        return

    file_diffs = parse_git_diff(diff_text)
    if not file_diffs:
        print("No file diffs found.")
        return

    mode_str = "DRY-RUN (no files modified)" if args.dry_run else "APPLYING CHANGES"
    print(f"Repository: {repo_root}")
    print(f"Mode: {mode_str}\n")

    total_format = 0
    total_semantic = 0
    total_mixed = 0

    for fdiff in file_diffs:
        if not fdiff.hunks:
            continue

        classifications: List[HunkCategory] = []
        mixed_details: List[str] = []

        for hunk in fdiff.hunks:
            cat = classify_hunk(hunk)
            classifications.append(cat)
            if cat == HunkCategory.MIXED:
                mixed_details.append(
                    f"    - Hunk at HEAD L{hunk.old_start} -> Worktree L{hunk.new_start} (+{hunk.new_count} lines)"
                )

        cnt_format = classifications.count(HunkCategory.FORMAT_ONLY)
        cnt_semantic = classifications.count(HunkCategory.SEMANTIC)
        cnt_mixed = classifications.count(HunkCategory.MIXED)

        total_format += cnt_format
        total_semantic += cnt_semantic
        total_mixed += cnt_mixed

        print(f"{fdiff.new_path}: 整形のみ {cnt_format} / 意味あり {cnt_semantic} / 混在 {cnt_mixed}")
        if mixed_details:
            print("  [混在ハンク一覧 (要手動レビュー)]:")
            for detail in mixed_details:
                print(detail)

        if not args.dry_run and cnt_format > 0:
            ok, msg = revert_format_hunks(repo_root, fdiff, classifications)
            if not ok:
                print(f"  [ERROR] {msg}", file=sys.stderr)

    print("\n" + "=" * 50)
    print("【全体サマリ】")
    print(f"  - 整形のみ (Reverted) : {total_format}")
    print(f"  - 意味あり (Preserved): {total_semantic}")
    print(f"  - 混在     (Skipped)  : {total_mixed}")
    print("=" * 50)


if __name__ == "__main__":
    main()
