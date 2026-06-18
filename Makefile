# OpenPuck developer convenience targets.

# Pin to clang-format 18 to match CI; output differs between versions, so a
# mismatch would make CI reject locally-formatted code. Resolve the binary in
# order of preference: the apt-style versioned name (CI / Linux), the Homebrew
# llvm@18 keg (macOS), then a bare clang-format as a last resort. Override with
# `make format CLANG_FORMAT=/path/to/clang-format` if yours lives elsewhere.
CLANG_FORMAT ?= $(shell command -v clang-format-18 \
	|| ([ -x /opt/homebrew/opt/llvm@18/bin/clang-format ] && echo /opt/homebrew/opt/llvm@18/bin/clang-format) \
	|| ([ -x /usr/local/opt/llvm@18/bin/clang-format ] && echo /usr/local/opt/llvm@18/bin/clang-format) \
	|| command -v clang-format)

# All hand-written C/C++ sources. Generated headers (git_version.h) are
# gitignored and excluded here so formatting never touches them.
FORMAT_FILES := $(shell find OpenPuck puck_sniffer pairtui \
	\( -name '*.c' -o -name '*.cpp' -o -name '*.h' -o -name '*.hpp' -o -name '*.ino' \) \
	-not -name 'git_version.h')

.PHONY: format format-check lint check

## Reformat all C/C++ sources in place using the Linux kernel style.
format:
	$(CLANG_FORMAT) -i $(FORMAT_FILES)

## Verify all C/C++ sources are formatted; exit non-zero if any differ.
## Used by CI to reject unformatted code.
format-check:
	$(CLANG_FORMAT) --dry-run --Werror $(FORMAT_FILES)

## Style rules clang-format can't enforce. Currently: a long trailing //
## comment must be moved onto its own line above the statement (clang-format
## won't relocate it, so it would otherwise split the code or run past 80).
lint:
	python3 tools/check-trailing-comments.py $(FORMAT_FILES)

## Everything CI gates on.
check: format-check lint
