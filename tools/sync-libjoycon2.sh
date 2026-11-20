#!/usr/bin/env bash
# Sync a release snapshot of libjoycon2/src into OpenPuck/src/libjoycon2/ and
# regenerate NOTICE.md with the pinned upstream ref and commit SHA.
#
# Usage:
#   ./tools/sync-libjoycon2.sh [path-or-git-url] [ref]
# Examples:
#   ./tools/sync-libjoycon2.sh ../libjoycon2 v0.1.0
#   ./tools/sync-libjoycon2.sh https://github.com/bmmund/libjoycon2.git v0.1.0
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC_ARG="${1:-https://github.com/bmmund/libjoycon2.git}"
REF_ARG="${2:-HEAD}"
DEST="${REPO_ROOT}/OpenPuck/src/libjoycon2"
UPSTREAM_URL="https://github.com/bmmund/libjoycon2"

TMPDIR=""
cleanup() {
	if [ -n "${TMPDIR}" ] && [ -d "${TMPDIR}" ]; then
		rm -rf "${TMPDIR}"
	fi
}
trap cleanup EXIT

if [ -d "${SRC_ARG}/.git" ]; then
	SRC_DIR="${SRC_ARG}"
	SHA="$(git -C "${SRC_DIR}" rev-parse "${REF_ARG}^{commit}")"
	TAG="$(git -C "${SRC_DIR}" describe --tags --exact-match "${SHA}" 2>/dev/null || echo "${REF_ARG}")"
	TMPDIR="$(mktemp -d)"
	git -C "${SRC_DIR}" archive "${SHA}" | tar -x -C "${TMPDIR}"
	WORK_DIR="${TMPDIR}"
else
	TMPDIR="$(mktemp -d)"
	git clone --depth 1 --branch "${REF_ARG}" "${SRC_ARG}" "${TMPDIR}" >/dev/null
	SHA="$(git -C "${TMPDIR}" rev-parse HEAD)"
	TAG="${REF_ARG}"
	WORK_DIR="${TMPDIR}"
fi

rm -rf "${DEST}"
mkdir -p "${DEST}"
cp "${WORK_DIR}"/src/*.c "${WORK_DIR}"/src/*.h "${DEST}/"
cp "${WORK_DIR}/LICENSE" "${DEST}/LICENSE"

TODAY="$(date -u +%Y-%m-%d)"
cat > "${DEST}/NOTICE.md" <<EOF
# Vendored libjoycon2

Upstream: ${UPSTREAM_URL}

Pinned release: **${TAG}**
Pinned git revision: **${SHA}**

Vendored unmodified into OpenPuck on **${TODAY}** via \`tools/sync-libjoycon2.sh\`.
Only \`src/*.c\`, \`src/*.h\`, and [LICENSE](LICENSE) are vendored; upstream tests,
examples, and protocol docs live in the upstream repository.

## Licensing and attribution

libjoycon2 is licensed under the **MIT License**. See [LICENSE](LICENSE).

- Copyright (c) 2026 Braden Mund

Protocol notes and Switch 2 Joy-Con 2 BLE / GATT / report documentation live in
[${UPSTREAM_URL}/blob/${TAG}/docs/protocol-map.md](${UPSTREAM_URL}/blob/${TAG}/docs/protocol-map.md).

## Integration contract

- OpenPuck instantiates a single static \`jc2_device_t\` and injects LittleFS
  bond persistence (\`/jc2_bonds.bin\`) via \`jc2_callbacks_t\`.
- \`jc2_ll_nrf52.c\` owns \`RADIO_IRQHandler\`, \`TIMER0_IRQHandler\`, and
  \`SWI0_EGU0_IRQHandler\` while Joy-Con 2 BLE mode is active and exposes
  \`jc2_ll_acquire_gap()\` / \`jc2_ll_remaining_gap_us()\` / \`jc2_ll_release_gap()\`
  for 2.4 GHz Steam Controller ESB coexistence.
- To update to a newer upstream release, run:
  \`./tools/sync-libjoycon2.sh ${UPSTREAM_URL}.git <tag>\`
EOF

echo "Synced libjoycon2 ${TAG} (${SHA}) -> ${DEST}"
