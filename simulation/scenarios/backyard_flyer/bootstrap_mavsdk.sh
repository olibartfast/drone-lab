#!/usr/bin/env bash
set -euo pipefail

if [[ "$(uname -s)" != "Linux" || "$(uname -m)" != "x86_64" ]]; then
  echo "MAVSDK bootstrap supports Linux x86_64 only" >&2
  exit 2
fi

VERSION="3.17.1"
SHA256="c9bda21698e21e682f176db3c68bbcf39f9e42853f5952cb4aa8b4efb694d328"
URL="https://github.com/mavlink/MAVSDK/releases/download/v${VERSION}/libmavsdk-dev_${VERSION}_ubuntu24.04_amd64.deb"
DESTINATION="${1:?destination directory required}"
ARCHIVE="${DESTINATION}/libmavsdk-dev.deb"
ROOT="${DESTINATION}/root"

mkdir -p "${DESTINATION}"
if [[ -f "${ARCHIVE}" ]] && ! printf '%s  %s\n' "${SHA256}" "${ARCHIVE}" | sha256sum --check --status; then
  mv "${ARCHIVE}" "${ARCHIVE}.invalid"
fi
if [[ ! -f "${ARCHIVE}" ]]; then
  temporary="${ARCHIVE}.download"
  wget --quiet --show-progress --output-document "${temporary}" "${URL}"
  printf '%s  %s\n' "${SHA256}" "${temporary}" | sha256sum --check --status
  mv "${temporary}" "${ARCHIVE}"
fi
if [[ ! -f "${ROOT}/usr/lib/cmake/MAVSDK/MAVSDKConfig.cmake" ]]; then
  mkdir -p "${ROOT}"
  dpkg-deb --extract "${ARCHIVE}" "${ROOT}"
fi
ln -sfn "libmavsdk.so.${VERSION}" "${ROOT}/usr/lib/libmavsdk.so.3"
ln -sfn "libmavsdk.so.3" "${ROOT}/usr/lib/libmavsdk.so"
printf '%s\n' "${ROOT}/usr"
