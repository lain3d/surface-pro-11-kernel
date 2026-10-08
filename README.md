<!-- SPDX-License-Identifier: GPL-2.0-only -->
# Surface Pro 11 experimental Linux kernel

> **Unmaintained research; experimental hardware changes; no support commitment.**
> Released for others to inspect, reuse, and continue. This is not a supported
> kernel distribution. Review changes before booting them; experiments involving
> firmware, regulators, or USB-C can disconnect a root disk or leave hardware wedged.

Target hardware: Microsoft Surface Pro 11, Snapdragon X Elite X1E80100, OLED,
Wi-Fi-only SKU. Results on this unit do not establish support for other Surface
models or configurations.

## Start with the research

[Public research and handoff](https://github.com/lain3d/surface-pro-11-research)
contains the measurements, reverse-engineering notes, board wiring, runtime
scripts, and userspace patches. In particular:

- [Camera state and measured fixes](https://github.com/lain3d/surface-pro-11-research/blob/main/design/camera-state-20260807.md)
- [Upstream handoff and remaining limits](https://github.com/lain3d/surface-pro-11-research/blob/main/design/upstream.md)
- [libcamera and FFmpeg patches; libaperture findings](https://github.com/lain3d/surface-pro-11-research/tree/main/patches)

## Default source tree

Start with [`main`](https://github.com/lain3d/surface-pro-11-kernel/tree/main):
it contains the latest released front-facing IMX681 driver, C-PHY/CAMSS changes,
and measured sensor-mode fixes.

`main` was created from `debug/camss-cphy` at
`4f548f23d5e56746650f1f1840f93329d884b937`, with this release README added.
No kernel code was changed or additional experiment branches merged for this
cutover. It preserves the source tree's existing boot-payload, rear-sensor
device-tree, and Type-C/ADSP ancestry; this does not newly qualify those features.
This is the latest preserved camera research, not a freshly tested all-features
integration or an upstream-ready series.

- [Front-camera driver](drivers/media/i2c/imx681.c)
- [CAMSS receiver/PHY source](drivers/media/platform/qcom/camss)
- [Board device trees](arch/arm64/boot/dts/qcom)

### Historical branches

These remain available for provenance and comparison, not as required branch
switches to access the latest camera implementation. Some changes are already
ancestors of `main`; others are divergent experiments. No blanket claim that
they are missing from, or compatible with, `main` is intended.

| Branch | Historical scope |
|---|---|
| [`sp11`](https://github.com/lain3d/surface-pro-11-kernel/tree/sp11) | Early bring-up base and initial publication notice; no longer the default |
| [`debug/camss-cphy`](https://github.com/lain3d/surface-pro-11-kernel/tree/debug/camss-cphy) | Preserved original camera tip used as the source for `main` |
| [`integration/camera`](https://github.com/lain3d/surface-pro-11-kernel/tree/integration/camera) | Earlier alternative camera integration and board device-tree wiring |
| [`camera/imx681`](https://github.com/lain3d/surface-pro-11-kernel/tree/camera/imx681) | Earlier IMX681 implementation; use `main` for the latest released source |
| [`camera/ov13858-dt`](https://github.com/lain3d/surface-pro-11-kernel/tree/camera/ov13858-dt) | Rear-sensor device-tree probing and power-control work, inherited by `main`; not proof of rear-camera capture |
| [`camera/denali-pm8010`](https://github.com/lain3d/surface-pro-11-kernel/tree/camera/denali-pm8010) | Separate camera PMIC/regulator experiment |
| [`usb4/platform-nhi`](https://github.com/lain3d/surface-pro-11-kernel/tree/usb4/platform-nhi) | Separate platform-enumerated USB4 NHI experiment; router binding is not proof of working USB4/eGPU support |
| [`wip/sp11-typec-adsp-fixes`](https://github.com/lain3d/surface-pro-11-kernel/tree/wip/sp11-typec-adsp-fixes) | USB-C/retimer/PHY experiments already in `main`'s ancestry, not newly qualified for external-root boot |
| [`audio/dmic-clock`](https://github.com/lain3d/surface-pro-11-kernel/tree/audio/dmic-clock) | Separate 2.4 MHz DMIC clock branch for comparison |
| [`wifi/wcn7850-perst`](https://github.com/lain3d/surface-pro-11-kernel/tree/wifi/wcn7850-perst) | Separate WCN7850 PCIe reset/wake branch for comparison |

The front camera was exercised during the original research, including desktop
capture and sensor-mode timing. That does not imply that every mode, the rear or
IR camera, a complete ISP/image-quality pipeline, or a modern upstream kernel is
supported. Historical validation is recorded in the research; no hardware
revalidation was performed for this publication.

## Provenance and license

The base tree is a squashed import of
[`jglathe/linux_ms_dev_kit`](https://github.com/jglathe/linux_ms_dev_kit) at
`bd336e2e0937cb5b2dd258e784b3b6267c653167` on `jg/ubuntu-qcom-x1e-7.1.y`, with
subsequent Surface-specific and imported patches. Existing authorship and
attribution are preserved.

See [COPYING](COPYING) and [LICENSES](LICENSES) for the kernel's existing licenses.
This release does **not** relicense the kernel as MIT. Camera register-table
provenance is documented in the driver and research; no redistribution permission
for proprietary Windows drivers or firmware is granted. Those binaries are not
provided by this release.

See [the upstream README](README) for general Linux kernel documentation.

## Publication notes

Published on 2026-10-07 for continuation by others. The subsequent default-branch
cutover exposes the preserved latest camera tree on `main` and changes only
release documentation relative to its source commit. Existing branch history,
author email addresses, local hostnames in authorship, and historical session
metadata remain intact. No kernel rebuild or hardware revalidation was performed
for publication or the default-branch cutover.
