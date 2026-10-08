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
- [libcamera, FFmpeg, and libaperture patches](https://github.com/lain3d/surface-pro-11-research/tree/main/patches)

## Branch map

The default `sp11` branch is the early bring-up base, **not** the latest camera
implementation. Later experiments remain on their original branches:

| Branch | Scope |
|---|---|
| [`debug/camss-cphy`](https://github.com/lain3d/surface-pro-11-kernel/tree/debug/camss-cphy) | Latest front-facing IMX681 driver, C-PHY/CAMSS changes, and measured sensor-mode fixes; experimental/debug code, not an upstream-ready series |
| [`integration/camera`](https://github.com/lain3d/surface-pro-11-kernel/tree/integration/camera) | Earlier camera integration and board device-tree wiring |
| [`camera/imx681`](https://github.com/lain3d/surface-pro-11-kernel/tree/camera/imx681) | Earlier IMX681 implementation; later fixes are on the debug branch |
| [`camera/ov13858-dt`](https://github.com/lain3d/surface-pro-11-kernel/tree/camera/ov13858-dt) | Rear-sensor device-tree probing and power-control work; not a claim of working rear-camera capture |
| [`camera/denali-pm8010`](https://github.com/lain3d/surface-pro-11-kernel/tree/camera/denali-pm8010) | Camera PMIC and board regulator work |
| [`usb4/platform-nhi`](https://github.com/lain3d/surface-pro-11-kernel/tree/usb4/platform-nhi) | Experimental platform-enumerated USB4 NHI work; router binding is not proof of working USB4/eGPU support |
| [`wip/sp11-typec-adsp-fixes`](https://github.com/lain3d/surface-pro-11-kernel/tree/wip/sp11-typec-adsp-fixes) | USB-C/retimer/PHY experiments for external-root boot and ADSP restart |
| [`audio/dmic-clock`](https://github.com/lain3d/surface-pro-11-kernel/tree/audio/dmic-clock) | 2.4 MHz DMIC clock change |
| [`wifi/wcn7850-perst`](https://github.com/lain3d/surface-pro-11-kernel/tree/wifi/wcn7850-perst) | WCN7850 PCIe reset/wake wiring |

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

Published on 2026-10-07 for continuation by others. This publication adds a
project-specific maintenance and branch-map notice without changing kernel code
or rewriting existing branch history. Existing author email addresses, local
hostnames in authorship, and historical session metadata remain in that history.
