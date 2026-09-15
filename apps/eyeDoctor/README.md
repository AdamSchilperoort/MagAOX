# Eye Doctor

MagAO-X C++ port of magpyx `eye_doctor_comprehensive` (`dm_eye_doctor`).
This app does **not** generate modes or write cacao DM channels itself.
It commands a remote INDI modeset device (a `dmMode` instance such as
`alpaoModes`), which owns the modeset, converts amplitudes to a DM shape,
and talks to the DM.

INDI equivalent of:

```
dm_eye_doctor 7624 alpaoModes camsci 8 2...10 0.1 --skip 1
```

| CLI | INDI / config |
| --- | --- |
| `alpaoModes` | `modes_device` |
| `camsci` | `shm_cam` / `cam_name` |
| `8` | `core_radius` |
| `2...10` | `mode_start`/`mode_end`, or `modes` |
| `0.1` | `search_range` |
| `--skip 1` | `skip_frames` |

`n_modes_loaded` is the number of `current_amps` elements published by that
device. If the requested search includes modes outside `0 .. n_modes_loaded-1`,
those modes are dropped with a warning.

No Python is used at runtime.

## Hardware wiring

| Role | Config / INDI | Meaning |
| --- | --- | --- |
| Modeset device | `eyedoctor.modes_device` | INDI `dmMode` app (`alpaoModes`, `wooferModes`, …). Commands go to `target_amps`; live values are read from `current_amps` (`0000`, `0001`, …). |
| Camera image | `shmims.shm_cam` | Real-time WFS / science camera shmim |
| Camera INDI | `camera.cam_name` | Device for `exptime` / `fps` / `emgain` / `blacklevel` SET |
| Flat | `shmims.shm_dm_flat` | cacao flat (typically `dmXXdisp00`), used only by `save_flat` |
| Sum / total | `shmims.shm_dm_sum` | cacao total (`dmXXdisp`), used only by `save_flat` |
| Flat directory | `eyedoctor.flat_dir` | Where `save_flat` writes timestamped FITS |

`save_flat` copies `shm_dm_sum` onto `shm_dm_flat`, zeros all amplitudes on
`modes_device`, and writes `flat_eyedoctor_YYYYMMDD-HHMMSS.fits`. That is
correct when the modal command is the only extra content in the cacao sum.

## Algorithm

Matches magpyx `eye_doctor_comprehensive`:

1. Optionally optimize `focus_mode_index` first (default 2; disable with
   `ignore_focus`).
2. Split the requested modes into clusters of `n_cluster` (default 5), shuffle
   each cluster if `randomize` is on, repeat each cluster `n_cluster_repeat`
   times and the whole sequence `n_seq_repeat` times.
3. For each mode, grid-search amplitudes over `±search_range/2` centered on
   the live `current_amps` value when `baseline` is on (magpyx default). If
   `baseline` is off, all modes are zeroed first and the search is about 0.
4. Each sample is sent as `modes_device.target_amps.NNNN` and waited until
   `current_amps` matches within `amp_tol` (timeout `amp_timeout`). Then
   `skip_frames` camera frames are discarded and `n_images` are averaged.
5. Metric is magpyx `get_image_coresum` (negative core flux after edge-median
   background). `search_kind` matches magpyx: `grid` (default, the
   `dm_eye_doctor` path) samples `n_steps` amplitudes `n_repeats` times and
   fits a quadratic; `brent` is scipy bounded Brent. Grid uses the same
   blank-sample truncation / refine fallback as before.

Toggle INDI `run` to start; toggle `run` off to stop without zeroing modes.
Pulse `reset_to_zero` to send 0 to every `target_amps` element. Pulse `abort`
to stop and zero. Pulse `save_flat` after a successful run to fold the total
command into the flat.

`cen_x` / `cen_y` are floating-point 0-based pixel coordinates of the PSF in
the camera shmim (the ROI), not an offset from center. `<0` (the default)
auto-centroids on the peak.

Set `eyedoctor.dark_lib_path` to a `darkCtrl` library (`dark_metadata.txt` plus
`dark_NNN.fits`). Pulse `reload_dark_lib` (or start a `run`) to pick the dark
whose `shm_cam_input`, `emgain`, and `blacklevel` match the live camera INDI
device (`cam_name`), and whose `exptime` is within `exptime_tol` (default
`1e-4` s). That dark is subtracted from each camera frame before the coresum
metric. Leave the path empty to keep edge-median background only.

I/O, metric, and grid-sweep helpers live in
`libMagAOX/app/dev/dmWavefrontControl.hpp`.

## Build

```bash
cd apps/eyeDoctor
make
```
