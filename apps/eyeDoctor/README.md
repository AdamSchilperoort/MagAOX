# Eye Doctor

MagAO-X C++ app that grid-searches modal amplitudes to maximize PSF core flux.
It does not generate modes or write cacao DM channels itself. It commands a
remote INDI modeset device (a `dmMode` instance such as `alpaoModes`), which
owns the modeset, converts amplitudes to a DM shape, and talks to the DM.

`modes_max` is the number of `current_amps` elements published by
`modes_device` (valid indices are `0 .. modes_max-1`). `current_mode` is the
index currently being optimized (`-1` when idle). If `mode_start`/`mode_end`
include indices outside that range, those modes are skipped with a warning.

## Hardware wiring

| Role | Config / INDI | Meaning |
| --- | --- | --- |
| Modeset device | `eyedoctor.modes_device` | INDI `dmMode` app (`alpaoModes`, `wooferModes`, …). Commands go to `target_amps`; live values are read from `current_amps` (`0000`, `0001`, …). |
| Camera image | `shmims.shm_cam` | Real-time WFS / science camera shmim |
| Camera INDI | `camera.cam_name` | Camera device. Only `exptime` is requested, and only between modes (or on saturation). |
| Flat | `shmims.shm_dm_flat` | cacao flat (typically `dmXXdisp00`), used only by `save_flat` |
| Sum / total | `shmims.shm_dm_sum` | cacao total (`dmXXdisp`), used only by `save_flat` |
| Flat directory | `eyedoctor.flat_dir` | Where `save_flat` writes timestamped FITS |

`save_flat` copies `shm_dm_sum` onto `shm_dm_flat`, zeros amplitudes on
`mode_start`..`mode_end`, and writes `flat_eyedoctor_YYYYMMDD-HHMMSS.fits`.
That is correct when the modal command is the only extra content in the cacao
sum.

## Algorithm

1. If `randomize` is on, optionally optimize `focus_mode_index` first (disable
   with `ignore_focus`), then split modes into shuffled clusters of
   `n_cluster`. If `randomize` is off, modes run in index order from
   `mode_start` to `mode_end`. The INDI toggle is initialized from config at
   startup (Off in the GUI means Off in the worker).
2. `n_seq_repeat` repeats the full sequence. Cluster shuffle/repeat only
   apply when `randomize` is on.
3. Between modes, GET `cam_name.exptime`. If that device or property is
   missing, the metric is raw negative coresum. If `exptime` is available,
   the metric is coresum / exptime (ADU/s).
4. If a frame peaks at or above `sat_thresh`, log a warning and send
   `exptime.target` reduced by `sat_exptime_frac` (default `0.1` = 10%).
   Exposure is restored when the run ends. `sat_thresh=0` disables this.
5. For each mode, search amplitudes over `±search_range/2` centered on the
   live `current_amps` value when `baseline` is on. If `baseline` is off,
   modes in `mode_start`..`mode_end` are zeroed first.
6. Each sample is sent as `modes_device.target_amps.NNNN` and waited until
   `current_amps` matches within `amp_tol` (timeout `amp_timeout`). Then
   `skip_frames` camera frames are discarded and `n_images` are averaged.
7. Minimize the metric to maximize core flux. `search_kind=grid` (default)
   samples `n_steps` amplitudes `n_repeats` times and fits a quadratic.
   `search_kind=brent` is a bounded 1-D Brent search. Grid drops blank
   (PSF-off-camera) samples and can refine around the best remaining point.

Toggle INDI `run` to start; toggle `run` off to stop without zeroing modes.
Pulse `reset_to_zero` to send 0 to `target_amps` for indices
`mode_start`..`mode_end` only. Pulse `abort` to stop and zero that same
range. Pulse `save_flat` after a successful run to fold the total
command into the flat.

`cen_x` / `cen_y` are floating-point 0-based pixel coordinates of the PSF in
the camera shmim (the ROI), not an offset from center. `<0` (the default)
auto-centroids on the peak.

I/O, metric, and grid-sweep helpers live in
`libMagAOX/app/dev/dmWavefrontControl.hpp`.

## Build

```bash
cd apps/eyeDoctor
make
```
