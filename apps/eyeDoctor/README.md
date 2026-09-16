# Eye Doctor

MagAO-X C++ app that grid-searches modal amplitudes to maximize PSF core flux.
It does not generate modes or write cacao DM channels itself. It commands modesets
for dm modes via a connection through INDI to another application
(e.g. a `dmMode` instance such as `alpaoModes`), which presents any modeset,
and connects to another DM app to convert mode amplitudes to a DM shape. 

After connecting to a dm modes app, it pulls the number of `current_amps` 
elements published by the `modes_device` and sets the `modes_max` value
so that the app knows the maximum number of modes it can explore. 
`current_mode` is the index currently bing optimized, or -1 when idle. 
`modes_start` through `mode_end` are the modes to be included
when running eyeDoctor. 

## Hardware wiring

| Role | Config / INDI | Meaning |
| --- | --- | --- |
| Modeset device | `eyedoctor.modes_device` | INDI `dmMode` app (`alpaoModes`, `wooferModes`, …). Commands go to `target_amps`; live values are read from `current_amps` |
| Camera image | `shmims.shm_cam` | Real-time WFS / science camera shmim |
| Camera INDI | `camera.cam_name` | Camera device. Only `exptime` is requested, and only between modes (or on saturation). |
| Flat | `shmims.shm_dm_flat` | cacao flat (typically `dmXXdisp00`), used only by `save_flat` |
| Sum / total | `shmims.shm_dm_sum` | cacao total (`dmXXdisp`), used only by `save_flat` to take summed dm command as the new flat to write |
| Flat directory | `eyedoctor.flat_dir` | Where `save_flat` writes the timestamped FITS flat file |

`save_flat` copies `shm_dm_sum` onto `shm_dm_flat`, zeros amplitudes on
`mode_start`..`mode_end`, and writes `flat_eyedoctor_YYYYMMDD-HHMMSS.fits`.
That is correct when the modal command is the only extra content in the cacao
sum.

## Algorithm Parameters

1. If `randomize` is on, optionally optimize `focus_mode_index` first 
   (or disable with `ignore_focus`), then split modes into shuffled clusters of
   `n_cluster`. If `randomize` is off, modes run in index order from
   `mode_start` to `mode_end`.
2. `n_seq_repeat` repeats the full sequence a number of times. 
   Repeating and shuffling the cluster only applies when `randomize` is on.
3. Between modes, gets the `cam_name.exptime`. If that device or property is
   missing, the metric is raw negative coresum. If `exptime` is available,
   the metric is coresum / exptime (ADU/s). Connects to a `cam_name` specified camera through INDI.
4. If a frame peaks at or above `sat_thresh`, log a warning and send
   `exptime.target` reduced by `sat_exptime_frac` (default `0.1` = 10%).
   Exposure is restored when the run ends. `sat_thresh=0` disables this.
5. For each mode, search amplitudes over `±search_range/2` centered on the
   live `current_amps` value when `baseline` is on. If `baseline` is off,
   modes in `mode_start`..`mode_end` are zeroed first.
6. Each sample is sent as `modes_device.target_amps.NNNN` and waits until
   `current_amps` matches within `amp_tol` (timeout `amp_timeout`). Then
   `skip_frames` camera frames are discarded and `n_images` are averaged.
7. Minimize the metric to maximize core flux. `search_kind=grid` (default)
   samples `n_steps` amplitudes `n_repeats` times and fits a quadratic.
   `search_kind=brent` is a bounded 1-D Brent search. Grid drops blank
   (PSF-off-camera) samples and can refine around the best remaining point.

Toggle INDI `run` to start; toggle `run` off to stop without zeroing modes.
Pulse `reset_to_zero` to send 0 to `target_amps` for indices
`mode_start`..`mode_end` only. Pulse `abort` to stop and zero that same
range. Toggle `save_flat` after a successful run to fold the total
command into the flat that is saved. Then load the file via the dm app.

`cen_x` / `cen_y` are floating-point 0-based pixel coordinates of the PSF in
the camera shmim (the ROI), not an offset from center. `<0` (the default)
auto-centroids on the peak.

I/O, metric, and grid-sweep helpers live in
`libMagAOX/app/dev/dmWavefrontControl.hpp`.

