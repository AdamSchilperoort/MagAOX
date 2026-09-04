# Eye Doctor

MagAO-X C++ port of magpyx / scoobpy `eye_doctor`. Modes are generated in
C++ (Noll Zernike via mxlib, or Sylvester Hadamard on a circular actuator
mask). The app grid-searches mode amplitudes on a cacao displacement
channel, measures PSF core flux on a camera shmim, and keeps the amplitude
that minimizes the metric.

No Python is used at runtime.

## Hardware wiring

| Role | Config / INDI | Meaning |
| --- | --- | --- |
| Eye-doctor command | `shmims.shm_dm_eyeDoc` | Accumulated winning command. Start this channel empty. |
| Sweep / poke | `shmims.shm_dm_eyeDoc_sweep` | Temporary modal pokes during the grid search |
| Flat | `shmims.shm_dm_flat` | cacao flat (typically `dmXXdisp00`) |
| Sum / total | `shmims.shm_dm_sum` | cacao total (`dmXXdisp`) |
| Camera image | `shmims.shm_cam` | Real-time WFS / science camera shmim |
| Camera INDI | `camera.cam_name` | Device for `exptime` / `fps` SET |
| Flat directory | `eyedoctor.flat_dir` | Where `save_flat` writes timestamped FITS |

cacao `dmcomb` sums channels, so two algorithm channels are required: the
sweep channel is zeroed after each mode and `shm_dm_eyeDoc` holds the
running solution. If both names are the same, a single channel is used and
the running command is added during the sweep.

`save_flat` copies `shm_dm_sum` onto `shm_dm_flat`, zeros `shm_dm_eyeDoc`
and the sweep channel, and writes
`flat_eyedoctor_YYYYMMDD-HHMMSS.fits` into `flat_dir`.

This is the correct method when the only extra live content in the sum is the
eye-doctor / sweep channels (which are then zeroed). Other non-zero cacao
channels that were already in the total would be double-counted in the new
flat. It may be a fair assumption that if doing eyeDoctor, only the flat channel
and eyeDoctor channels are the ones being poked, though could warn if other channels have values.

I/O, mode generation, metric, and grid-sweep live in
`libMagAOX/app/dev/dmWavefrontControl.hpp`. This is a rough stab at getting common core dm/wfs algorithms 
into a library that apps can link against and use.

## Modes

| `mode_type` | Behavior |
| --- | --- |
| `zernike` | Generate `n_modes` Noll Zernikes sized to the DM channel. Plane 0 is piston (j=1); index 3 is focus (j=4). |
| `hadamard` | Circular actuator mask, Sylvester Hadamard of the next power of two of valid actuators. |
| `fits` | Load a 3D FITS cube (`eyedoctor.modeset`). Optional fallback only. |

## Algorithm

For each mode in `[mode_start, mode_end]`:

1. Write `amp * mode` on the sweep channel for amplitudes in
   `±search_range/2`. Spacing is `search_step` if that is >0, otherwise
   `search_range / (n_steps-1)`. Each amplitude is sampled `n_repeats`
   times, waiting `dm_delay` then averaging `n_images` camera frames.
2. Metric is magpyx `get_image_coresum` (negative core flux after edge-median
   background subtraction).
3. Fit a quadratic (`search_kind=fit`) or take the mean argmin
   (`search_kind=mean`). Samples whose image peak is below `blank_thresh`
   (default: 10% of the brightest sample in the sweep) are treated as
   PSF-off-camera and dropped; the quadratic is fit on the remaining
   contiguous island. If that fit is still rejected, a finer sweep is
   taken around the best on-camera sample. If that also fails, the
   best-sample amplitude is used when it still shows core flux, otherwise
   amplitude 0.
4. Add the winning shape to `shm_dm_eyeDoc`.

Toggle INDI `run` to start; toggle `run` off to stop without clearing the
accumulated command. Pulse `reset_to_zero` to write zeros to both
`shm_dm_eyeDoc` and `shm_dm_eyeDoc_sweep` (and stop a run if one is in
progress). Pulse `abort` to stop and zero the same channels. Pulse
`save_flat` after a successful run to fold the total command into the flat.
Status, current mode, amplitude, metric, and `last_flat` are published as
read-only properties.

`cen_x` / `cen_y` are floating-point 0-based pixel coordinates of the PSF in
the camera shmim (the ROI), not an offset from center. `<0` (the default)
auto-centroids on the peak. A 512×512 ROI with the PSF in the middle is
`cen_x=255.5`, `cen_y=255.5` (or `256`). Integers still work.

`sat_thresh` (default 55000 ADU) logs a MagAO-X warning with the current
mode index when a camera frame peak meets or exceeds the threshold. Set to
`0` to disable. The loop does not abort.

`blank_thresh` (default `0`) is the peak ADU below which a sweep sample is
treated as blank (PSF walked off the camera). `0` means 10% of the maximum
peak seen in that sweep.

Set `eyedoctor.dark_lib_path` to a `darkCtrl` library (`dark_metadata.txt` plus
`dark_NNN.fits`). Pulse `reload_dark_lib` (or start a `run`) to pick the dark
whose `shm_cam_input`, `emgain`, and `blacklevel` match the live camera INDI
device (`cam_name`), and whose `exptime` is within `exptime_tol` (default
`1e-4` s). That dark is subtracted from each camera frame before the coresum
metric. `last_dark` publishes the FITS path in use. Leave the path empty to
keep edge-median background only.

## Build

```bash
cd apps/eyeDoctor
make
```
