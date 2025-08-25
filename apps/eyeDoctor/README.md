# Eye Doctor Application

The Eye Doctor application is a MagAO-X C++ application that performs automatic optimization of deformable mirror (DM) modes for coronagraph performance. It is based on the Python `eye_doctor.py` implementation and provides a full INDI interface for control and monitoring.

## Overview

The Eye Doctor application automates the process of:
1. Poking DM actuators in specific patterns
2. Measuring the resulting Point Spread Function (PSF) quality
3. Optimizing DM mode coefficients to improve coronagraph performance
4. Coordinating timing between DM commands and camera acquisitions

## Features

- **INDI Interface**: Full INDI control interface instead of command-line arguments
- **Configurable Parameters**: All optimization parameters configurable via INDI
- **Camera Selection**: Drop-down selection from available cameras
- **DM Poke Coordination**: Built-in timing control for DM-camera synchronization
- **CACAO Integration**: Support for CACAO mlat parameter for DM poke delays
- **Reusable Framework**: Built on the `dmPokeWFS` framework for future applications

## Configuration

The application uses a configuration file (`eyeDoctor.conf`) with the following sections:

### [eyedoctor]
- `dmModes`: DM device to use for optimization
- `shmim`: Camera shmim to target
- `psfCoreRadiusPixels`: PSF core radius in pixels
- `modesToOptimize`: Comma-separated list of modes to optimize
- `searchRange`: Search range for optimization
- `availableCameras`: List of available cameras
- `dmPokeDelay`: Delay between DM commands in microseconds

### [wfscam]
- `camDevName`: INDI device name of the WFS camera
- `loopSemWait`: Semaphore wait time for WFS loop
- `imageSemWait`: Semaphore wait time for image availability

### [wfsdark]
- `shmimName`: Dark frame shmim name

### [pokecen]
- `dmChannel`: DM channel for pokes
- `pokeX`, `pokeY`: Actuator coordinates to poke
- `pokeAmp`: Poke amplitude
- `dmSleep`: Sleep time for DM command application
- `nPokeImages`: Number of images to average
- `nPokeAverage`: Number of poke sequences to average

## INDI Properties

### Read/Write Properties
- `dmModes`: DM modes configuration
- `shmim`: Camera selection
- `psfCoreRadiusPixels`: PSF core radius
- `modesToOptimize`: Modes to optimize
- `searchRange`: Optimization search range
- `availableCameras`: Available camera list
- `dmPokeDelay`: DM poke delay timing

### Control Properties
- `startOptimization`: Start optimization process
- `stopOptimization`: Stop optimization process

### Read-Only Properties
- `optimizationStatus`: Current optimization status
- `results`: Optimization progress and results

## Building

```bash
cd apps/eyeDoctor
make
```

## Usage

1. Start the application with the configuration file:
   ```bash
   ./eyeDoctor -c eyeDoctor.conf
   ```

2. Connect to the INDI server to control the application

3. Configure parameters via INDI properties

4. Start optimization using the `startOptimization` switch

5. Monitor progress via the `optimizationStatus` and `results` properties

## Architecture

The application is built using the `dmPokeWFS` framework which provides:
- DM poke coordination
- Camera image acquisition
- Dark frame handling
- Timing control
- Basic measurement infrastructure

This framework can be reused for other applications that need similar DM-camera coordination, such as:
- EFC (Electric Field Conjugation) control
- DM alignment tools
- Wavefront sensing applications

## Future Enhancements

- Integration with CACAO for real-time DM control
- Advanced PSF analysis algorithms
- Machine learning optimization
- Multi-camera support
- Automated mode selection
- Performance metrics and reporting
