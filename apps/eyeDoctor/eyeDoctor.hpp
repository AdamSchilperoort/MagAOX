/** \file eyeDoctor.hpp
  * \brief The MagAO-X Eye Doctor header file
  *
  * \ingroup eyeDoctor_files
  */

#ifndef eyeDoctor_hpp
#define eyeDoctor_hpp

#include <mx/improc/eigenImage.hpp>
#include <mx/improc/milkImage.hpp>
#include <mx/improc/eigenCube.hpp>
#include <sstream>
#include <thread>
#include <mutex>
#include <sys/syscall.h>
#include <unistd.h>
using namespace mx::improc;

#include "../../libMagAOX/libMagAOX.hpp" //Note this is included on command line to trigger pch
#include "../../magaox_git_version.h"
#include "../../libMagAOX/app/dev/dmWavefrontControl.hpp"

/** \defgroup eyeDoctor
  * \brief The MagAO-X application to perform eye doctor measurements for DM optimization
  *
  * <a href="../handbook/operating/software/apps/eyeDoctor.html">Application Documentation</a>
  *
  * \ingroup apps
  *
  */

/** \defgroup eyeDoctor_files
  * \ingroup eyeDoctor
  */

namespace MagAOX
{
namespace app
{

/// The MagAO-X Eye Doctor Application
/**
 * \ingroup eyeDoctor
 */
class eyeDoctor : public MagAOXApp<true>,
                 public dev::telemeter<eyeDoctor>
{
    // Give the test harness access.
    friend class eyeDoctor_test;

    friend class dev::telemeter<eyeDoctor>;
    typedef dev::telemeter<eyeDoctor> telemeterT;

protected:
    /** \name Wavefront Control Helper
      * @{
      */
    
    /// Utility class instance for wavefront sensing algorithms
    dev::dmWavefrontControl m_wfsHelper;
    
    ///@}

protected:
    /** \name Configurable Parameters
      *@{
     */
    
    // DM Control Mode Selection
    std::string m_dmControlMode;                  ///< "magaox" or "cacao" - determines interface type
    
    // MagAOX App Mode: Interface with MagAOX DM applications
    std::vector<std::string> m_availableDMApps;   ///< List of available MagAOX DM apps (dmwoofer, dmtweeter, etc.)
    std::string m_selectedDMApp;                  ///< Currently selected MagAOX DM app
    
    // CACAO Shmim Mode: Interface directly with CACAO streams
    std::vector<std::string> m_availableDMShmims; ///< List of available CACAO DM shmims (dm00disp, dm01disp, etc.)
    std::string m_selectedDMShmim;                ///< Currently selected CACAO DM shmim
    
    // Resolved DM names (set based on mode and selection)
    std::string m_dmDeviceName;                   ///< Actual DM device name (for MagAOX app mode)
    std::string m_dmShmimName;                    ///< DM shared memory name (for CACAO mode or app->shmim mapping)
    
    // Camera Selection
    std::vector<std::string> m_availableCameras;  ///< List of available cameras
    std::string m_selectedCamera;                 ///< Currently selected camera for wavefront sensing
    
    // Mode Specification
    int m_startModeIndex;                         ///< Starting mode index to optimize
    int m_endModeIndex;                           ///< Ending mode index to optimize
    
    // Camera/WFS Configuration
    std::string m_wfsShmimName;          ///< WFS camera shared memory name
    std::string m_wfsCameraName;         ///< WFS camera device name
    std::string m_wfsDarkShmimName;      ///< Optional dark frame shmim
    
    // Modeset Configuration
    std::vector<std::string> m_modesetFiles;  ///< List of modeset FITS files
    std::vector<std::string> m_modesetNames;  ///< Names for each modeset
    std::string m_defaultModeSet;             ///< Default modeset to use
    
    // Algorithm Parameters (from console_comprehensive)
    double m_psfCoreRadiusPixels;        ///< Radius of the PSF core to measure
    double m_searchRange;                ///< Search range for optimization
    int m_nSteps;                        ///< Number of points to sample in grid search
    int m_nRepeats;                      ///< Number of sweeps
    int m_nClusterRepeats;               ///< Number of times to repeat a cluster of modes
    int m_nSeqRepeat;                    ///< Number of times to repeat optimization of all modes
    int m_nImages;                       ///< Number of images to collect from shmim
    int m_cenX;                          ///< Fixed PSF centroid X value
    int m_cenY;                          ///< Fixed PSF centroid Y value
    int m_skipFrames;                    ///< Number of frames to skip
    bool m_resetToZero;                  ///< Ignore current mode value and optimize about 0
    bool m_ignoreFocus;                  ///< Skip focus mode optimization
    
    // Optimization Parameters
    double m_targetLatency;              ///< Target latency between DM and camera (microseconds)
    double m_latencyTolerance;           ///< Tolerance around target latency (microseconds)
    bool m_autoOptimizeLatency;          ///< Whether to automatically optimize latency
    int m_maxIterations;                 ///< Maximum number of optimization iterations
    double m_convergenceThreshold;       ///< Convergence threshold for optimization
    bool m_adaptiveStepSize;             ///< Whether to use adaptive step sizes
    
    // DM Metadata Parameters
    int m_numActuators;                  ///< Number of actuators in the DM
    int m_gridWidth;                     ///< Actuator grid width
    int m_gridHeight;                    ///< Actuator grid height
    std::string m_dmType;                ///< DM type/manufacturer
    std::vector<int> m_deadActuators;    ///< List of dead actuator indices
    std::string m_couplingMatrix;        ///< Path to coupling matrix file (optional)
    std::string m_actuatorGains;         ///< Path to actuator gains file (optional)
    std::string m_actuatorLimits;        ///< Path to actuator limits file (optional)
    
    ///@}

    // Eye doctor specific members
    mx::improc::eigenImage<float> m_psfImage;
    mx::improc::eigenImage<float> m_optimizedImage;
    std::vector<float> m_modeCoefficients;
    std::vector<float> m_optimizationResults;
    
    // State management
    bool m_optimizationInProgress;
    bool m_measurementComplete;
    int m_currentModeIndex;
    int m_totalModes;

public:
    /// Default c'tor.
    eyeDoctor();

    /// D'tor, declared and defined for noexcept.
    ~eyeDoctor() noexcept
    {
        // Try to cleanup, but don't do anything that could throw or access invalid memory
        if(m_optimizationThread.joinable())
        {
            m_optimizationInProgress = false;
            m_optimizationThreadInit = false;
            // Don't try to join in destructor - let std::thread destructor handle it
            // Joining here can cause issues if base classes are already destroyed
        }
    }

    virtual void setupConfig();

    /// Implementation of loadConfig logic, separated for testing.
    /** This is called by loadConfig().
      */
    int loadConfigImpl( mx::app::appConfigurator & _config /**< [in] an application configuration from which to load values*/);

    virtual void loadConfig();

    /// Startup function
    /**
      *
      */
    virtual int appStartup();

    /// Implementation of the FSM for eyeDoctor.
    /** 
      * \returns 0 on no critical error
      * \returns -1 on an error requiring shutdown
      */
    virtual int appLogic();

    /// Shutdown the app.
    /** 
      *
      */
    virtual int appShutdown();

    // Required interface functions for dmWavefrontControl
    int runWavefrontSensing();
    int analyzeWavefrontSensing();

    // Eye doctor specific functions
    int runOptimizationAlgorithm();  // Main optimization algorithm
    int measurePSF();
    int optimizeMode(int modeIndex);
    int calculateOptimizationResult();

protected:
    /** \name INDI Interface
      * @{
      */

    // DM Control Mode
    pcf::IndiProperty m_indiP_dmControlMode;
    
    // MagAOX App Mode
    pcf::IndiProperty m_indiP_availableDMApps;
    pcf::IndiProperty m_indiP_selectedDMApp;
    
    // CACAO Shmim Mode
    pcf::IndiProperty m_indiP_availableDMShmims;
    pcf::IndiProperty m_indiP_selectedDMShmim;
    
    // Legacy property (for backward compatibility, shows current selection)
    pcf::IndiProperty m_indiP_availableDMs;
    pcf::IndiProperty m_indiP_selectedDM;

    // Camera Selection
    pcf::IndiProperty m_indiP_availableCameras;
    pcf::IndiProperty m_indiP_selectedCamera;

    // Mode Specification
    pcf::IndiProperty m_indiP_startModeIndex;
    pcf::IndiProperty m_indiP_endModeIndex;

    // Algorithm Parameters
    pcf::IndiProperty m_indiP_psfCoreRadiusPixels;
    pcf::IndiProperty m_indiP_searchRange;
    pcf::IndiProperty m_indiP_nSteps;
    pcf::IndiProperty m_indiP_nRepeats;
    pcf::IndiProperty m_indiP_nClusterRepeats;
    pcf::IndiProperty m_indiP_nSeqRepeat;
    pcf::IndiProperty m_indiP_nImages;
    pcf::IndiProperty m_indiP_cenX;
    pcf::IndiProperty m_indiP_cenY;
    pcf::IndiProperty m_indiP_skipFrames;
    pcf::IndiProperty m_indiP_resetToZero;
    pcf::IndiProperty m_indiP_ignoreFocus;

    // Optimization Control
    pcf::IndiProperty m_indiP_optimizationStatus;
    pcf::IndiProperty m_indiP_results;
    pcf::IndiProperty m_indiP_runOptimization;  // Single toggle switch to start/stop
    
    // Optimization thread
    std::thread m_optimizationThread;
    bool m_optimizationThreadInit{false};
    pid_t m_optimizationThreadPID{0};
    static void optimizationThreadStart(eyeDoctor *e);
    
    // INDI property sending flag
    bool m_indiPropertiesSent{false};
    void optimizationThreadExec();

    // Algorithm-specific INDI properties
    pcf::IndiProperty m_indiP_targetLatency;
    pcf::IndiProperty m_indiP_autoOptimizeLatency;

    // DM Metadata INDI properties
    pcf::IndiProperty m_indiP_numActuators;
    pcf::IndiProperty m_indiP_gridWidth;
    pcf::IndiProperty m_indiP_gridHeight;
    pcf::IndiProperty m_indiP_dmType;
    pcf::IndiProperty m_indiP_deadActuators;
    pcf::IndiProperty m_indiP_couplingMatrix;
    pcf::IndiProperty m_indiP_actuatorGains;
    pcf::IndiProperty m_indiP_actuatorLimits;

    // Callback declarations for eyeDoctor-specific properties
    
    // DM Control Mode callbacks
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_dmControlMode);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_selectedDMApp);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_selectedDMShmim);
    
    // Legacy callbacks (for backward compatibility)
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_availableDMs);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_selectedDM);
    
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_selectedCamera);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_startModeIndex);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_endModeIndex);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_psfCoreRadiusPixels);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_searchRange);  // Base class registers, derived implements
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_nSteps);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_nRepeats);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_nClusterRepeats);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_nSeqRepeat);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_nImages);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_cenX);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_cenY);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_skipFrames);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_resetToZero);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_ignoreFocus);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_targetLatency);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_autoOptimizeLatency);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_runOptimization);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_optimizationStatus);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_results);
    
    // Callback declarations for DM metadata properties
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_numActuators);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_gridWidth);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_gridHeight);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_dmType);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_deadActuators);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_couplingMatrix);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_actuatorGains);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_actuatorLimits);


    ///@}

    /** \name Telemeter Interface
      * @{
      */

    int recordTelem(const telem_pokeloop *);

    int recordEyeDoctor(bool force = false);

    ///@}

private:
    int updateOptimizationStatus();
    int updateResults();
    
    // Dynamic configuration methods for dmWavefrontControl
    int updateDMConfiguration();
    int updateCameraConfiguration();
    int updateSearchParameters();
    
    // DM device mapping
    struct DMDeviceInfo {
        std::string deviceName;
        std::string shmimName;
        std::string description;
    };
    std::map<std::string, DMDeviceInfo> m_dmDeviceMap;


};

eyeDoctor::eyeDoctor() : MagAOXApp(MAGAOX_CURRENT_SHA1, MAGAOX_REPO_MODIFIED)
{
    // Initialize DM control mode and device parameters
    m_dmControlMode = "magaox";  // Default to MagAOX app mode
    m_availableDMApps = {"dmwoofer", "dmncpc", "dmtweeter", "dmkilo"};
    m_selectedDMApp = "dmtweeter";
    m_availableDMShmims = {"dm00disp", "dm01disp", "dm02disp"};
    m_selectedDMShmim = "dm01disp";
    m_dmDeviceName = "dmtweeter";
    m_dmShmimName = "dm01disp";
    
    // Initialize camera parameters
    m_availableCameras = {"camsci1", "camsci2", "camlowfs", "camtip", "camflowf"};
    m_selectedCamera = "camsci1";
    
    // Initialize mode specification
    m_startModeIndex = 1;
    m_endModeIndex = 5;
    
    // Initialize algorithm parameters
    m_psfCoreRadiusPixels = 5.0;
    m_searchRange = 0.1;
    m_nSteps = 20;
    m_nRepeats = 3;
    m_nClusterRepeats = 1;
    m_nSeqRepeat = 1;
    m_nImages = 1;
    m_cenX = 64;
    m_cenY = 64;
    m_skipFrames = 0;
    m_resetToZero = false;
    m_ignoreFocus = false;
    
    // Initialize optimization parameters
    m_targetLatency = 1000; // 1ms default
    m_latencyTolerance = 100; // 100 microseconds tolerance
    m_autoOptimizeLatency = false;
    m_maxIterations = 100;
    m_convergenceThreshold = 1e-6;
    m_adaptiveStepSize = true;
    
    // Initialize DM metadata parameters (will be loaded from config)
    m_numActuators = 0;  // MUST be 0 so config loading works
    m_gridWidth = 0;     // MUST be 0 so config loading works
    m_gridHeight = 0;    // MUST be 0 so config loading works
    m_dmType = "Boston Micromachines";
    m_deadActuators.clear();
    m_couplingMatrix = "";
    m_actuatorGains = "";
    m_actuatorLimits = "";
    
    // Initialize state management
    m_optimizationInProgress = false;
    m_measurementComplete = false;
    m_currentModeIndex = 0;
    m_totalModes = 1; // Placeholder - parse from m_modesToOptimize
    
    // Initialize DM control mode
    m_dmControlMode = "magaox"; // Default to MagAOX app mode
    
    // Initialize DM device mapping for MagAOX mode
    m_dmDeviceMap = {
        {"dmwoofer", {"dmwoofer", "dm00disp", "DM Woofer"}},
        {"dmncpc", {"dmncpc", "dm02disp", "DM NCPc"}},
        {"dmtweeter", {"dmtweeter", "dm01disp", "DM Tweeter"}},
        {"dmkilo", {"dmkilo", "dm00disp", "DM Kilo"}}
    };
    
    return;
}

void eyeDoctor::setupConfig()
{
    // Call base class setupConfig first
    MagAOXApp<true>::setupConfig();

    // DM Control Mode
    config.add("eyedoctor.dmControlMode", "", "eyedoctor.dmControlMode", argType::Optional, "eyedoctor", "dmControlMode", false, "string", "DM control mode: 'magaox' (interface with MagAOX apps) or 'cacao' (direct shmim)");
    
    // MagAOX App Mode: DM Application Selection
    config.add("eyedoctor.availableDMApps", "", "eyedoctor.availableDMApps", argType::Optional, "eyedoctor", "availableDMApps", false, "vector<string>", "List of available MagAOX DM apps (dmwoofer, dmtweeter, dmkilo, etc.)");
    config.add("eyedoctor.selectedDMApp", "", "eyedoctor.selectedDMApp", argType::Optional, "eyedoctor", "selectedDMApp", false, "string", "Selected MagAOX DM app (for magaox mode)");
    
    // CACAO Shmim Mode: Direct shmim Selection
    config.add("eyedoctor.availableDMShmims", "", "eyedoctor.availableDMShmims", argType::Optional, "eyedoctor", "availableDMShmims", false, "vector<string>", "List of available CACAO DM shmims (dm00disp, dm01disp, dm02disp, etc.)");
    config.add("eyedoctor.selectedDMShmim", "", "eyedoctor.selectedDMShmim", argType::Optional, "eyedoctor", "selectedDMShmim", false, "string", "Selected CACAO DM shmim (for cacao mode)");
    
    // Camera Selection
    config.add("eyedoctor.availableCameras", "", "eyedoctor.availableCameras", argType::Required, "eyedoctor", "availableCameras", false, "vector<string>", "List of available cameras");
    config.add("eyedoctor.selectedCamera", "", "eyedoctor.selectedCamera", argType::Required, "eyedoctor", "selectedCamera", false, "string", "Selected camera for wavefront sensing");
    
    // Mode Specification
    config.add("eyedoctor.startModeIndex", "", "eyedoctor.startModeIndex", argType::Required, "eyedoctor", "startModeIndex", false, "int", "Starting mode index to optimize");
    config.add("eyedoctor.endModeIndex", "", "eyedoctor.endModeIndex", argType::Required, "eyedoctor", "endModeIndex", false, "int", "Ending mode index to optimize");
    
    // Algorithm Parameters
    config.add("eyedoctor.psfCoreRadiusPixels", "", "eyedoctor.psfCoreRadiusPixels", argType::Required, "eyedoctor", "psfCoreRadiusPixels", false, "float", "Radius of the PSF core to measure");
    config.add("eyedoctor.searchRange", "", "eyedoctor.searchRange", argType::Required, "eyedoctor", "searchRange", false, "float", "Range of values in microns for grid search");
    config.add("eyedoctor.nSteps", "", "eyedoctor.nSteps", argType::Required, "eyedoctor", "nSteps", false, "int", "Number of points to sample in grid search");
    config.add("eyedoctor.nRepeats", "", "eyedoctor.nRepeats", argType::Required, "eyedoctor", "nRepeats", false, "int", "Number of sweeps");
    config.add("eyedoctor.nClusterRepeats", "", "eyedoctor.nClusterRepeats", argType::Required, "eyedoctor", "nClusterRepeats", false, "int", "Number of times to repeat a cluster of modes");
    config.add("eyedoctor.nSeqRepeat", "", "eyedoctor.nSeqRepeat", argType::Required, "eyedoctor", "nSeqRepeat", false, "int", "Number of times to repeat optimization of all modes");
    config.add("eyedoctor.nImages", "", "eyedoctor.nImages", argType::Required, "eyedoctor", "nImages", false, "int", "Number of images to collect from shmim");
    config.add("eyedoctor.cenX", "", "eyedoctor.cenX", argType::Required, "eyedoctor", "cenX", false, "int", "Fixed PSF centroid X value");
    config.add("eyedoctor.cenY", "", "eyedoctor.cenY", argType::Required, "eyedoctor", "cenY", false, "int", "Fixed PSF centroid Y value");
    config.add("eyedoctor.skipFrames", "", "eyedoctor.skipFrames", argType::Required, "eyedoctor", "skipFrames", false, "int", "Number of frames to skip");
    config.add("eyedoctor.resetToZero", "", "eyedoctor.resetToZero", argType::Required, "eyedoctor", "resetToZero", false, "bool", "Ignore current mode value and optimize about 0");
    config.add("eyedoctor.ignoreFocus", "", "eyedoctor.ignoreFocus", argType::Required, "eyedoctor", "ignoreFocus", false, "bool", "Skip focus mode optimization");
    
    // Optimization Parameters
    config.add("eyedoctor.targetLatency", "", "eyedoctor.targetLatency", argType::Required, "eyedoctor", "targetLatency", false, "float", "Target latency between DM and camera (microseconds)");
    config.add("eyedoctor.latencyTolerance", "", "eyedoctor.latencyTolerance", argType::Required, "eyedoctor", "latencyTolerance", false, "float", "Tolerance around target latency (microseconds)");
    config.add("eyedoctor.autoOptimizeLatency", "", "eyedoctor.autoOptimizeLatency", argType::Required, "eyedoctor", "autoOptimizeLatency", false, "bool", "Whether to automatically optimize latency");
    config.add("eyedoctor.maxIterations", "", "eyedoctor.maxIterations", argType::Required, "eyedoctor", "maxIterations", false, "int", "Maximum number of optimization iterations");
    config.add("eyedoctor.convergenceThreshold", "", "eyedoctor.convergenceThreshold", argType::Required, "eyedoctor", "convergenceThreshold", false, "float", "Convergence threshold for optimization");
    config.add("eyedoctor.adaptiveStepSize", "", "eyedoctor.adaptiveStepSize", argType::Required, "eyedoctor", "adaptiveStepSize", false, "bool", "Whether to use adaptive step sizes");
    
    // DM Metadata Parameters - read from dmWavefrontControl section for INDI display/update
    // These are kept as member variables for INDI interface, but loaded from base class config
    config.add("eyedoctor.numActuators", "", "eyedoctor.numActuators", argType::Optional, "eyedoctor", "numActuators", false, "int", "Number of actuators in the DM (reads from dmWavefrontControl if not set)");
    config.add("eyedoctor.gridWidth", "", "eyedoctor.gridWidth", argType::Optional, "eyedoctor", "gridWidth", false, "int", "Actuator grid width (reads from dmWavefrontControl if not set)");
    config.add("eyedoctor.gridHeight", "", "eyedoctor.gridHeight", argType::Optional, "eyedoctor", "gridHeight", false, "int", "Actuator grid height (reads from dmWavefrontControl if not set)");
    config.add("eyedoctor.dmType", "", "eyedoctor.dmType", argType::Optional, "eyedoctor", "dmType", false, "string", "DM type/manufacturer (reads from dmWavefrontControl if not set)");
    config.add("eyedoctor.deadActuators", "", "eyedoctor.deadActuators", argType::Optional, "eyedoctor", "deadActuators", false, "vector<int>", "List of dead actuator indices (reads from dmWavefrontControl if not set)");
    config.add("eyedoctor.couplingMatrix", "", "eyedoctor.couplingMatrix", argType::Optional, "eyedoctor", "couplingMatrix", false, "string", "Path to coupling matrix file (optional)");
    config.add("eyedoctor.actuatorGains", "", "eyedoctor.actuatorGains", argType::Optional, "eyedoctor", "actuatorGains", false, "string", "Path to actuator gains file (optional)");
    config.add("eyedoctor.actuatorLimits", "", "eyedoctor.actuatorLimits", argType::Optional, "eyedoctor", "actuatorLimits", false, "string", "Path to actuator limits file (optional)");
    
    // DM and Camera Configuration
    config.add("eyedoctor.dmDeviceName", "", "eyedoctor.dmDeviceName", argType::Optional, "eyedoctor", "dmDeviceName", false, "string", "DM device name (dmkilo, dmtweeter, etc.)");
    config.add("eyedoctor.dmShmimName", "", "eyedoctor.dmShmimName", argType::Optional, "eyedoctor", "dmShmimName", false, "string", "DM shared memory name (dm00disp, dm01disp, etc.)");
    config.add("eyedoctor.deviceName", "", "eyedoctor.deviceName", argType::Optional, "eyedoctor", "deviceName", false, "string", "DM device/shmim channel name (e.g., dm00disp06)");
    config.add("eyedoctor.wfsShmimName", "", "eyedoctor.wfsShmimName", argType::Optional, "eyedoctor", "wfsShmimName", false, "string", "WFS camera shared memory name");
    config.add("eyedoctor.wfsCameraName", "", "eyedoctor.wfsCameraName", argType::Optional, "eyedoctor", "wfsCameraName", false, "string", "WFS camera device name");
    config.add("eyedoctor.wfsDarkShmimName", "", "eyedoctor.wfsDarkShmimName", argType::Optional, "eyedoctor", "wfsDarkShmimName", false, "string", "WFS dark frame shmim name (optional)");
    config.add("eyedoctor.actuator_spacing", "", "eyedoctor.actuator_spacing", argType::Optional, "eyedoctor", "actuator_spacing", false, "float", "Actuator spacing in mm");
    config.add("eyedoctor.max_stroke", "", "eyedoctor.max_stroke", argType::Optional, "eyedoctor", "max_stroke", false, "float", "Maximum stroke in microns");
    
    // Modeset Configuration
    config.add("eyedoctor.modesets", "", "eyedoctor.modesets", argType::Optional, "eyedoctor", "modesets", false, "string", "Comma-separated list of modeset FITS file paths");
    config.add("eyedoctor.modeset_names", "", "eyedoctor.modeset_names", argType::Optional, "eyedoctor", "modeset_names", false, "string", "Comma-separated list of names for each modeset");
    config.add("eyedoctor.default_modeset", "", "eyedoctor.default_modeset", argType::Optional, "eyedoctor", "default_modeset", false, "string", "Name of the default modeset to use");
}

int eyeDoctor::loadConfigImpl( mx::app::appConfigurator & _config )
{
    // DM Control Mode
    _config(m_dmControlMode, "eyedoctor.dmControlMode");
    
    // MagAOX App Mode: DM Application Selection
    _config(m_availableDMApps, "eyedoctor.availableDMApps");
    _config(m_selectedDMApp, "eyedoctor.selectedDMApp");
    
    // CACAO Shmim Mode: Direct shmim Selection
    _config(m_availableDMShmims, "eyedoctor.availableDMShmims");
    _config(m_selectedDMShmim, "eyedoctor.selectedDMShmim");
    
    // Camera Selection
    _config(m_availableCameras, "eyedoctor.availableCameras");
    _config(m_selectedCamera, "eyedoctor.selectedCamera");
    
    // Mode Specification
    _config(m_startModeIndex, "eyedoctor.startModeIndex");
    _config(m_endModeIndex, "eyedoctor.endModeIndex");
    
    // Algorithm Parameters
    _config(m_psfCoreRadiusPixels, "eyedoctor.psfCoreRadiusPixels");
    _config(m_searchRange, "eyedoctor.searchRange");
    _config(m_nSteps, "eyedoctor.nSteps");
    _config(m_nRepeats, "eyedoctor.nRepeats");
    _config(m_nClusterRepeats, "eyedoctor.nClusterRepeats");
    _config(m_nSeqRepeat, "eyedoctor.nSeqRepeat");
    _config(m_nImages, "eyedoctor.nImages");
    _config(m_cenX, "eyedoctor.cenX");
    _config(m_cenY, "eyedoctor.cenY");
    _config(m_skipFrames, "eyedoctor.skipFrames");
    _config(m_resetToZero, "eyedoctor.resetToZero");
    _config(m_ignoreFocus, "eyedoctor.ignoreFocus");
    
    // Optimization Parameters
    _config(m_targetLatency, "eyedoctor.targetLatency");
    _config(m_latencyTolerance, "eyedoctor.latencyTolerance");
    _config(m_autoOptimizeLatency, "eyedoctor.autoOptimizeLatency");
    _config(m_maxIterations, "eyedoctor.maxIterations");
    _config(m_convergenceThreshold, "eyedoctor.convergenceThreshold");
    _config(m_adaptiveStepSize, "eyedoctor.adaptiveStepSize");
    
    // DM Metadata Parameters
    _config(m_numActuators, "eyedoctor.numActuators");
    _config(m_gridWidth, "eyedoctor.gridWidth");
    _config(m_gridHeight, "eyedoctor.gridHeight");
    _config(m_dmType, "eyedoctor.dmType");
    _config(m_deadActuators, "eyedoctor.deadActuators");
    _config(m_couplingMatrix, "eyedoctor.couplingMatrix");
    _config(m_actuatorGains, "eyedoctor.actuatorGains");
    _config(m_actuatorLimits, "eyedoctor.actuatorLimits");
    
    // DM and Camera Configuration (these will be set by mode resolution or used as overrides)
    std::string overrideDmDevice, overrideDmShmim;
    _config(overrideDmDevice, "eyedoctor.dmDeviceName");
    _config(overrideDmShmim, "eyedoctor.dmShmimName");
    _config(m_wfsShmimName, "eyedoctor.wfsShmimName");
    _config(m_wfsCameraName, "eyedoctor.wfsCameraName");
    _config(m_wfsDarkShmimName, "eyedoctor.wfsDarkShmimName");
    
    // Modeset Configuration
    std::string modesetStr, modesetNamesStr;
    _config(modesetStr, "eyedoctor.modesets");
    _config(modesetNamesStr, "eyedoctor.modeset_names");
    _config(m_defaultModeSet, "eyedoctor.default_modeset");
    
    // Parse comma-separated modeset files
    if (!modesetStr.empty()) {
        std::stringstream ss(modesetStr);
        std::string item;
        while (std::getline(ss, item, ',')) {
            // Trim whitespace
            item.erase(0, item.find_first_not_of(" \t\n\r"));
            item.erase(item.find_last_not_of(" \t\n\r") + 1);
            if (!item.empty()) {
                m_modesetFiles.push_back(item);
            }
        }
    }
    
    // Parse comma-separated modeset names
    if (!modesetNamesStr.empty()) {
        std::stringstream ss(modesetNamesStr);
        std::string item;
        while (std::getline(ss, item, ',')) {
            // Trim whitespace
            item.erase(0, item.find_first_not_of(" \t\n\r"));
            item.erase(item.find_last_not_of(" \t\n\r") + 1);
            if (!item.empty()) {
                m_modesetNames.push_back(item);
            }
        }
    }

    // Resolve DM names based on control mode
    if (m_dmControlMode == "magaox") {
        // MagAOX App Mode: Use selected app name and map to shmim
        if (!m_selectedDMApp.empty()) {
            m_dmDeviceName = m_selectedDMApp;
            
            // Map app name to corresponding shmim
            if (m_dmDeviceMap.find(m_selectedDMApp) != m_dmDeviceMap.end()) {
                m_dmShmimName = m_dmDeviceMap[m_selectedDMApp].shmimName;
            } else {
                log<text_log>("Unknown DM app: " + m_selectedDMApp + ", using deviceName from config", logPrio::LOG_WARNING);
            }
        }
    } else if (m_dmControlMode == "cacao") {
        // CACAO Shmim Mode: Use selected shmim directly
        if (!m_selectedDMShmim.empty()) {
            m_dmShmimName = m_selectedDMShmim;
            m_dmDeviceName = "";  // Not interfacing with a MagAOX app
            log<text_log>("Using CACAO mode with shmim: " + m_dmShmimName, logPrio::LOG_INFO);
        }
    } else {
        log<text_log>("Invalid dmControlMode: " + m_dmControlMode + " (must be 'magaox' or 'cacao')", logPrio::LOG_ERROR);
        return -1;
    }
    
    // Apply explicit overrides if provided
    // (These can be used for manual configuration or backward compatibility)
    if (!overrideDmDevice.empty()) {
        m_dmDeviceName = overrideDmDevice;
        log<text_log>("DM device name overridden to: " + m_dmDeviceName, logPrio::LOG_INFO);
    }
    if (!overrideDmShmim.empty()) {
        m_dmShmimName = overrideDmShmim;
        log<text_log>("DM shmim name overridden to: " + m_dmShmimName, logPrio::LOG_INFO);
    }

    // Calculate total number of modes from start and end indices
    m_totalModes = m_endModeIndex - m_startModeIndex + 1;
    
    // Configure dmWavefrontControl base class parameters
    // Note: This would require the base class to have update methods
    // For now, we'll set these in the setupINDI method
    
    return 0;
}

void eyeDoctor::loadConfig()
{
    // Call base class loadConfig first
    MagAOXApp<true>::loadConfig();
    
    // Then load our specific configuration
    loadConfigImpl(config);
}

int eyeDoctor::appStartup()
{
    log<text_log>("eyeDoctor::appStartup() - Starting initialization");
    
    // Initialize helper class with DM metadata
    m_wfsHelper.m_dmInfo.name = m_dmDeviceName;
    m_wfsHelper.m_dmInfo.numActuators = m_numActuators;
    m_wfsHelper.m_dmInfo.width = m_gridWidth;
    m_wfsHelper.m_dmInfo.height = m_gridHeight;
    m_wfsHelper.m_dmInfo.actuatorSpacing = 0.4; // From config or default
    m_wfsHelper.m_dmInfo.maxStroke = 5.0; // From config or default
    
    if (m_wfsHelper.initializeDMMetadata() < 0)
    {
        log<software_error>({__FILE__, __LINE__, "Failed to initialize DM metadata"});
        return -1;
    }
    
    // Load modesets
    if (!m_modesetFiles.empty())
    {
        for (size_t i = 0; i < m_modesetFiles.size(); ++i)
        {
            std::string name = (i < m_modesetNames.size()) ? m_modesetNames[i] : "modeset_" + std::to_string(i);
            int result = m_wfsHelper.loadModeSet(m_modesetFiles[i], name);
            if (result < 0)
            {
                log<software_error>({__FILE__, __LINE__, "Failed to load modeset from " + m_modesetFiles[i] + " (error code: " + std::to_string(result) + ")"});
                return -1;
            }
            log<text_log>("Loaded modeset '" + name + "' from " + m_modesetFiles[i]);
        }
        m_wfsHelper.m_defaultModeSet = m_defaultModeSet;
    }
    
    log<text_log>("eyeDoctor::appStartup() - Helper class initialized, registering INDI properties");

    // ========== DM CONTROL MODE PROPERTIES ==========
    
    // DM Control Mode (magaox or cacao)
    m_indiP_dmControlMode = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_dmControlMode.setDevice(this->configName());
    m_indiP_dmControlMode.setName("dmControlMode");
    m_indiP_dmControlMode.setGroup("dm_config");
    m_indiP_dmControlMode.setLabel("DM Control Mode");
    m_indiP_dmControlMode.setPerm(pcf::IndiProperty::ReadWrite);
    m_indiP_dmControlMode.setState(pcf::IndiProperty::Idle);
    m_indiP_dmControlMode.add(pcf::IndiElement("current", m_dmControlMode));
    m_indiP_dmControlMode.add(pcf::IndiElement("target", m_dmControlMode));
    if(registerIndiPropertyNew(m_indiP_dmControlMode, &eyeDoctor::st_newCallBack_m_indiP_dmControlMode) < 0) return -1;
    
    // ========== MAGAOX APP MODE PROPERTIES ==========
    
    // Available MagAOX DM Apps (read-only list as comma-separated string)
    std::string availableAppsStr = "";
    for (size_t i = 0; i < m_availableDMApps.size(); ++i) {
        if (i > 0) availableAppsStr += ",";
        availableAppsStr += m_availableDMApps[i];
    }
    m_indiP_availableDMApps = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_availableDMApps.setDevice(this->configName());
    m_indiP_availableDMApps.setName("availableDMApps");
    m_indiP_availableDMApps.setGroup("dm_config");
    m_indiP_availableDMApps.setLabel("Available MagAOX DM Apps");
    m_indiP_availableDMApps.setPerm(pcf::IndiProperty::ReadOnly);
    m_indiP_availableDMApps.setState(pcf::IndiProperty::Idle);
    m_indiP_availableDMApps.add(pcf::IndiElement("current", availableAppsStr));
    if(registerIndiPropertyReadOnly(m_indiP_availableDMApps) < 0) return -1;
    
    // Selected MagAOX DM App
    m_indiP_selectedDMApp = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_selectedDMApp.setDevice(this->configName());
    m_indiP_selectedDMApp.setName("selectedDMApp");
    m_indiP_selectedDMApp.setGroup("dm_config");
    m_indiP_selectedDMApp.setLabel("Selected MagAOX DM App");
    m_indiP_selectedDMApp.setPerm(pcf::IndiProperty::ReadWrite);
    m_indiP_selectedDMApp.setState(pcf::IndiProperty::Idle);
    m_indiP_selectedDMApp.add(pcf::IndiElement("current", m_selectedDMApp));
    m_indiP_selectedDMApp.add(pcf::IndiElement("target", m_selectedDMApp));
    if(registerIndiPropertyNew(m_indiP_selectedDMApp, &eyeDoctor::st_newCallBack_m_indiP_selectedDMApp) < 0) return -1;
    
    // ========== CACAO SHMIM MODE PROPERTIES ==========
    
    // Available CACAO DM Shmims (read-only list as comma-separated string)
    std::string availableShmimsStr = "";
    for (size_t i = 0; i < m_availableDMShmims.size(); ++i) {
        if (i > 0) availableShmimsStr += ",";
        availableShmimsStr += m_availableDMShmims[i];
    }
    m_indiP_availableDMShmims = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_availableDMShmims.setDevice(this->configName());
    m_indiP_availableDMShmims.setName("availableDMShmims");
    m_indiP_availableDMShmims.setGroup("dm_config");
    m_indiP_availableDMShmims.setLabel("Available CACAO DM Shmims");
    m_indiP_availableDMShmims.setPerm(pcf::IndiProperty::ReadOnly);
    m_indiP_availableDMShmims.setState(pcf::IndiProperty::Idle);
    m_indiP_availableDMShmims.add(pcf::IndiElement("current", availableShmimsStr));
    if(registerIndiPropertyReadOnly(m_indiP_availableDMShmims) < 0) return -1;
    
    // Selected CACAO DM Shmim
    m_indiP_selectedDMShmim = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_selectedDMShmim.setDevice(this->configName());
    m_indiP_selectedDMShmim.setName("selectedDMShmim");
    m_indiP_selectedDMShmim.setGroup("dm_config");
    m_indiP_selectedDMShmim.setLabel("Selected CACAO DM Shmim");
    m_indiP_selectedDMShmim.setPerm(pcf::IndiProperty::ReadWrite);
    m_indiP_selectedDMShmim.setState(pcf::IndiProperty::Idle);
    m_indiP_selectedDMShmim.add(pcf::IndiElement("current", m_selectedDMShmim));
    m_indiP_selectedDMShmim.add(pcf::IndiElement("target", m_selectedDMShmim));
    if(registerIndiPropertyNew(m_indiP_selectedDMShmim, &eyeDoctor::st_newCallBack_m_indiP_selectedDMShmim) < 0) return -1;
    
    // ========== LEGACY PROPERTIES (BACKWARD COMPATIBILITY) ==========
    
    // Determine current selection based on control mode for legacy property
    std::string currentDMSelection = (m_dmControlMode == "magaox") ? m_selectedDMApp : m_selectedDMShmim;
    
    // Legacy: Available DMs (Read-only) - shows current selection
    m_indiP_availableDMs = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_availableDMs.setDevice(this->configName());
    m_indiP_availableDMs.setName("availableDMs");
    m_indiP_availableDMs.setGroup("main");
    m_indiP_availableDMs.setLabel("Current DM (" + m_dmControlMode + " mode)");
    m_indiP_availableDMs.setPerm(pcf::IndiProperty::ReadOnly);
    m_indiP_availableDMs.setState(pcf::IndiProperty::Idle);
    m_indiP_availableDMs.add(pcf::IndiElement("current", currentDMSelection));
    if(registerIndiPropertyReadOnly(m_indiP_availableDMs) < 0) return -1;

    // Legacy: DM Device Selection (redirects to appropriate mode)
    m_indiP_selectedDM = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_selectedDM.setDevice(this->configName());
    m_indiP_selectedDM.setName("selectedDM");
    m_indiP_selectedDM.setGroup("main");
    m_indiP_selectedDM.setLabel("Selected DM");
    m_indiP_selectedDM.setPerm(pcf::IndiProperty::ReadWrite);
    m_indiP_selectedDM.setState(pcf::IndiProperty::Idle);
    m_indiP_selectedDM.add(pcf::IndiElement("current", currentDMSelection));
    m_indiP_selectedDM.add(pcf::IndiElement("target", currentDMSelection));
    if(registerIndiPropertyNew(m_indiP_selectedDM, &eyeDoctor::st_newCallBack_m_indiP_selectedDM) < 0) return -1;
    
    log<text_log>("eyeDoctor::appStartup() - Registered DM control mode properties (mode: " + m_dmControlMode + ")");

    // Setup Camera Selection INDI Property
    m_indiP_selectedCamera = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_selectedCamera.setDevice(this->configName());
    m_indiP_selectedCamera.setName("selectedCamera");
    m_indiP_selectedCamera.setGroup("main");
    m_indiP_selectedCamera.setLabel("Selected Camera");
    m_indiP_selectedCamera.setPerm(pcf::IndiProperty::ReadWrite);
    m_indiP_selectedCamera.setState(pcf::IndiProperty::Idle);
    m_indiP_selectedCamera.add(pcf::IndiElement("current", m_selectedCamera));
    m_indiP_selectedCamera.add(pcf::IndiElement("target", m_selectedCamera));
    if(registerIndiPropertyNew(m_indiP_selectedCamera, &eyeDoctor::st_newCallBack_m_indiP_selectedCamera) < 0) return -1;
    log<text_log>("eyeDoctor::appStartup() - Registered camera selection properties");

    // Setup Mode Range INDI Properties
    this->createStandardIndiNumber(m_indiP_startModeIndex, "startModeIndex", 1, 100, 1, "%d");
    m_indiP_startModeIndex["current"].set(m_startModeIndex);
    m_indiP_startModeIndex["target"].set(m_startModeIndex);
    if(registerIndiPropertyNew(m_indiP_startModeIndex, &eyeDoctor::st_newCallBack_m_indiP_startModeIndex) < 0) return -1;

    this->createStandardIndiNumber(m_indiP_endModeIndex, "endModeIndex", 1, 100, 1, "%d");
    m_indiP_endModeIndex["current"].set(m_endModeIndex);
    m_indiP_endModeIndex["target"].set(m_endModeIndex);
    if(registerIndiPropertyNew(m_indiP_endModeIndex, &eyeDoctor::st_newCallBack_m_indiP_endModeIndex) < 0) return -1;

    // Setup Algorithm Parameters INDI Properties
    this->createStandardIndiNumber(m_indiP_psfCoreRadiusPixels, "psfCoreRadiusPixels", 1.0, 50.0, 0.1, "%0.1f");
    m_indiP_psfCoreRadiusPixels["current"].set(m_psfCoreRadiusPixels);
    m_indiP_psfCoreRadiusPixels["target"].set(m_psfCoreRadiusPixels);
    if(registerIndiPropertyNew(m_indiP_psfCoreRadiusPixels, &eyeDoctor::st_newCallBack_m_indiP_psfCoreRadiusPixels) < 0) return -1;

    // Register searchRange property
    this->createStandardIndiNumber(m_indiP_searchRange, "searchRange", 0.1, 10.0, 0.1, "%0.3f");
    m_indiP_searchRange["current"].set(m_searchRange);
    m_indiP_searchRange["target"].set(m_searchRange);
    if(registerIndiPropertyNew(m_indiP_searchRange, &eyeDoctor::st_newCallBack_m_indiP_searchRange) < 0) return -1;

    this->createStandardIndiNumber(m_indiP_nSteps, "nSteps", 5, 100, 1, "%d");
    m_indiP_nSteps["current"].set(m_nSteps);
    m_indiP_nSteps["target"].set(m_nSteps);
    if(registerIndiPropertyNew(m_indiP_nSteps, &eyeDoctor::st_newCallBack_m_indiP_nSteps) < 0) return -1;

    this->createStandardIndiNumber(m_indiP_nRepeats, "nRepeats", 1, 10, 1, "%d");
    m_indiP_nRepeats["current"].set(m_nRepeats);
    m_indiP_nRepeats["target"].set(m_nRepeats);
    if(registerIndiPropertyNew(m_indiP_nRepeats, &eyeDoctor::st_newCallBack_m_indiP_nRepeats) < 0) return -1;

    log<text_log>("About to create nImages property");
    this->createStandardIndiNumber(m_indiP_nImages, "nImages", 1, 10, 1, "%d");
    log<text_log>("Created nImages property - device: " + m_indiP_nImages.getDevice() + ", name: " + m_indiP_nImages.getName());
    m_indiP_nImages["current"].set(m_nImages);
    m_indiP_nImages["target"].set(m_nImages);
    log<text_log>("Set nImages current=" + std::to_string(m_nImages) + ", about to register");
    if(registerIndiPropertyNew(m_indiP_nImages, &eyeDoctor::st_newCallBack_m_indiP_nImages) < 0) {
        log<software_error>({__FILE__, __LINE__, "Failed to register nImages"});
        return -1;
    }
    log<text_log>("Successfully registered nImages with key: " + m_indiP_nImages.createUniqueKey());
    
    this->createStandardIndiNumber(m_indiP_nClusterRepeats, "nClusterRepeats", 1, 10, 1, "%d");
    m_indiP_nClusterRepeats["current"].set(m_nClusterRepeats);
    m_indiP_nClusterRepeats["target"].set(m_nClusterRepeats);
    if(registerIndiPropertyNew(m_indiP_nClusterRepeats, &eyeDoctor::st_newCallBack_m_indiP_nClusterRepeats) < 0) return -1;
    
    this->createStandardIndiNumber(m_indiP_nSeqRepeat, "nSeqRepeat", 1, 10, 1, "%d");
    m_indiP_nSeqRepeat["current"].set(m_nSeqRepeat);
    m_indiP_nSeqRepeat["target"].set(m_nSeqRepeat);
    if(registerIndiPropertyNew(m_indiP_nSeqRepeat, &eyeDoctor::st_newCallBack_m_indiP_nSeqRepeat) < 0) return -1;
    
    this->createStandardIndiNumber(m_indiP_cenX, "cenX", 0, 1000, 1, "%d");
    m_indiP_cenX["current"].set(m_cenX);
    m_indiP_cenX["target"].set(m_cenX);
    if(registerIndiPropertyNew(m_indiP_cenX, &eyeDoctor::st_newCallBack_m_indiP_cenX) < 0) return -1;
    
    this->createStandardIndiNumber(m_indiP_cenY, "cenY", 0, 1000, 1, "%d");
    m_indiP_cenY["current"].set(m_cenY);
    m_indiP_cenY["target"].set(m_cenY);
    if(registerIndiPropertyNew(m_indiP_cenY, &eyeDoctor::st_newCallBack_m_indiP_cenY) < 0) return -1;
    
    this->createStandardIndiNumber(m_indiP_skipFrames, "skipFrames", 0, 100, 1, "%d");
    m_indiP_skipFrames["current"].set(m_skipFrames);
    m_indiP_skipFrames["target"].set(m_skipFrames);
    if(registerIndiPropertyNew(m_indiP_skipFrames, &eyeDoctor::st_newCallBack_m_indiP_skipFrames) < 0) return -1;
    
    this->createStandardIndiToggleSw(m_indiP_resetToZero, "resetToZero", "Reset To Zero Before Optimization");
    if(registerIndiPropertyNew(m_indiP_resetToZero, &eyeDoctor::st_newCallBack_m_indiP_resetToZero) < 0) return -1;
    
    this->createStandardIndiToggleSw(m_indiP_ignoreFocus, "ignoreFocus", "Ignore Focus Mode");
    if(registerIndiPropertyNew(m_indiP_ignoreFocus, &eyeDoctor::st_newCallBack_m_indiP_ignoreFocus) < 0) return -1;

    // Setup eyeDoctor-specific INDI properties
    // Available Cameras (comma-separated list)
    std::string cameraListStr = "";
    for (size_t i = 0; i < m_availableCameras.size(); ++i) {
        if (i > 0) cameraListStr += ",";
        cameraListStr += m_availableCameras[i];
    }
    m_indiP_availableCameras = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_availableCameras.setDevice(this->configName());
    m_indiP_availableCameras.setName("availableCameras");
    m_indiP_availableCameras.setGroup("main");
    m_indiP_availableCameras.setLabel("Available Cameras");
    m_indiP_availableCameras.setPerm(pcf::IndiProperty::ReadOnly);
    m_indiP_availableCameras.setState(pcf::IndiProperty::Idle);
    m_indiP_availableCameras.add(pcf::IndiElement("current", cameraListStr));
    if(registerIndiPropertyReadOnly(m_indiP_availableCameras) < 0) return -1;

    m_indiP_results = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_results.setDevice(this->configName());
    m_indiP_results.setName("results");
    m_indiP_results.setGroup("main");
    m_indiP_results.setLabel("Results");
    m_indiP_results.setPerm(pcf::IndiProperty::ReadOnly);
    m_indiP_results.setState(pcf::IndiProperty::Idle);
    m_indiP_results.add(pcf::IndiElement("current", "No results yet"));
    if(registerIndiPropertyReadOnly(m_indiP_results) < 0) return -1;

    m_indiP_optimizationStatus = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_optimizationStatus.setDevice(this->configName());
    m_indiP_optimizationStatus.setName("optimizationStatus");
    m_indiP_optimizationStatus.setGroup("main");
    m_indiP_optimizationStatus.setLabel("Optimization Status");
    m_indiP_optimizationStatus.setPerm(pcf::IndiProperty::ReadOnly);
    m_indiP_optimizationStatus.setState(pcf::IndiProperty::Idle);
    m_indiP_optimizationStatus.add(pcf::IndiElement("current", "Idle"));
    if(registerIndiPropertyReadOnly(m_indiP_optimizationStatus) < 0) return -1;



    // Create algorithm-specific INDI properties
    this->createStandardIndiNumber(m_indiP_targetLatency, "targetLatency", 100, 10000, 100, "%0.0f");
    m_indiP_targetLatency["current"].set(m_targetLatency);
    m_indiP_targetLatency["target"].set(m_targetLatency);
    
    this->createStandardIndiToggleSw(m_indiP_autoOptimizeLatency, "autoOptimizeLatency", "Auto Optimize Latency");
    
    this->createStandardIndiToggleSw(m_indiP_runOptimization, "runOptimization", "Run Eye Doctor Optimization");
    m_indiP_runOptimization.setState(pcf::IndiProperty::Idle);

    // Register algorithm-specific INDI properties
    if(this->registerIndiPropertyNew(m_indiP_targetLatency, &eyeDoctor::st_newCallBack_m_indiP_targetLatency) < 0) return -1;
    if(this->registerIndiPropertyNew(m_indiP_autoOptimizeLatency, &eyeDoctor::st_newCallBack_m_indiP_autoOptimizeLatency) < 0) return -1;
    if(this->registerIndiPropertyNew(m_indiP_runOptimization, &eyeDoctor::st_newCallBack_m_indiP_runOptimization) < 0) return -1;
    log<text_log>("eyeDoctor::appStartup() - Registered all algorithm parameter properties");
    
    // Setup DM Metadata INDI properties
    this->createStandardIndiNumber(m_indiP_numActuators, "numActuators", 1, 100000, 1, "%d");
    m_indiP_numActuators["current"].set(m_numActuators);
    m_indiP_numActuators["target"].set(m_numActuators);
    if(registerIndiPropertyNew(m_indiP_numActuators, &eyeDoctor::st_newCallBack_m_indiP_numActuators) < 0) return -1;

    this->createStandardIndiNumber(m_indiP_gridWidth, "gridWidth", 1, 1000, 1, "%d");
    m_indiP_gridWidth["current"].set(m_gridWidth);
    m_indiP_gridWidth["target"].set(m_gridWidth);
    if(registerIndiPropertyNew(m_indiP_gridWidth, &eyeDoctor::st_newCallBack_m_indiP_gridWidth) < 0) return -1;

    this->createStandardIndiNumber(m_indiP_gridHeight, "gridHeight", 1, 1000, 1, "%d");
    m_indiP_gridHeight["current"].set(m_gridHeight);
    m_indiP_gridHeight["target"].set(m_gridHeight);
    if(registerIndiPropertyNew(m_indiP_gridHeight, &eyeDoctor::st_newCallBack_m_indiP_gridHeight) < 0) return -1;

    m_indiP_dmType = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_dmType.setDevice(this->configName());
    m_indiP_dmType.setName("dmType");
    m_indiP_dmType.setGroup("dmMetadata");
    m_indiP_dmType.setLabel("DM Type");
    m_indiP_dmType.setPerm(pcf::IndiProperty::ReadWrite);
    m_indiP_dmType.setState(pcf::IndiProperty::Idle);
    m_indiP_dmType.add(pcf::IndiElement("current", m_dmType));
    m_indiP_dmType.add(pcf::IndiElement("target", m_dmType));
    if(registerIndiPropertyNew(m_indiP_dmType, &eyeDoctor::st_newCallBack_m_indiP_dmType) < 0) return -1;

    m_indiP_deadActuators = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_deadActuators.setDevice(this->configName());
    m_indiP_deadActuators.setName("deadActuators");
    m_indiP_deadActuators.setGroup("dmMetadata");
    m_indiP_deadActuators.setLabel("Dead Actuators");
    m_indiP_deadActuators.setPerm(pcf::IndiProperty::ReadWrite);
    m_indiP_deadActuators.setState(pcf::IndiProperty::Idle);
    std::string deadActuatorsStr = "";
    for(size_t i = 0; i < m_deadActuators.size(); ++i) {
        if(i > 0) deadActuatorsStr += ",";
        deadActuatorsStr += std::to_string(m_deadActuators[i]);
    }
    m_indiP_deadActuators.add(pcf::IndiElement("current", deadActuatorsStr));
    m_indiP_deadActuators.add(pcf::IndiElement("target", deadActuatorsStr));
    if(registerIndiPropertyNew(m_indiP_deadActuators, &eyeDoctor::st_newCallBack_m_indiP_deadActuators) < 0) return -1;

    m_indiP_couplingMatrix = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_couplingMatrix.setDevice(this->configName());
    m_indiP_couplingMatrix.setName("couplingMatrix");
    m_indiP_couplingMatrix.setGroup("dmMetadata");
    m_indiP_couplingMatrix.setLabel("Coupling Matrix File");
    m_indiP_couplingMatrix.setPerm(pcf::IndiProperty::ReadWrite);
    m_indiP_couplingMatrix.setState(pcf::IndiProperty::Idle);
    m_indiP_couplingMatrix.add(pcf::IndiElement("current", m_couplingMatrix));
    m_indiP_couplingMatrix.add(pcf::IndiElement("target", m_couplingMatrix));
    if(registerIndiPropertyNew(m_indiP_couplingMatrix, &eyeDoctor::st_newCallBack_m_indiP_couplingMatrix) < 0) return -1;

    m_indiP_actuatorGains = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_actuatorGains.setDevice(this->configName());
    m_indiP_actuatorGains.setName("actuatorGains");
    m_indiP_actuatorGains.setGroup("dmMetadata");
    m_indiP_actuatorGains.setLabel("Actuator Gains File");
    m_indiP_actuatorGains.setPerm(pcf::IndiProperty::ReadWrite);
    m_indiP_actuatorGains.setState(pcf::IndiProperty::Idle);
    m_indiP_actuatorGains.add(pcf::IndiElement("current", m_actuatorGains));
    m_indiP_actuatorGains.add(pcf::IndiElement("target", m_actuatorGains));
    if(registerIndiPropertyNew(m_indiP_actuatorGains, &eyeDoctor::st_newCallBack_m_indiP_actuatorGains) < 0) return -1;

    m_indiP_actuatorLimits = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_actuatorLimits.setDevice(this->configName());
    m_indiP_actuatorLimits.setName("actuatorLimits");
    m_indiP_actuatorLimits.setGroup("dmMetadata");
    m_indiP_actuatorLimits.setLabel("Actuator Limits File");
    m_indiP_actuatorLimits.setPerm(pcf::IndiProperty::ReadWrite);
    m_indiP_actuatorLimits.setState(pcf::IndiProperty::Idle);
    m_indiP_actuatorLimits.add(pcf::IndiElement("current", m_actuatorLimits));
    m_indiP_actuatorLimits.add(pcf::IndiElement("target", m_actuatorLimits));
    if(registerIndiPropertyNew(m_indiP_actuatorLimits, &eyeDoctor::st_newCallBack_m_indiP_actuatorLimits) < 0) return -1;
    
    log<text_log>("eyeDoctor::appStartup() - Registered all DM metadata properties");

    // Configure dmWavefrontControl base class with current settings
    updateDMConfiguration();
    updateCameraConfiguration();
    updateSearchParameters();
    
    // Initialize telemeter
    if(telemeterT::appStartup() < 0)
    {
        return log<software_error,-1>({__FILE__,__LINE__});
    }
    
    log<text_log>("eyeDoctor::appStartup() - All INDI properties registered, starting optimization thread");
    
    // Start optimization thread (waits for toggle to be turned on)
    // Use a dummy INDI property for thread monitoring (not exposed to INDI)
    pcf::IndiProperty dummyThreadProp;
    if(threadStart(m_optimizationThread, m_optimizationThreadInit, m_optimizationThreadPID, 
                   dummyThreadProp, 0, "", "optimization", this, optimizationThreadStart) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__, "Failed to start optimization thread"});
    }
    log<text_log>("eyeDoctor::appStartup() - Optimization thread started successfully");
    
    log<text_log>("eyeDoctor::appStartup() - Complete - all properties registered successfully");
    log<text_log>("eyeDoctor::appStartup() - Properties will be sent to INDI server when communications are initialized");

    return 0;
}

int eyeDoctor::appLogic()
{
    // Send properties to INDI server once when communications are ready
    if (!m_indiPropertiesSent && m_indiDriver != nullptr)
    {
        // Try to send a test property to see if INDI is ready
        // If it works, send all our properties
        if (sendNewProperty(m_indiP_dmControlMode) >= 0)
        {
            // INDI is ready, send all properties (only if they have valid device names, indicating they're initialized)
            // Helper lambda to safely send a property
            auto safeSend = [this](pcf::IndiProperty &prop) {
                if (prop.hasValidDevice() && prop.hasValidName() && prop.getType() != pcf::IndiProperty::Unknown) {
                    return sendNewProperty(prop);
                }
                return 0; // Skip if not properly initialized
            };
            
            // DM Control Mode properties
            safeSend(m_indiP_availableDMApps);
            safeSend(m_indiP_selectedDMApp);
            safeSend(m_indiP_availableDMShmims);
            safeSend(m_indiP_selectedDMShmim);
            
            // Legacy DM properties
            safeSend(m_indiP_availableDMs);
            safeSend(m_indiP_selectedDM);
            
            // Camera properties
            safeSend(m_indiP_availableCameras);
            safeSend(m_indiP_selectedCamera);
            
            // Mode range properties
            safeSend(m_indiP_startModeIndex);
            safeSend(m_indiP_endModeIndex);
            
            // Algorithm parameters
            safeSend(m_indiP_psfCoreRadiusPixels);
            safeSend(m_indiP_searchRange);
            safeSend(m_indiP_nSteps);
            safeSend(m_indiP_nRepeats);
            safeSend(m_indiP_nClusterRepeats);
            safeSend(m_indiP_nSeqRepeat);
            safeSend(m_indiP_nImages);
            safeSend(m_indiP_cenX);
            safeSend(m_indiP_cenY);
            safeSend(m_indiP_skipFrames);
            safeSend(m_indiP_resetToZero);
            safeSend(m_indiP_ignoreFocus);
            
            // Optimization properties
            safeSend(m_indiP_targetLatency);
            safeSend(m_indiP_autoOptimizeLatency);
            safeSend(m_indiP_runOptimization);
            safeSend(m_indiP_optimizationStatus);
            safeSend(m_indiP_results);
            
            // DM metadata properties
            safeSend(m_indiP_numActuators);
            safeSend(m_indiP_gridWidth);
            safeSend(m_indiP_gridHeight);
            safeSend(m_indiP_dmType);
            safeSend(m_indiP_deadActuators);
            safeSend(m_indiP_couplingMatrix);
            safeSend(m_indiP_actuatorGains);
            safeSend(m_indiP_actuatorLimits);
            
            m_indiPropertiesSent = true;
            log<text_log>("eyeDoctor::appLogic() - All properties sent to INDI server");
        }
        // If sendNewProperty returns < 0, INDI not ready yet, will try again next iteration
    }

    return 0;
}

int eyeDoctor::appShutdown()
{
    // Stop optimization if running
    m_optimizationInProgress = false;
    
    // Signal thread to exit by clearing init flag (thread checks shutdown())
    m_optimizationThreadInit = false;
    
    // Wait for thread to finish with timeout
    if(m_optimizationThread.joinable())
    {
        try
        {
            // Give thread a moment to see shutdown flag
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            
            // Try to join with a short timeout approach - check if still running
            for(int i = 0; i < 50 && m_optimizationThread.joinable(); ++i)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            // Final join attempt
            if(m_optimizationThread.joinable())
            {
                m_optimizationThread.join();
            }
        }
        catch(const std::exception& e)
        {
            log<software_error>({__FILE__, __LINE__, "Exception joining optimization thread: " + std::string(e.what())});
        }
        catch(...)
        {
            log<software_error>({__FILE__, __LINE__, "Unknown exception joining optimization thread"});
        }
    }
    
    return 0;
}

// Required interface functions for dmPokeWFS
int eyeDoctor::runWavefrontSensing()
{
    // Run the wavefront sensing process
    {
        // Take dark frame if needed
        m_measurementComplete = false;
    }

    // Run the basic wavefront sensing measurement
    return 0; // Placeholder - implement actual wavefront sensing logic
}

int eyeDoctor::analyzeWavefrontSensing()
{
    // Analyze the poke image and calculate PSF metrics
    if(measurePSF() < 0)
    {
        return -1;
    }

    // Update measurement results
    // Placeholder - actual calculation needed

    m_measurementComplete = true;
    return 0;
}

// Eye doctor specific functions
void eyeDoctor::optimizationThreadStart(eyeDoctor *e)
{
    e->optimizationThreadExec();
}

void eyeDoctor::optimizationThreadExec()
{
    m_optimizationThreadPID = syscall(SYS_gettid);
    
    // Wait for thread initialization to complete
    while(m_optimizationThreadInit == true && shutdown() == 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    log<text_log>("Optimization thread started");
    
    while(shutdown() == 0)
    {
        // Wait for optimization to be enabled via INDI toggle
        if(m_optimizationInProgress)
        {
            log<text_log>("Starting eye doctor optimization...");
            updateOptimizationStatus();
            
            // Calculate total modes to optimize
            m_currentModeIndex = 0;
            m_totalModes = m_endModeIndex - m_startModeIndex + 1;
            
            // Run the optimization algorithm
            int result = runOptimizationAlgorithm();
            
            // Algorithm completed - automatically turn off toggle
            m_optimizationInProgress = false;
            updateIfChanged(m_indiP_runOptimization, "toggle", pcf::IndiElement::Off, pcf::IndiProperty::Idle);
            
            if(result == 0)
            {
                log<text_log>("Optimization completed successfully");
                updateIfChanged(m_indiP_optimizationStatus, "current", std::string("Completed"));
            }
            else
            {
                log<text_log>("Optimization completed with errors");
                updateIfChanged(m_indiP_optimizationStatus, "current", std::string("Error"));
            }
            
            updateOptimizationStatus();
        }
        
        // Sleep for a bit before checking again
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    log<text_log>("Optimization thread exiting");
}

int eyeDoctor::runOptimizationAlgorithm()
{
    // Main optimization loop - runs through all modes
    try
    {
        // Check if we should optimize focus first (unless ignoreFocus is set)
        int startMode = m_startModeIndex;
        if(!m_ignoreFocus && m_startModeIndex == 1)
        {
            // Mode 1 is typically focus, start there
            startMode = 1;
        }
        
        for(int modeIdx = startMode; modeIdx <= m_endModeIndex && m_optimizationInProgress; ++modeIdx)
        {
            m_currentModeIndex = modeIdx;
            updateResults();
            
            log<text_log>("Optimizing mode " + std::to_string(modeIdx));
            
            // Calculate search bounds
            std::pair<double, double> bounds = {-m_searchRange/2.0, m_searchRange/2.0};
            
            // TODO: Implement grid sweep optimization with image collection callback
            // The helper class needs a callback function to collect images
            /*
            auto imageCollector = [this](double amplitude) -> std::vector<eigenImage<float>> {
                // TODO: Apply mode amplitude to DM via shmim
                // TODO: Collect m_nImages from WFS camera via shmim
                // Return collected images
                return std::vector<eigenImage<float>>();
            };
            
            double optimalValue = m_wfsHelper.gridSweepOptimization(
                bounds,
                m_nSteps,
                m_nRepeats,
                imageCollector,
                "coreSum",
                {{"radius", m_psfCoreRadiusPixels}, {"cenX", m_cenX}, {"cenY", m_cenY}},
                "fit"
            );
            */
            
            double optimalValue = 0.0; // Placeholder
            log<text_log>("Mode " + std::to_string(modeIdx) + " optimization skipped (not yet implemented)");
            
            // Small delay between modes
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        return 0;
    }
    catch(const std::exception& e)
    {
        log<software_error>({__FILE__, __LINE__, "Optimization error: " + std::string(e.what())});
        return -1;
    }
}

int eyeDoctor::measurePSF()
{
    // Placeholder for PSF measurement logic
    // This would analyze the current image and extract PSF metrics
    return 0;
}

int eyeDoctor::optimizeMode(int modeIndex)
{
    // Placeholder for mode optimization logic
    // This would apply the mode, measure PSF, and optimize coefficients
    (void)modeIndex; // Suppress unused parameter warning
    return 0;
}

int eyeDoctor::calculateOptimizationResult()
{
    // Placeholder for result calculation
    return 0;
}

int eyeDoctor::updateOptimizationStatus()
{
    std::string status = m_optimizationInProgress ? "Running" : "Idle";
    m_indiP_optimizationStatus["current"] = status;
    updateIfChanged(m_indiP_optimizationStatus, "current", status);
    return 0;
}

int eyeDoctor::updateResults()
{
    updateIfChanged(m_indiP_results, "progress", (double)m_currentModeIndex / m_totalModes);
    updateIfChanged(m_indiP_results, "currentMode", m_currentModeIndex);
    return 0;
}

// Dynamic configuration methods for dmWavefrontControl
int eyeDoctor::updateDMConfiguration()
{
    // This method will update the dmWavefrontControl parameters based on INDI properties
    // For example, if m_dmDeviceName or m_dmShmimName changes, update the dmWavefrontControl
    // This is a placeholder and would require actual implementation of dmWavefrontControl::updateConfig
    return 0;
}

int eyeDoctor::updateCameraConfiguration()
{
    // This method will update the camera selection based on INDI properties
    // For example, if m_selectedCamera changes, update the dmWavefrontControl
    // This is a placeholder and would require actual implementation of dmWavefrontControl::updateConfig
    return 0;
}

int eyeDoctor::updateSearchParameters()
{
    // This method will update the search parameters (psfCoreRadiusPixels, searchRange, nSteps)
    // based on INDI properties.
    // This is a placeholder and would require actual implementation of dmWavefrontControl::updateConfig
    return 0;
}




// Telemeter interface
int eyeDoctor::recordTelem(const telem_pokeloop *)
{
    return recordEyeDoctor(true);
}

int eyeDoctor::recordEyeDoctor(bool force)
{
    // Placeholder for telemetry recording
    (void)force; // Suppress unused parameter warning
    return 0;
}

// Callback definitions for dmWavefrontControl properties

// Old base class callbacks removed - these properties no longer exist

// ========== DM CONTROL MODE CALLBACKS ==========

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_dmControlMode)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_dmControlMode, ipRecv)
   
    std::string target;
    if(indiTargetUpdate(m_indiP_dmControlMode, target, ipRecv, true) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    // Validate mode
    if (target != "magaox" && target != "cacao") {
        log<text_log>("Invalid dmControlMode: " + target + " (must be 'magaox' or 'cacao')", logPrio::LOG_ERROR);
        return -1;
    }
    
    m_dmControlMode = target;
    log<text_log>("DM control mode changed to: " + m_dmControlMode, logPrio::LOG_INFO);
    
    // Update current value
    updateIfChanged(m_indiP_dmControlMode, "current", m_dmControlMode);
    
    // Re-resolve internal DM names based on new mode
    if (m_dmControlMode == "magaox") {
        if (m_dmDeviceMap.find(m_selectedDMApp) != m_dmDeviceMap.end()) {
            m_dmShmimName = m_dmDeviceMap[m_selectedDMApp].shmimName;
        }
    } else {
        m_dmShmimName = m_selectedDMShmim;
    }
    
    updateDMConfiguration();
    
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_selectedDMApp)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_selectedDMApp, ipRecv)
   
    std::string target;
    if(indiTargetUpdate(m_indiP_selectedDMApp, target, ipRecv, true) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    m_selectedDMApp = target;
    log<text_log>("Selected MagAOX DM app: " + m_selectedDMApp, logPrio::LOG_INFO);
    
    // Update current value
    updateIfChanged(m_indiP_selectedDMApp, "current", m_selectedDMApp);
    
    // Only resolve internal names if in magaox mode
    if (m_dmControlMode == "magaox") {
        if (m_dmDeviceMap.find(target) != m_dmDeviceMap.end()) {
            m_dmShmimName = m_dmDeviceMap[target].shmimName;
        }
        updateDMConfiguration();
    }
    
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_selectedDMShmim)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_selectedDMShmim, ipRecv)
   
    std::string target;
    if(indiTargetUpdate(m_indiP_selectedDMShmim, target, ipRecv, true) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    m_selectedDMShmim = target;
    log<text_log>("Selected CACAO DM shmim: " + m_selectedDMShmim, logPrio::LOG_INFO);
    
    // Update current value
    updateIfChanged(m_indiP_selectedDMShmim, "current", m_selectedDMShmim);
    
    // Only resolve internal names if in cacao mode
    if (m_dmControlMode == "cacao") {
        updateDMConfiguration();
    }
    
    return 0;
}

// ========== LEGACY CALLBACK (BACKWARD COMPATIBILITY) ==========

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_selectedDM)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_selectedDM, ipRecv)
   
    std::string target;
    if(indiTargetUpdate(m_indiP_selectedDM, target, ipRecv, true) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    // Update the selected DM based on control mode
    if (m_dmControlMode == "magaox") {
        m_selectedDMApp = target;
        // Resolve shmim name from app mapping
        if (m_dmDeviceMap.find(target) != m_dmDeviceMap.end()) {
            m_dmShmimName = m_dmDeviceMap[target].shmimName;
        }
        // Update the selectedDMApp property current value
        updateIfChanged(m_indiP_selectedDMApp, "current", m_selectedDMApp);
    } else if (m_dmControlMode == "cacao") {
        m_selectedDMShmim = target;
        // Update the selectedDMShmim property current value
        updateIfChanged(m_indiP_selectedDMShmim, "current", m_selectedDMShmim);
    }
    
    // Update legacy property current value
    updateIfChanged(m_indiP_selectedDM, "current", target);
    
    updateDMConfiguration();
    
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_selectedCamera)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_selectedCamera, ipRecv)
   
    std::string target;
    if(indiTargetUpdate(m_indiP_selectedCamera, target, ipRecv, true) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    // Update the selected camera and reconfigure
    m_selectedCamera = target;
    
    // Update current value
    updateIfChanged(m_indiP_selectedCamera, "current", m_selectedCamera);
    
    updateCameraConfiguration();
    
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_startModeIndex)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_startModeIndex, ipRecv)
   
    int target;
    if(indiTargetUpdate(m_indiP_startModeIndex, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    // Update the start mode index and recalculate total modes
    m_startModeIndex = target;
    m_totalModes = m_endModeIndex - m_startModeIndex + 1;
    
    updateIfChanged(m_indiP_startModeIndex, "current", m_startModeIndex);
    
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_endModeIndex)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_endModeIndex, ipRecv)
   
    int target;
    if(indiTargetUpdate(m_indiP_endModeIndex, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    // Update the end mode index and recalculate total modes
    m_endModeIndex = target;
    m_totalModes = m_endModeIndex - m_startModeIndex + 1;
    
    updateIfChanged(m_indiP_endModeIndex, "current", m_endModeIndex);
    
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_psfCoreRadiusPixels)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_psfCoreRadiusPixels, ipRecv)
   
    float target;
    if(indiTargetUpdate(m_indiP_psfCoreRadiusPixels, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    // Update the PSF core radius and reconfigure
    m_psfCoreRadiusPixels = target;
    updateSearchParameters();
    
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_nSteps)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_nSteps, ipRecv)
   
    int target;
    if(indiTargetUpdate(m_indiP_nSteps, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    // Update the number of steps
    m_nSteps = target;
    
    updateIfChanged(m_indiP_nSteps, "current", m_nSteps);
    
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_nRepeats)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_nRepeats, ipRecv)
   
    int target;
    if(indiTargetUpdate(m_indiP_nRepeats, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    // Update the number of repeats
    m_nRepeats = target;
    
    updateIfChanged(m_indiP_nRepeats, "current", m_nRepeats);
    
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_nImages)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_nImages, ipRecv)
   
    int target;
    if(indiTargetUpdate(m_indiP_nImages, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    // Update the number of images
    m_nImages = target;
    
    updateIfChanged(m_indiP_nImages, "current", m_nImages);
    
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_nClusterRepeats)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_nClusterRepeats, ipRecv)
   
    int target;
    if(indiTargetUpdate(m_indiP_nClusterRepeats, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    m_nClusterRepeats = target;
    
    updateIfChanged(m_indiP_nClusterRepeats, "current", m_nClusterRepeats);
    
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_nSeqRepeat)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_nSeqRepeat, ipRecv)
   
    int target;
    if(indiTargetUpdate(m_indiP_nSeqRepeat, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    m_nSeqRepeat = target;
    
    updateIfChanged(m_indiP_nSeqRepeat, "current", m_nSeqRepeat);
    
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_cenX)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_cenX, ipRecv)
   
    int target;
    if(indiTargetUpdate(m_indiP_cenX, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    m_cenX = target;
    
    updateIfChanged(m_indiP_cenX, "current", m_cenX);
    
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_cenY)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_cenY, ipRecv)
   
    int target;
    if(indiTargetUpdate(m_indiP_cenY, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    m_cenY = target;
    
    updateIfChanged(m_indiP_cenY, "current", m_cenY);
    
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_skipFrames)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_skipFrames, ipRecv)
   
    int target;
    if(indiTargetUpdate(m_indiP_skipFrames, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    m_skipFrames = target;
    
    updateIfChanged(m_indiP_skipFrames, "current", m_skipFrames);
    
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_resetToZero)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_resetToZero, ipRecv)
   
    if(ipRecv.find("toggle") == 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__, "toggle element not found"});
    }

    m_resetToZero = (ipRecv["toggle"].getSwitchState() == pcf::IndiElement::On);
    updateSwitchIfChanged(m_indiP_resetToZero, "toggle", m_resetToZero ? pcf::IndiElement::On : pcf::IndiElement::Off);
    
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_ignoreFocus)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_ignoreFocus, ipRecv)
   
    if(ipRecv.find("toggle") == 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__, "toggle element not found"});
    }

    m_ignoreFocus = (ipRecv["toggle"].getSwitchState() == pcf::IndiElement::On);
    updateSwitchIfChanged(m_indiP_ignoreFocus, "toggle", m_ignoreFocus ? pcf::IndiElement::On : pcf::IndiElement::Off);
    
    return 0;
}

// searchRange callback
INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_searchRange)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_searchRange, ipRecv)
   
    float target;
    if(indiTargetUpdate(m_indiP_searchRange, target, ipRecv, true) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }
   
    m_searchRange = target;
    updateIfChanged(m_indiP_searchRange, "current", m_searchRange);
   
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_targetLatency)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_targetLatency, ipRecv)
   
    float target;
    if(indiTargetUpdate(m_indiP_targetLatency, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    // Update the dmWavefrontControl property
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_autoOptimizeLatency)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_autoOptimizeLatency, ipRecv)
   
    m_autoOptimizeLatency = (ipRecv["toggle"].getSwitchState() == pcf::IndiElement::On);
    updateSwitchIfChanged(m_indiP_autoOptimizeLatency, "toggle", m_autoOptimizeLatency ? pcf::IndiElement::On : pcf::IndiElement::Off);
    
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_optimizationStatus)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_optimizationStatus, ipRecv)
   
    // This is a read-only property, so we don't need to update anything
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_runOptimization)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_runOptimization, ipRecv)
   
    std::lock_guard<std::mutex> lock(m_indiMutex);
    
    bool current_state = ipRecv["toggle"].getSwitchState() == pcf::IndiElement::On;
    bool previous_state = m_optimizationInProgress;
    
    if(current_state && !previous_state)
    {
        // Toggled ON - start optimization
        if(m_optimizationInProgress)
        {
            log<text_log>("Optimization already in progress");
            return 0;
        }
        
        // Validate parameters before starting
        if(m_startModeIndex < 1 || m_endModeIndex < m_startModeIndex)
        {
            log<software_error>({__FILE__, __LINE__, "Invalid mode range: " + std::to_string(m_startModeIndex) + " to " + std::to_string(m_endModeIndex)});
            updateIfChanged(m_indiP_runOptimization, "toggle", pcf::IndiElement::Off, pcf::IndiProperty::Alert);
            return -1;
        }
        
        m_optimizationInProgress = true;
        updateIfChanged(m_indiP_runOptimization, "toggle", pcf::IndiElement::On, pcf::IndiProperty::Busy);
        log<text_log>("Optimization started: modes " + std::to_string(m_startModeIndex) + " to " + std::to_string(m_endModeIndex));
    }
    else if(!current_state && previous_state)
    {
        // Toggled OFF - stop optimization
        m_optimizationInProgress = false;
        updateIfChanged(m_indiP_runOptimization, "toggle", pcf::IndiElement::Off, pcf::IndiProperty::Idle);
        log<text_log>("Optimization stopped by user");
    }
    
    return 0;
}

// DM Metadata callbacks
INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_numActuators)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_numActuators, ipRecv)
   
    int target;
    if(indiTargetUpdate(m_indiP_numActuators, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    m_numActuators = target;
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_gridWidth)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_gridWidth, ipRecv)
   
    int target;
    if(indiTargetUpdate(m_indiP_gridWidth, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    m_gridWidth = target;
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_gridHeight)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_gridHeight, ipRecv)
   
    int target;
    if(indiTargetUpdate(m_indiP_gridHeight, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    m_gridHeight = target;
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_dmType)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_dmType, ipRecv)
   
    std::string target;
    if(indiTargetUpdate(m_indiP_dmType, target, ipRecv, true) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    m_dmType = target;
    updateIfChanged(m_indiP_dmType, "current", m_dmType);
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_deadActuators)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_deadActuators, ipRecv)
   
    std::string target;
    if(indiTargetUpdate(m_indiP_deadActuators, target, ipRecv, true) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    // Parse comma-separated list of actuator indices
    m_deadActuators.clear();
    if(!target.empty()) {
        std::istringstream iss(target);
        std::string token;
        while(std::getline(iss, token, ',')) {
            if(!token.empty()) {
                try {
                    m_deadActuators.push_back(std::stoi(token));
                } catch(...) {
                    // Skip invalid entries
                }
            }
        }
    }
    
    // Reconstruct the string to update current value
    std::string deadActuatorsStr = "";
    for(size_t i = 0; i < m_deadActuators.size(); ++i) {
        if(i > 0) deadActuatorsStr += ",";
        deadActuatorsStr += std::to_string(m_deadActuators[i]);
    }
    updateIfChanged(m_indiP_deadActuators, "current", deadActuatorsStr);
    
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_couplingMatrix)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_couplingMatrix, ipRecv)
   
    std::string target;
    if(indiTargetUpdate(m_indiP_couplingMatrix, target, ipRecv, true) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    m_couplingMatrix = target;
    updateIfChanged(m_indiP_couplingMatrix, "current", m_couplingMatrix);
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_actuatorGains)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_actuatorGains, ipRecv)
   
    std::string target;
    if(indiTargetUpdate(m_indiP_actuatorGains, target, ipRecv, true) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    m_actuatorGains = target;
    updateIfChanged(m_indiP_actuatorGains, "current", m_actuatorGains);
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_actuatorLimits)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_actuatorLimits, ipRecv)
   
    std::string target;
    if(indiTargetUpdate(m_indiP_actuatorLimits, target, ipRecv, true) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    m_actuatorLimits = target;
    updateIfChanged(m_indiP_actuatorLimits, "current", m_actuatorLimits);
    return 0;
}

} //namespace app
} //namespace MagAOX

#endif //eyeDoctor_hpp
