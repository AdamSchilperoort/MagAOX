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
                 public dev::dmWavefrontControl<eyeDoctor>,
                 public dev::telemeter<eyeDoctor>
{
    // Give the test harness access.
    friend class eyeDoctor_test;

    friend class dev::dmWavefrontControl<eyeDoctor>;
    
    typedef dev::dmWavefrontControl<eyeDoctor> dmWavefrontControlT;

    friend class dev::telemeter<eyeDoctor>;
    typedef dev::telemeter<eyeDoctor> telemeterT;

protected:
    /** \name Configurable Parameters
      *@{
     */
    
    // DM Device Selection
    std::vector<std::string> m_availableDMs;      ///< List of available DM devices
    std::string m_selectedDM;                     ///< Currently selected DM device
    std::string m_dmDeviceName;                   ///< Actual DM device name (dmwoofer, dmncpc, dmtweeter, dmkilo)
    std::string m_dmShmimName;                    ///< DM shared memory name (dm00disp, dm01disp, dm02disp)
    
    // Camera Selection
    std::vector<std::string> m_availableCameras;  ///< List of available cameras
    std::string m_selectedCamera;                 ///< Currently selected camera for wavefront sensing
    
    // Mode Specification
    int m_startModeIndex;                         ///< Starting mode index to optimize
    int m_endModeIndex;                           ///< Ending mode index to optimize
    
    // Algorithm Parameters (from console_comprehensive)
    double m_psfCoreRadiusPixels;        ///< Radius of the PSF core to measure
    double m_searchRange;                ///< Range of values in microns for grid search
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
    {}

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
    int startOptimization();
    int stopOptimization();
    int measurePSF();
    int optimizeMode(int modeIndex);
    int calculateOptimizationResult();

protected:
    /** \name INDI Interface
      * @{
      */

    // DM Device Selection
    pcf::IndiProperty m_indiP_availableDMs;
    pcf::IndiProperty m_indiP_selectedDM;
    pcf::IndiProperty m_indiP_dmDeviceName;
    pcf::IndiProperty m_indiP_dmShmimName;

    // Camera Selection
    pcf::IndiProperty m_indiP_availableCameras;
    pcf::IndiProperty m_indiP_selectedCamera;

    // Mode Specification
    pcf::IndiProperty m_indiP_startModeIndex;
    pcf::IndiProperty m_indiP_endModeIndex;

    // Algorithm Parameters
    pcf::IndiProperty m_indiP_psfCoreRadiusPixels;
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
    pcf::IndiProperty m_indiP_startOptimization;
    pcf::IndiProperty m_indiP_stopOptimization;

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

    // Callback declarations for dmWavefrontControl properties
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_dmPokeAmplitude);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_dmPokeDelay);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_wfsCamera);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_psfCoreRadius);

    // Callback declarations for eyeDoctor-specific properties
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_availableDMs);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_selectedDM);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_selectedCamera);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_startModeIndex);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_endModeIndex);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_psfCoreRadiusPixels);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_searchRange);
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
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_startOptimization);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_stopOptimization);
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
    // Initialize DM device parameters
    m_availableDMs = {"wooferModes", "ncpcModes", "tweeterModes", "kiloModes"};
    m_selectedDM = "tweeterModes";
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
    
    // Initialize DM metadata parameters
    m_numActuators = 4096;
    m_gridWidth = 64;
    m_gridHeight = 64;
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
    
    // Initialize DM device mapping
    m_dmDeviceMap = {
        {"wooferModes", {"dmwoofer", "dm00disp", "DM Woofer"}},
        {"ncpcModes", {"dmncpc", "dm02disp", "DM NCPc"}},
        {"tweeterModes", {"dmtweeter", "dm01disp", "DM Tweeter"}},
        {"kiloModes", {"dmkilo", "dm00disp", "DM Kilo"}}
    };
    
    return;
}

void eyeDoctor::setupConfig()
{
    // Call base class setupConfig first
    MagAOXApp<true>::setupConfig();
    
    DMWAVEFRONTCONTROL_SETUP_CONFIG(config);

    // DM Device Selection
    config.add("eyedoctor.availableDMs", "", "eyedoctor.availableDMs", argType::Required, "eyedoctor", "availableDMs", false, "vector<string>", "List of available DM devices");
    config.add("eyedoctor.selectedDM", "", "eyedoctor.selectedDM", argType::Required, "eyedoctor", "selectedDM", false, "string", "Selected DM device");
    
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
    
    // DM Metadata Parameters
    config.add("eyedoctor.numActuators", "", "eyedoctor.numActuators", argType::Required, "eyedoctor", "numActuators", false, "int", "Number of actuators in the DM");
    config.add("eyedoctor.gridWidth", "", "eyedoctor.gridWidth", argType::Required, "eyedoctor", "gridWidth", false, "int", "Actuator grid width");
    config.add("eyedoctor.gridHeight", "", "eyedoctor.gridHeight", argType::Required, "eyedoctor", "gridHeight", false, "int", "Actuator grid height");
    config.add("eyedoctor.dmType", "", "eyedoctor.dmType", argType::Required, "eyedoctor", "dmType", false, "string", "DM type/manufacturer");
    config.add("eyedoctor.deadActuators", "", "eyedoctor.deadActuators", argType::Required, "eyedoctor", "deadActuators", false, "vector<int>", "List of dead actuator indices (comma-separated)");
    config.add("eyedoctor.couplingMatrix", "", "eyedoctor.couplingMatrix", argType::Required, "eyedoctor", "couplingMatrix", false, "string", "Path to coupling matrix file (optional)");
    config.add("eyedoctor.actuatorGains", "", "eyedoctor.actuatorGains", argType::Required, "eyedoctor", "actuatorGains", false, "string", "Path to actuator gains file (optional)");
    config.add("eyedoctor.actuatorLimits", "", "eyedoctor.actuatorLimits", argType::Required, "eyedoctor", "actuatorLimits", false, "string", "Path to actuator limits file (optional)");
}

int eyeDoctor::loadConfigImpl( mx::app::appConfigurator & _config )
{
    DMWAVEFRONTCONTROL_LOAD_CONFIG(_config);

    // DM Device Selection
    _config(m_availableDMs, "eyedoctor.availableDMs");
    _config(m_selectedDM, "eyedoctor.selectedDM");
    
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

    // Map DM device selection to actual device names and shared memory
    if(m_selectedDM == "wooferModes") {
        m_dmDeviceName = "dmwoofer";
        m_dmShmimName = "dm00disp";
    } else if(m_selectedDM == "ncpcModes") {
        m_dmDeviceName = "dmncpc";
        m_dmShmimName = "dm02disp";
    } else if(m_selectedDM == "tweeterModes") {
        m_dmDeviceName = "dmtweeter";
        m_dmShmimName = "dm01disp";
    } else if(m_selectedDM == "kiloModes") {
        m_dmDeviceName = "dmkilo";
        m_dmShmimName = "dm00disp";
    } else {
        // Default to tweeter if invalid selection
        m_dmDeviceName = "dmtweeter";
        m_dmShmimName = "dm01disp";
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
    DMWAVEFRONTCONTROL_APP_STARTUP;

    // Setup Available DMs INDI Property (Read-only)
    m_indiP_availableDMs = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_availableDMs.setDevice(this->configName());
    m_indiP_availableDMs.setName("availableDMs");
    m_indiP_availableDMs.setGroup("main");
    m_indiP_availableDMs.setLabel("Available DM Devices");
    m_indiP_availableDMs.add(pcf::IndiElement("current", m_selectedDM));
    if(registerIndiPropertyReadOnly(m_indiP_availableDMs) < 0) return -1;

    // Setup DM Device Selection INDI Property
    m_indiP_selectedDM = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_selectedDM.setDevice(this->configName());
    m_indiP_selectedDM.setName("selectedDM");
    m_indiP_selectedDM.setGroup("main");
    m_indiP_selectedDM.setLabel("Selected DM Device");
    m_indiP_selectedDM.add(pcf::IndiElement("current", m_selectedDM));
    m_indiP_selectedDM.add(pcf::IndiElement("target", m_selectedDM));
    if(registerIndiPropertyNew(m_indiP_selectedDM, &eyeDoctor::st_newCallBack_m_indiP_selectedDM) < 0) return -1;

    // Setup Camera Selection INDI Property
    m_indiP_selectedCamera = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_selectedCamera.setDevice(this->configName());
    m_indiP_selectedCamera.setName("selectedCamera");
    m_indiP_selectedCamera.setGroup("main");
    m_indiP_selectedCamera.setLabel("Selected Camera");
    m_indiP_selectedCamera.add(pcf::IndiElement("current", m_selectedCamera));
    m_indiP_selectedCamera.add(pcf::IndiElement("target", m_selectedCamera));
    if(registerIndiPropertyNew(m_indiP_selectedCamera, &eyeDoctor::st_newCallBack_m_indiP_selectedCamera) < 0) return -1;

    // Setup Mode Range INDI Properties
    this->createStandardIndiNumber(m_indiP_startModeIndex, "startModeIndex", 1, 100, 1, "%d");
    if(registerIndiPropertyNew(m_indiP_startModeIndex, &eyeDoctor::st_newCallBack_m_indiP_startModeIndex) < 0) return -1;

    this->createStandardIndiNumber(m_indiP_endModeIndex, "endModeIndex", 1, 100, 1, "%d");
    if(registerIndiPropertyNew(m_indiP_endModeIndex, &eyeDoctor::st_newCallBack_m_indiP_endModeIndex) < 0) return -1;

    // Setup Algorithm Parameters INDI Properties
    this->createStandardIndiNumber(m_indiP_psfCoreRadiusPixels, "psfCoreRadiusPixels", 1.0, 50.0, 0.1, "%0.1f");
    if(registerIndiPropertyNew(m_indiP_psfCoreRadiusPixels, &eyeDoctor::st_newCallBack_m_indiP_psfCoreRadiusPixels) < 0) return -1;

    this->createStandardIndiNumber(m_indiP_nSteps, "nSteps", 5, 100, 1, "%d");
    if(registerIndiPropertyNew(m_indiP_nSteps, &eyeDoctor::st_newCallBack_m_indiP_nSteps) < 0) return -1;

    this->createStandardIndiNumber(m_indiP_nRepeats, "nRepeats", 1, 10, 1, "%d");
    if(registerIndiPropertyNew(m_indiP_nRepeats, &eyeDoctor::st_newCallBack_m_indiP_nRepeats) < 0) return -1;

    this->createStandardIndiNumber(m_indiP_nImages, "nImages", 1, 10, 1, "%d");
    if(registerIndiPropertyNew(m_indiP_nImages, &eyeDoctor::st_newCallBack_m_indiP_nImages) < 0) return -1;

    // Setup eyeDoctor-specific INDI properties
    m_indiP_availableCameras = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_availableCameras.setDevice(this->configName());
    m_indiP_availableCameras.setName("availableCameras");
    m_indiP_availableCameras.setGroup("main");
    m_indiP_availableCameras.setLabel("Available Cameras");
    m_indiP_availableCameras.add(pcf::IndiElement("current", m_selectedCamera));
    if(registerIndiPropertyReadOnly(m_indiP_availableCameras) < 0) return -1;

    m_indiP_results = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_results.setDevice(this->configName());
    m_indiP_results.setName("results");
    m_indiP_results.setGroup("main");
    m_indiP_results.setLabel("Results");
    m_indiP_results.add(pcf::IndiElement("current", "No results yet"));
    if(registerIndiPropertyReadOnly(m_indiP_results) < 0) return -1;

    m_indiP_optimizationStatus = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_optimizationStatus.setDevice(this->configName());
    m_indiP_optimizationStatus.setName("optimizationStatus");
    m_indiP_optimizationStatus.setGroup("main");
    m_indiP_optimizationStatus.setLabel("Optimization Status");
    m_indiP_optimizationStatus.add(pcf::IndiElement("current", "Idle"));
    if(registerIndiPropertyReadOnly(m_indiP_optimizationStatus) < 0) return -1;



    // Create algorithm-specific INDI properties
    this->createStandardIndiNumber(m_indiP_targetLatency, "targetLatency", 100, 10000, 100, "%0.0f");
    this->createStandardIndiToggleSw(m_indiP_autoOptimizeLatency, "autoOptimizeLatency", "Auto Optimize Latency");
    this->createStandardIndiRequestSw(m_indiP_startOptimization, "startOptimization");
    this->createStandardIndiRequestSw(m_indiP_stopOptimization, "stopOptimization");

    // Register algorithm-specific INDI properties
    if(this->registerIndiPropertyNew(m_indiP_targetLatency, &eyeDoctor::st_newCallBack_m_indiP_targetLatency) < 0) return -1;
    if(this->registerIndiPropertyNew(m_indiP_autoOptimizeLatency, &eyeDoctor::st_newCallBack_m_indiP_autoOptimizeLatency) < 0) return -1;
    if(this->registerIndiPropertyNew(m_indiP_startOptimization, &eyeDoctor::st_newCallBack_m_indiP_startOptimization) < 0) return -1;
    if(this->registerIndiPropertyNew(m_indiP_stopOptimization, &eyeDoctor::st_newCallBack_m_indiP_stopOptimization) < 0) return -1;

    // Setup DM Metadata INDI properties
    this->createStandardIndiNumber(m_indiP_numActuators, "numActuators", 1, 100000, 1, "%d");
    if(registerIndiPropertyNew(m_indiP_numActuators, &eyeDoctor::st_newCallBack_m_indiP_numActuators) < 0) return -1;

    this->createStandardIndiNumber(m_indiP_gridWidth, "gridWidth", 1, 1000, 1, "%d");
    if(registerIndiPropertyNew(m_indiP_gridWidth, &eyeDoctor::st_newCallBack_m_indiP_gridWidth) < 0) return -1;

    this->createStandardIndiNumber(m_indiP_gridHeight, "gridHeight", 1, 1000, 1, "%d");
    if(registerIndiPropertyNew(m_indiP_gridHeight, &eyeDoctor::st_newCallBack_m_indiP_gridHeight) < 0) return -1;

    m_indiP_dmType = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_dmType.setDevice(this->configName());
    m_indiP_dmType.setName("dmType");
    m_indiP_dmType.setGroup("dmMetadata");
    m_indiP_dmType.setLabel("DM Type");
    m_indiP_dmType.add(pcf::IndiElement("current", m_dmType));
    m_indiP_dmType.add(pcf::IndiElement("target", m_dmType));
    if(registerIndiPropertyNew(m_indiP_dmType, &eyeDoctor::st_newCallBack_m_indiP_dmType) < 0) return -1;

    m_indiP_deadActuators = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_deadActuators.setDevice(this->configName());
    m_indiP_deadActuators.setName("deadActuators");
    m_indiP_deadActuators.setGroup("dmMetadata");
    m_indiP_deadActuators.setLabel("Dead Actuators");
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
    m_indiP_couplingMatrix.add(pcf::IndiElement("current", m_couplingMatrix));
    m_indiP_couplingMatrix.add(pcf::IndiElement("target", m_couplingMatrix));
    if(registerIndiPropertyNew(m_indiP_couplingMatrix, &eyeDoctor::st_newCallBack_m_indiP_couplingMatrix) < 0) return -1;

    m_indiP_actuatorGains = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_actuatorGains.setDevice(this->configName());
    m_indiP_actuatorGains.setName("actuatorGains");
    m_indiP_actuatorGains.setGroup("dmMetadata");
    m_indiP_actuatorGains.setLabel("Actuator Gains File");
    m_indiP_actuatorGains.add(pcf::IndiElement("current", m_actuatorGains));
    m_indiP_actuatorGains.add(pcf::IndiElement("target", m_actuatorGains));
    if(registerIndiPropertyNew(m_indiP_actuatorGains, &eyeDoctor::st_newCallBack_m_indiP_actuatorGains) < 0) return -1;

    m_indiP_actuatorLimits = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_actuatorLimits.setDevice(this->configName());
    m_indiP_actuatorLimits.setName("actuatorLimits");
    m_indiP_actuatorLimits.setGroup("dmMetadata");
    m_indiP_actuatorLimits.setLabel("Actuator Limits File");
    m_indiP_actuatorLimits.add(pcf::IndiElement("current", m_actuatorLimits));
    m_indiP_actuatorLimits.add(pcf::IndiElement("target", m_actuatorLimits));
    if(registerIndiPropertyNew(m_indiP_actuatorLimits, &eyeDoctor::st_newCallBack_m_indiP_actuatorLimits) < 0) return -1;

    // Configure dmWavefrontControl base class with current settings
    updateDMConfiguration();
    updateCameraConfiguration();
    updateSearchParameters();

    return 0;
}

int eyeDoctor::appLogic()
{
    DMWAVEFRONTCONTROL_APP_LOGIC;

    // Update eyeDoctor-specific INDI properties
    updateIfChanged(m_indiP_availableCameras, "current", m_selectedCamera);

    return 0;
}

int eyeDoctor::appShutdown()
{
    DMWAVEFRONTCONTROL_APP_SHUTDOWN;
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
int eyeDoctor::startOptimization()
{
    if(m_optimizationInProgress)
    {
        return 0; // Already running
    }

    m_optimizationInProgress = true;
    m_currentModeIndex = 0;
    m_totalModes = 1; // Placeholder - parse from m_modesToOptimize

    updateOptimizationStatus();
    return 0;
}

int eyeDoctor::stopOptimization()
{
    m_optimizationInProgress = false;
    updateOptimizationStatus();
    return 0;
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

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_dmPokeAmplitude)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_dmPokeAmplitude, ipRecv)
   
    float target;
    if(indiTargetUpdate(m_indiP_dmPokeAmplitude, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    // Update the dmWavefrontControl property
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_dmPokeDelay)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_dmPokeDelay, ipRecv)
   
    float target;
    if(indiTargetUpdate(m_indiP_dmPokeDelay, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    // Update the dmWavefrontControl property
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_wfsCamera)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_wfsCamera, ipRecv)
   
    std::string target;
    if(indiTargetUpdate(m_indiP_wfsCamera, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    // Update the dmWavefrontControl property
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_psfCoreRadius)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_psfCoreRadius, ipRecv)
   
    float target;
    if(indiTargetUpdate(m_indiP_psfCoreRadius, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    // Update the dmWavefrontControl property
    return 0;
}

// Callback definitions for eyeDoctor-specific properties
INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_selectedDM)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_selectedDM, ipRecv)
   
    std::string target;
    if(indiTargetUpdate(m_indiP_selectedDM, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    // Update the selected DM and reconfigure
    m_selectedDM = target;
    updateDMConfiguration();
    
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_selectedCamera)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_selectedCamera, ipRecv)
   
    std::string target;
    if(indiTargetUpdate(m_indiP_selectedCamera, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    // Update the selected camera and reconfigure
    m_selectedCamera = target;
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
    
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_searchRange)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_searchRange, ipRecv)
   
    float target;
    if(indiTargetUpdate(m_indiP_searchRange, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    // Update the dmWavefrontControl property
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
   
    bool target;
    if(indiTargetUpdate(m_indiP_autoOptimizeLatency, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    // Update the dmWavefrontControl property
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_optimizationStatus)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_optimizationStatus, ipRecv)
   
    // This is a read-only property, so we don't need to update anything
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_startOptimization)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_startOptimization, ipRecv)
   
    if(ipRecv.find("toggle") != true)
    {
        return -1;
    }

    if(ipRecv["toggle"].getSwitchState() == pcf::IndiElement::On)
    {
        if(startOptimization() < 0)
        {
            return log<software_error,-1>({__FILE__, __LINE__});
        }
    }

    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_stopOptimization)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_stopOptimization, ipRecv)
   
    if(ipRecv.find("toggle") != true)
    {
        return -1;
    }

    if(ipRecv["toggle"].getSwitchState() == pcf::IndiElement::On)
    {
        if(stopOptimization() < 0)
        {
            return log<software_error,-1>({__FILE__, __LINE__});
        }
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
    if(indiTargetUpdate(m_indiP_dmType, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    m_dmType = target;
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_deadActuators)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_deadActuators, ipRecv)
   
    std::string target;
    if(indiTargetUpdate(m_indiP_deadActuators, target, ipRecv, false) < 0)
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
    
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_couplingMatrix)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_couplingMatrix, ipRecv)
   
    std::string target;
    if(indiTargetUpdate(m_indiP_couplingMatrix, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    m_couplingMatrix = target;
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_actuatorGains)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_actuatorGains, ipRecv)
   
    std::string target;
    if(indiTargetUpdate(m_indiP_actuatorGains, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    m_actuatorGains = target;
    return 0;
}

INDI_NEWCALLBACK_DEFN(eyeDoctor, m_indiP_actuatorLimits)(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS(m_indiP_actuatorLimits, ipRecv)
   
    std::string target;
    if(indiTargetUpdate(m_indiP_actuatorLimits, target, ipRecv, false) < 0)
    {
        return log<software_error,-1>({__FILE__, __LINE__});
    }

    m_actuatorLimits = target;
    return 0;
}

} //namespace app
} //namespace MagAOX

#endif //eyeDoctor_hpp
