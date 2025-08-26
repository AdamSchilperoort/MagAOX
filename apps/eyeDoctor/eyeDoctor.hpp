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
    
    std::string m_modesToOptimize; ///< Modes to optimize
    std::vector<std::string> m_availableCameras; ///< List of available cameras
    std::string m_selectedCamera; ///< Currently selected camera
    
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

    pcf::IndiProperty m_indiP_availableCameras;

    pcf::IndiProperty m_indiP_optimizationStatus;
    pcf::IndiProperty m_indiP_results;

    // Algorithm-specific INDI properties
    pcf::IndiProperty m_indiP_targetLatency;
    pcf::IndiProperty m_indiP_autoOptimizeLatency;
    pcf::IndiProperty m_indiP_startOptimization;
    pcf::IndiProperty m_indiP_stopOptimization;

    // Algorithm-specific parameters
    double m_targetLatency;               ///< Target latency between DM and camera (microseconds)
    double m_latencyTolerance;            ///< Tolerance around target latency (microseconds)
    bool m_autoOptimizeLatency;           ///< Whether to automatically optimize latency
    
    // Optimization parameters
    int m_maxIterations;                  ///< Maximum number of optimization iterations
    double m_convergenceThreshold;        ///< Convergence threshold for optimization
    bool m_adaptiveStepSize;              ///< Whether to use adaptive step sizes

    // Callback declarations for dmWavefrontControl properties

    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_dmPokeAmplitude);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_dmPokeDelay);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_wfsCamera);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_psfCoreRadius);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_searchRange);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_targetLatency);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_autoOptimizeLatency);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_optimizationStatus);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_startOptimization);
    INDI_NEWCALLBACK_DECL(eyeDoctor, m_indiP_stopOptimization);



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


};

eyeDoctor::eyeDoctor() : MagAOXApp(MAGAOX_CURRENT_SHA1, MAGAOX_REPO_MODIFIED)
{
    m_optimizationInProgress = false;
    m_measurementComplete = false;
    m_currentModeIndex = 0;
    m_totalModes = 0;
    
    // Initialize algorithm-specific parameters
    m_targetLatency = 1000; // 1ms default
    m_latencyTolerance = 100; // 100 microseconds tolerance
    m_autoOptimizeLatency = false;
    
    // Initialize optimization parameters
    m_maxIterations = 100;
    m_convergenceThreshold = 1e-6;
    m_adaptiveStepSize = true;
    
    return;
}

void eyeDoctor::setupConfig()
{
    DMWAVEFRONTCONTROL_SETUP_CONFIG(config);

    config.add("eyedoctor.modesToOptimize", "", "eyedoctor.modesToOptimize", argType::Required, "eyedoctor", "modesToOptimize", false, "string", "Modes to optimize");
    config.add("eyedoctor.availableCameras", "", "eyedoctor.availableCameras", argType::Required, "eyedoctor", "availableCameras", false, "vector<string>", "List of available cameras");
    
    // Algorithm-specific configuration
    config.add("eyedoctor.targetLatency", "", "eyedoctor.targetLatency", argType::Required, "eyedoctor", "targetLatency", false, "float", "Target latency between DM and camera (microseconds)");
    config.add("eyedoctor.latencyTolerance", "", "eyedoctor.latencyTolerance", argType::Required, "eyedoctor", "latencyTolerance", false, "float", "Tolerance around target latency (microseconds)");
    config.add("eyedoctor.autoOptimizeLatency", "", "eyedoctor.autoOptimizeLatency", argType::Required, "eyedoctor", "autoOptimizeLatency", false, "bool", "Whether to automatically optimize latency");
    
    config.add("eyedoctor.maxIterations", "", "eyedoctor.maxIterations", argType::Required, "eyedoctor", "maxIterations", false, "int", "Maximum number of optimization iterations");
    config.add("eyedoctor.convergenceThreshold", "", "eyedoctor.convergenceThreshold", argType::Required, "eyedoctor", "convergenceThreshold", false, "float", "Convergence threshold for optimization");
    config.add("eyedoctor.adaptiveStepSize", "", "eyedoctor.adaptiveStepSize", argType::Required, "eyedoctor", "adaptiveStepSize", false, "bool", "Whether to use adaptive step sizes");
}

int eyeDoctor::loadConfigImpl( mx::app::appConfigurator & _config )
{
    DMWAVEFRONTCONTROL_LOAD_CONFIG(_config);

    _config(m_modesToOptimize, "eyedoctor.modesToOptimize");
    _config(m_availableCameras, "eyedoctor.availableCameras");

    // Load algorithm-specific configuration
    _config(m_targetLatency, "eyedoctor.targetLatency");
    _config(m_latencyTolerance, "eyedoctor.latencyTolerance");
    _config(m_autoOptimizeLatency, "eyedoctor.autoOptimizeLatency");
    _config(m_maxIterations, "eyedoctor.maxIterations");
    _config(m_convergenceThreshold, "eyedoctor.convergenceThreshold");
    _config(m_adaptiveStepSize, "eyedoctor.adaptiveStepSize");

    // Set default selected camera to first available
    if(m_availableCameras.size() > 0)
    {
        m_selectedCamera = m_availableCameras[0];
    }

    return 0;
}

void eyeDoctor::loadConfig()
{
    loadConfigImpl(config);
}

int eyeDoctor::appStartup()
{
    DMWAVEFRONTCONTROL_APP_STARTUP;

    // Setup eyeDoctor-specific INDI properties
    m_indiP_availableCameras = pcf::IndiProperty(pcf::IndiProperty::Text);
    m_indiP_availableCameras.setDevice(this->configName());
    m_indiP_availableCameras.setName("availableCameras");
    m_indiP_availableCameras.setGroup("main");
    m_indiP_availableCameras.setLabel("Available Cameras");
    m_indiP_availableCameras.add(pcf::IndiElement("current"));
    m_indiP_availableCameras["current"].setValue(m_selectedCamera);
    if(registerIndiPropertyReadOnly(m_indiP_availableCameras) < 0) return -1;

    registerIndiPropertyReadOnly(m_indiP_optimizationStatus, "optimizationStatus", pcf::IndiProperty::Text, pcf::IndiProperty::ReadOnly, pcf::IndiProperty::Idle);
    m_indiP_optimizationStatus.add({"status", "Idle"});

    registerIndiPropertyReadOnly(m_indiP_results, "results", pcf::IndiProperty::Number, pcf::IndiProperty::ReadOnly, pcf::IndiProperty::Idle);
    m_indiP_results.add({"progress", 0.0});
    m_indiP_results.add({"currentMode", 0});

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
    m_indiP_optimizationStatus["status"] = status;
    updateIfChanged(m_indiP_optimizationStatus, "status", status);
    return 0;
}

int eyeDoctor::updateResults()
{
    updateIfChanged(m_indiP_results, "progress", (double)m_currentModeIndex / m_totalModes);
    updateIfChanged(m_indiP_results, "currentMode", m_currentModeIndex);
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

} //namespace app
} //namespace MagAOX

#endif //eyeDoctor_hpp
