/** \file dmWavefrontControl.hpp
  * \brief The MagAO-X DM Wavefront Control header file
  *
  * \ingroup dmWavefrontControl_files
  */

#ifndef dmWavefrontControl_hpp
#define dmWavefrontControl_hpp

#include <mx/improc/eigenImage.hpp>
#include <mx/improc/milkImage.hpp>
#include <mx/improc/eigenCube.hpp>
using namespace mx::improc;

#include <vector>
#include <string>
#include <functional>
#include <chrono>
#include <thread>
#include <cmath>
#include <limits>

// Include MagAOX headers for INDI and configuration
#include "../MagAOXApp.hpp"
#include "../indiMacros.hpp"

/** \defgroup dmWavefrontControl
  * \brief The MagAO-X device to coordinate DM wavefront sensing and control
  *
  * <a href="../handbook/operating/software/apps/dmWavefrontControl.html">Application Documentation</a>
  *
  * \ingroup apps
  *
  */

/** \defgroup dmWavefrontControl_files
  * \ingroup dmWavefrontControl
  */

namespace MagAOX
{
namespace app
{
namespace dev
{

/// A base class to coordinate DM wavefront sensing and control
/** CRTP class `derivedT` has the following requirements:
  * 
  * - Must be derived from MagAOXApp<true>
  * 
  * - Must be derived from `dev::dmWavefrontControl<DERIVEDNAME>` (replace DERIVEDNAME with derivedT class name)
  * 
  * - Must contain the following friend declarations (replace DERIVEDNAME with derivedT class name):
  *   \code
  *      friend class dev::dmWavefrontControl<DERIVEDNAME>;
  *   \endcode
  * 
  * - Must contain the following typedefs (replace DERIVEDNAME with derivedT class name):
  *   \code
  *       typedef dev::dmWavefrontControl<DERIVEDNAME> dmWavefrontControlT;
  *   \endcode
  * 
  * - Must provide the following interfaces:
  *   \code
  *       // Run the wavefront sensing process
  *       // This coordinates the DM poking and camera acquisition
  *       // 
  *       // returns 0 on success
  *       // returns < 0 on an error
  *       int runWavefrontSensing();
  *   \endcode
  * 
  * - Must provide the following interface:
  *   \code 
  *       // Analyze the wavefront sensing results
  *       // This analyzes the results and updates metrics
  *       //
  *       // returns 0 on success
  *       // returns < 0 on an error
  *       int analyzeWavefrontSensing();
  *   \endcode
  * 
  * - Must call this base class's setupConfig(), loadConfig(), appStartup(), appStartup(), and appShutdown() in the 
  *    appropriate functions.  For convenience the following macros are defined to provide error checking:
  *    \code  
  *       DMWAVEFRONTCONTROL_SETUP_CONFIG( cfig )
  *       DMWAVEFRONTCONTROL_LOAD_CONFIG( cfig )
  *       DMWAVEFRONTCONTROL_APP_STARTUP
  *       DMWAVEFRONTCONTROL_APP_LOGIC
  *       DMWAVEFRONTCONTROL_APP_SHUTDOWN
  *    \endcode
  * 
  * \ingroup appdev
  */
template<class derivedT>
class dmWavefrontControl 
{

public:

protected:

    /** \name Configurable Parameters
      *@{
      */
    
    // DM configuration
    std::string m_dmDeviceName;           ///< Device name for the DM
    std::vector<float> m_dmModes;         ///< DM modes to use for optimization
    double m_dmPokeAmplitude;             ///< Amplitude of DM pokes (volts or microns)
    double m_dmPokeDelay;                 ///< Delay between DM commands in microseconds (CACAO mlat parameter)
    
    // Camera configuration
    std::string m_wfsCameraName;          ///< Device name for the wavefront sensing camera
    std::string m_wfsShmimName;           ///< Shared memory image name for WFS camera
    std::string m_wfsDarkShmimName;       ///< Shared memory image name for WFS dark frames
    
    // Wavefront sensing parameters
    double m_psfCoreRadiusPixels;         ///< PSF core radius in pixels for optimization
    std::vector<int> m_modesToOptimize;   ///< Specific modes to optimize
    double m_searchRange;                 ///< Search range for optimization
    
    // Timing and latency
    double m_targetLatency;               ///< Target latency between DM and camera (microseconds)
    double m_latencyTolerance;            ///< Tolerance for latency optimization (microseconds)
    bool m_autoOptimizeLatency;           ///< Whether to automatically optimize latency
    
    // Optimization parameters
    int m_maxIterations;                  ///< Maximum number of optimization iterations
    double m_convergenceThreshold;        ///< Convergence threshold for optimization
    bool m_adaptiveStepSize;              ///< Whether to use adaptive step sizes
    
    ///@}

    /** \name INDI Interface
      * @{
      */
    
    pcf::IndiProperty m_indiP_dmModes;
    INDI_NEWCALLBACK_DECL(derivedT, m_indiP_dmModes);
    
    pcf::IndiProperty m_indiP_dmPokeAmplitude;
    INDI_NEWCALLBACK_DECL(derivedT, m_indiP_dmPokeAmplitude);
    
    pcf::IndiProperty m_indiP_dmPokeDelay;
    INDI_NEWCALLBACK_DECL(derivedT, m_indiP_dmPokeDelay);
    
    pcf::IndiProperty m_indiP_wfsCamera;
    INDI_NEWCALLBACK_DECL(derivedT, m_indiP_wfsCamera);
    
    pcf::IndiProperty m_indiP_psfCoreRadius;
    INDI_NEWCALLBACK_DECL(derivedT, m_indiP_psfCoreRadius);
    
    pcf::IndiProperty m_indiP_modesToOptimize;
    INDI_NEWCALLBACK_DECL(derivedT, m_indiP_modesToOptimize);
    
    pcf::IndiProperty m_indiP_searchRange;
    INDI_NEWCALLBACK_DECL(derivedT, m_indiP_searchRange);
    
    pcf::IndiProperty m_indiP_targetLatency;
    INDI_NEWCALLBACK_DECL(derivedT, m_indiP_targetLatency);
    
    pcf::IndiProperty m_indiP_autoOptimizeLatency;
    INDI_NEWCALLBACK_DECL(derivedT, m_indiP_autoOptimizeLatency);
    
    pcf::IndiProperty m_indiP_optimizationStatus;
    INDI_NEWCALLBACK_DECL(derivedT, m_indiP_optimizationStatus);
    
    pcf::IndiProperty m_indiP_startOptimization;
    INDI_NEWCALLBACK_DECL(derivedT, m_indiP_startOptimization);
    
    pcf::IndiProperty m_indiP_stopOptimization;
    INDI_NEWCALLBACK_DECL(derivedT, m_indiP_stopOptimization);
    
    // INDI callback declarations
    static constexpr const char* st_newCallBack_dmPokeAmplitude = "dmPokeAmplitude";
    static constexpr const char* st_newCallBack_dmPokeDelay = "dmPokeDelay";
    static constexpr const char* st_newCallBack_wfsCamera = "wfsCamera";
    static constexpr const char* st_newCallBack_psfCoreRadius = "psfCoreRadius";
    static constexpr const char* st_newCallBack_searchRange = "searchRange";
    static constexpr const char* st_newCallBack_targetLatency = "targetLatency";
    static constexpr const char* st_newCallBack_autoOptimizeLatency = "autoOptimizeLatency";
    static constexpr const char* st_newCallBack_startOptimization = "startOptimization";
    static constexpr const char* st_newCallBack_stopOptimization = "stopOptimization";
    
    ///@}

    /** \name Internal State
      * @{
      */
    
    bool m_optimizationRunning;           ///< Whether optimization is currently running
    int m_currentIteration;               ///< Current optimization iteration
    double m_currentMetric;               ///< Current optimization metric value
    double m_bestMetric;                  ///< Best optimization metric value achieved
    std::vector<float> m_bestModes;      ///< Best DM modes found so far
    
    // Timing measurements
    std::chrono::high_resolution_clock::time_point m_lastDmCommand;
    std::chrono::high_resolution_clock::time_point m_lastFrameReceived;
    double m_currentLatency;              ///< Current measured latency (microseconds)
    double m_averageLatency;              ///< Average latency over recent measurements
    
    // Timing statistics
    static constexpr size_t m_timingBufferSize{ 100 }; ///< Size of circular buffer for timing statistics
    std::vector<double> m_latencyBuffer;               ///< Circular buffer for latency measurements
    std::vector<double> m_mlatBuffer;                  ///< Circular buffer for mlat measurements
    size_t m_timingBufferIndex{ 0 };                   ///< Current index in circular buffers
    
    ///@}

    /** \name CRTP Interface
      * @{
      */
    
    /// Get reference to derived class
    /** This is required for CRTP to work properly
      */
    derivedT & derived() { return static_cast<derivedT &>(*this); }
    
    /// Get const reference to derived class
    /** This is required for CRTP to work properly
      */
    const derivedT & derived() const { return static_cast<const derivedT &>(*this); }
    
    ///@}

public:

    /// Default constructor
    dmWavefrontControl();

    /// Virtual destructor
    virtual ~dmWavefrontControl();

    /** \name Configuration Interface
      * @{
      */
    
    /// Setup configuration parameters
    /** This should be called in derivedT::setupConfig
      * 
      * \param config [in] the application configuration structure
      */
    void setupConfig(mx::app::appConfigurator &config);
    
    /// Load configuration parameters
    /** This should be called in derivedT::loadConfig
      * 
      * \param config [in] the application configuration structure
      */
    void loadConfig(mx::app::appConfigurator &config);
    
    ///@}

    /** \name Application Lifecycle Interface
      * @{
      */
    
    /// Application startup
    /** This should be called in derivedT::appStartup
      * 
      * \returns 0 on success
      * \returns < 0 on error
      */
    int appStartup();
    
    /// Application logic
    /** This should be called in derivedT::appLogic
      * 
      * \returns 0 on success
      * \returns < 0 on error
      */
    int appLogic();
    
    /// Application shutdown
    /** This should be called in derivedT::appShutdown
      * 
      * \returns 0 on success
      * \returns < 0 on error
      */
    int appShutdown();
    
    ///@}

    /** \name Core Functionality Interface
      * @{
      */
    
    /// Poke the DM with specified modes
    /** Applies the specified DM modes and waits for the specified delay
      * 
      * \param modes [in] Vector of DM mode amplitudes to apply
      * \param delay_us [in] Delay to wait after applying modes (microseconds)
      * 
      * \returns 0 on success
      * \returns < 0 on error
      */
    int pokeDM(const std::vector<float>& modes, double delay_us = 0);
    
    /// Sense the wavefront using the WFS camera
    /** Acquires images from the WFS camera and processes them
      * 
      * \param timeout_ms [in] Timeout for image acquisition (milliseconds)
      * 
      * \returns 0 on success
      * \returns < 0 on error
      */
    int senseWavefront(double timeout_ms = 1000);
    
    /// Apply DM modes for optimization
    /** Applies the specified DM modes for optimization purposes
      * 
      * \param modes [in] Vector of DM mode amplitudes to apply
      * 
      * \returns 0 on success
      * \returns < 0 on error
      */
    int applyModes(const std::vector<float>& modes);
    
    /// Measure current latency between DM and camera
    /** Returns the current measured latency value
      * 
      * \returns Current latency in microseconds
      */
    double measureLatency() const { return m_currentLatency; }
    
    /// Get current optimization status
    /** Returns the current optimization status
      * 
      * \returns Current optimization metric value
      */
    double getOptimizationStatus() const { return m_currentMetric; }
    
    /// Start optimization process
    /** Initiates the optimization process
      * 
      * \returns 0 on success
      * \returns < 0 on error
      */
    int startOptimization();
    
    /// Stop optimization process
    /** Stops the optimization process
      * 
      * \returns 0 on success
      * \returns < 0 on error
      */
    int stopOptimization();
    
    ///@}

protected:

    /** \name Internal Methods
      * @{
      */
    
    /// Initialize timing infrastructure
    /** Sets up the timing measurement system
      * 
      * \returns 0 on success
      * \returns < 0 on error
      */
    int initializeTiming();
    
    /// Update timing measurements
    /** Updates timing statistics after frame acquisition
      * 
      * \param frameTime [in] Time when frame was received
      * 
      * \returns 0 on success
      * \returns < 0 on error
      */
    int updateTiming(const std::chrono::high_resolution_clock::time_point& frameTime);
    
    /// Calculate optimization metric
    /** Calculates the optimization metric from wavefront sensing data
      * 
      * \param image [in] The wavefront sensing image
      * 
      * \returns The calculated metric value
      */
    double calculateOptimizationMetric(const eigenImage<float>& image);
    
    /// Optimize latency automatically
    /** Automatically optimizes the DM poke delay based on measured latency
      * 
      * \returns 0 on success
      * \returns < 0 on error
      */
    int optimizeLatency();
    
    ///@}

}; // class dmWavefrontControl

// Implementation of template methods (inline for template classes)

template<class derivedT>
dmWavefrontControl<derivedT>::dmWavefrontControl()
{
    // Initialize member variables
    m_dmPokeAmplitude = 0.1;
    m_dmPokeDelay = 10000; // 10ms default
    m_psfCoreRadiusPixels = 5.0;
    m_searchRange = 1.0;
    m_targetLatency = 1000; // 1ms default
    m_latencyTolerance = 100; // 100μs tolerance
    m_autoOptimizeLatency = false;
    m_maxIterations = 100;
    m_convergenceThreshold = 1e-6;
    m_adaptiveStepSize = true;
    
    m_optimizationRunning = false;
    m_currentIteration = 0;
    m_currentMetric = 0.0;
    m_bestMetric = std::numeric_limits<double>::max();
    m_currentLatency = 0.0;
    m_averageLatency = 0.0;
}

template<class derivedT>
dmWavefrontControl<derivedT>::~dmWavefrontControl()
{
    // Cleanup if needed
}

template<class derivedT>
void dmWavefrontControl<derivedT>::setupConfig(mx::app::appConfigurator &config)
{
    config.add("dmWavefrontControl.dmDeviceName", "", "dmWavefrontControl.dmDeviceName", 
               argType::Required, "dmWavefrontControl", "dmDeviceName", false, "string", 
               "Device name for the DM");
    
    config.add("dmWavefrontControl.dmPokeAmplitude", "", "dmWavefrontControl.dmPokeAmplitude", 
               argType::Required, "dmWavefrontControl", "dmPokeAmplitude", false, "float", 
               "Amplitude of DM pokes (volts or microns)");
    
    config.add("dmWavefrontControl.dmPokeDelay", "", "dmWavefrontControl.dmPokeDelay", 
               argType::Required, "dmWavefrontControl", "dmPokeDelay", false, "float", 
               "Delay between DM commands in microseconds");
    
    config.add("dmWavefrontControl.wfsCameraName", "", "dmWavefrontControl.wfsCameraName", 
               argType::Required, "dmWavefrontControl", "wfsCameraName", false, "string", 
               "Device name for the wavefront sensing camera");
    
    config.add("dmWavefrontControl.wfsShmimName", "", "dmWavefrontControl.wfsShmimName", 
               argType::Required, "dmWavefrontControl", "wfsShmimName", false, "string", 
               "Shared memory image name for WFS camera");
    
    config.add("dmWavefrontControl.wfsDarkShmimName", "", "dmWavefrontControl.wfsDarkShmimName", 
               argType::Required, "dmWavefrontControl", "wfsDarkShmimName", false, "string", 
               "Shared memory image name for WFS dark frames");
    
    config.add("dmWavefrontControl.psfCoreRadiusPixels", "", "dmWavefrontControl.psfCoreRadiusPixels", 
               argType::Required, "dmWavefrontControl", "psfCoreRadiusPixels", false, "float", 
               "PSF core radius in pixels for optimization");
    
    config.add("dmWavefrontControl.searchRange", "", "dmWavefrontControl.searchRange", 
               argType::Required, "dmWavefrontControl", "searchRange", false, "float", 
               "Search range for optimization");
    
    config.add("dmWavefrontControl.targetLatency", "", "dmWavefrontControl.targetLatency", 
               argType::Required, "dmWavefrontControl", "targetLatency", false, "float", 
               "Target latency between DM and camera (microseconds)");
    
    config.add("dmWavefrontControl.latencyTolerance", "", "dmWavefrontControl.latencyTolerance", 
               argType::Required, "dmWavefrontControl", "latencyTolerance", false, "float", 
               "Tolerance for latency optimization (microseconds)");
    
    config.add("dmWavefrontControl.autoOptimizeLatency", "", "dmWavefrontControl.autoOptimizeLatency", 
               argType::Required, "dmWavefrontControl", "autoOptimizeLatency", false, "bool", 
               "Whether to automatically optimize latency");
    
    config.add("dmWavefrontControl.maxIterations", "", "dmWavefrontControl.maxIterations", 
               argType::Required, "dmWavefrontControl", "maxIterations", false, "int", 
               "Maximum number of optimization iterations");
    
    config.add("dmWavefrontControl.convergenceThreshold", "", "dmWavefrontControl.convergenceThreshold", 
               argType::Required, "dmWavefrontControl", "convergenceThreshold", false, "float", 
               "Convergence threshold for optimization");
    
    config.add("dmWavefrontControl.adaptiveStepSize", "", "dmWavefrontControl.adaptiveStepSize", 
               argType::Required, "dmWavefrontControl", "adaptiveStepSize", false, "bool", 
               "Whether to use adaptive step sizes");
}

template<class derivedT>
void dmWavefrontControl<derivedT>::loadConfig(mx::app::appConfigurator &config)
{
    config(m_dmDeviceName, "dmWavefrontControl.dmDeviceName");
    config(m_dmPokeAmplitude, "dmWavefrontControl.dmPokeAmplitude");
    config(m_dmPokeDelay, "dmWavefrontControl.dmPokeDelay");
    config(m_wfsCameraName, "dmWavefrontControl.wfsCameraName");
    config(m_wfsShmimName, "dmWavefrontControl.wfsShmimName");
    config(m_wfsDarkShmimName, "dmWavefrontControl.wfsDarkShmimName");
    config(m_psfCoreRadiusPixels, "dmWavefrontControl.psfCoreRadiusPixels");
    config(m_searchRange, "dmWavefrontControl.searchRange");
    config(m_targetLatency, "dmWavefrontControl.targetLatency");
    config(m_latencyTolerance, "dmWavefrontControl.latencyTolerance");
    config(m_autoOptimizeLatency, "dmWavefrontControl.autoOptimizeLatency");
    config(m_maxIterations, "dmWavefrontControl.maxIterations");
    config(m_convergenceThreshold, "dmWavefrontControl.convergenceThreshold");
    config(m_adaptiveStepSize, "dmWavefrontControl.adaptiveStepSize");
}

template<class derivedT>
int dmWavefrontControl<derivedT>::appStartup()
{
    // Initialize timing infrastructure
    if(initializeTiming() < 0)
    {
        return -1;
    }
    
    // Create INDI properties
    derived().createStandardIndiNumber(m_indiP_dmPokeAmplitude, "dmPokeAmplitude", 0.001, 10.0, 0.001, "%0.3f");
    derived().createStandardIndiNumber(m_indiP_dmPokeDelay, "dmPokeDelay", 100, 100000, 100, "%0.0f");
    derived().createStandardIndiText(m_indiP_wfsCamera, "wfsCamera", "current", "WFS Camera");
    derived().createStandardIndiNumber(m_indiP_psfCoreRadius, "psfCoreRadius", 1.0, 100.0, 0.1, "%0.1f");
    derived().createStandardIndiNumber(m_indiP_searchRange, "searchRange", 0.01, 10.0, 0.01, "%0.2f");
    derived().createStandardIndiNumber(m_indiP_targetLatency, "targetLatency", 100, 10000, 100, "%0.0f");
    derived().createStandardIndiToggleSw(m_indiP_autoOptimizeLatency, "autoOptimizeLatency", "Auto Optimize Latency");
    derived().createROIndiText(m_indiP_optimizationStatus, "optimizationStatus", "current", "Optimization Status");
    derived().createStandardIndiRequestSw(m_indiP_startOptimization, "startOptimization");
    derived().createStandardIndiRequestSw(m_indiP_stopOptimization, "stopOptimization");
    
            // Register INDI properties
        // Register properties that require callbacks (derived classes must implement these)
        if(derived().registerIndiPropertyNew(m_indiP_dmPokeAmplitude, &derivedT::st_newCallBack_m_indiP_dmPokeAmplitude) < 0) return -1;
        if(derived().registerIndiPropertyNew(m_indiP_dmPokeDelay, &derivedT::st_newCallBack_m_indiP_dmPokeDelay) < 0) return -1;
        if(derived().registerIndiPropertyNew(m_indiP_wfsCamera, &derivedT::st_newCallBack_m_indiP_wfsCamera) < 0) return -1;
        if(derived().registerIndiPropertyNew(m_indiP_psfCoreRadius, &derivedT::st_newCallBack_m_indiP_psfCoreRadius) < 0) return -1;
        if(derived().registerIndiPropertyNew(m_indiP_searchRange, &derivedT::st_newCallBack_m_indiP_searchRange) < 0) return -1;
        if(derived().registerIndiPropertyNew(m_indiP_targetLatency, &derivedT::st_newCallBack_m_indiP_targetLatency) < 0) return -1;
        if(derived().registerIndiPropertyNew(m_indiP_autoOptimizeLatency, &derivedT::st_newCallBack_m_indiP_autoOptimizeLatency) < 0) return -1;
        if(derived().registerIndiPropertyNew(m_indiP_startOptimization, &derivedT::st_newCallBack_m_indiP_startOptimization) < 0) return -1;
        if(derived().registerIndiPropertyNew(m_indiP_stopOptimization, &derivedT::st_newCallBack_m_indiP_stopOptimization) < 0) return -1;
        
        // Register read-only properties
        if(derived().registerIndiPropertyReadOnly(m_indiP_optimizationStatus) < 0) return -1;
    
    return 0;
}

template<class derivedT>
int dmWavefrontControl<derivedT>::appLogic()
{
    // Update optimization status if running
    if(m_optimizationRunning)
    {
        // Check if optimization should continue
        if(m_currentIteration >= m_maxIterations)
        {
            stopOptimization();
        }
        else
        {
            // Run one optimization step
            // This would be implemented by derived classes
        }
    }
    
    // Auto-optimize latency if enabled
    if(m_autoOptimizeLatency)
    {
        optimizeLatency();
    }
    
    return 0;
}

template<class derivedT>
int dmWavefrontControl<derivedT>::appShutdown()
{
    // Stop any running optimization
    if(m_optimizationRunning)
    {
        stopOptimization();
    }
    
    return 0;
}

template<class derivedT>
int dmWavefrontControl<derivedT>::pokeDM(const std::vector<float>& modes, double delay_us)
{
    // Record the time when DM command is sent
    m_lastDmCommand = std::chrono::high_resolution_clock::now();
    
    // Apply DM modes (to be implemented by derived class)
    if(applyModes(modes) < 0)
    {
        return -1;
    }
    
    // Wait for the specified delay
    if(delay_us > 0)
    {
        std::this_thread::sleep_for(std::chrono::microseconds(static_cast<long long>(delay_us)));
    }
    
    return 0;
}

template<class derivedT>
int dmWavefrontControl<derivedT>::senseWavefront(double timeout_ms)
{
    // This is a placeholder - derived classes should implement actual wavefront sensing
    // The key is to call updateTiming() when a frame is received
    
    // Simulate frame reception for now
    auto frameTime = std::chrono::high_resolution_clock::now();
    updateTiming(frameTime);
    
    return 0;
}

template<class derivedT>
int dmWavefrontControl<derivedT>::applyModes(const std::vector<float>& modes)
{
    // This is a placeholder - derived classes should implement actual DM mode application
    // Store the modes for later use
    m_dmModes = modes;
    
    return 0;
}

template<class derivedT>
int dmWavefrontControl<derivedT>::startOptimization()
{
    if(m_optimizationRunning)
    {
        return 0; // Already running
    }
    
    m_optimizationRunning = true;
    m_currentIteration = 0;
    m_bestMetric = std::numeric_limits<double>::max();
    
    // Update INDI status
    derived().updateIfChanged(m_indiP_optimizationStatus, "current", "Running", INDI_BUSY);
    
    return 0;
}

template<class derivedT>
int dmWavefrontControl<derivedT>::stopOptimization()
{
    if(!m_optimizationRunning)
    {
        return 0; // Not running
    }
    
    m_optimizationRunning = false;
    
    // Update INDI status
    derived().updateIfChanged(m_indiP_optimizationStatus, "current", "Idle", INDI_IDLE);
    
    return 0;
}

template<class derivedT>
int dmWavefrontControl<derivedT>::initializeTiming()
{
    // Initialize timing buffers
    m_latencyBuffer.reserve(m_timingBufferSize);
    m_mlatBuffer.reserve(m_timingBufferSize);
    
    // Set initial timing values
    m_lastDmCommand = std::chrono::high_resolution_clock::now();
    m_lastFrameReceived = std::chrono::high_resolution_clock::now();
    
    return 0;
}

template<class derivedT>
int dmWavefrontControl<derivedT>::updateTiming(const std::chrono::high_resolution_clock::time_point& frameTime)
{
    m_lastFrameReceived = frameTime;
    
    // Calculate latency from last DM command to frame reception
    auto latency = std::chrono::duration_cast<std::chrono::microseconds>(frameTime - m_lastDmCommand);
    m_currentLatency = static_cast<double>(latency.count());
    
    // Update running average
    m_averageLatency = 0.9 * m_averageLatency + 0.1 * m_currentLatency;
    
    return 0;
}

template<class derivedT>
double dmWavefrontControl<derivedT>::calculateOptimizationMetric(const eigenImage<float>& image)
{
    // This is a placeholder - derived classes should implement actual metric calculation
    // For now, return a simple metric based on image statistics
    
    if(image.rows() == 0 || image.cols() == 0)
    {
        return std::numeric_limits<double>::max();
    }
    
    // Calculate RMS of the image as a simple metric
    double sum = 0.0;
    double sumSq = 0.0;
    int count = 0;
    
    for(int i = 0; i < image.rows(); ++i)
    {
        for(int j = 0; j < image.cols(); ++j)
        {
            double val = static_cast<double>(image(i, j));
            sum += val;
            sumSq += val * val;
            count++;
        }
    }
    
    if(count == 0) return std::numeric_limits<double>::max();
    
    double mean = sum / count;
    double variance = (sumSq / count) - (mean * mean);
    double rms = sqrt(std::max(0.0, variance));
    
    return rms;
}

template<class derivedT>
int dmWavefrontControl<derivedT>::optimizeLatency()
{
    // Simple latency optimization: adjust DM poke delay based on measured latency
    if(m_currentLatency > m_targetLatency + m_latencyTolerance)
    {
        // Latency is too high, increase delay
        m_dmPokeDelay = std::min(m_dmPokeDelay * 1.1, 100000.0); // Max 100ms
    }
    else if(m_currentLatency < m_targetLatency - m_latencyTolerance)
    {
        // Latency is too low, decrease delay
        m_dmPokeDelay = std::max(m_dmPokeDelay * 0.9, 100.0); // Min 100μs
    }
    
    // Update INDI property
    derived().updateIfChanged(m_indiP_dmPokeDelay, "current", m_dmPokeDelay, INDI_IDLE);
    
    return 0;
}

// Convenience macros for derived classes
#define DMWAVEFRONTCONTROL_SETUP_CONFIG( cfig ) \
    dmWavefrontControlT::setupConfig( cfig )

#define DMWAVEFRONTCONTROL_LOAD_CONFIG( cfig ) \
    dmWavefrontControlT::loadConfig( cfig )

#define DMWAVEFRONTCONTROL_APP_STARTUP \
    if( dmWavefrontControlT::appStartup() < 0 ) \
    { \
        return -1; \
    }

#define DMWAVEFRONTCONTROL_APP_LOGIC \
    if( dmWavefrontControlT::appLogic() < 0 ) \
    { \
        return -1; \
    }

#define DMWAVEFRONTCONTROL_APP_SHUTDOWN \
    if( dmWavefrontControlT::appShutdown() < 0 ) \
    { \
        return -1; \
    }

} // namespace dev
} // namespace app
} // namespace MagAOX

#endif // dmWavefrontControl_hpp
