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
#include <fstream>
#include <sstream>
#include <iomanip>
#include <map>
#include <memory>
#include <algorithm>

// Include MagAOX headers for INDI and configuration
#include "../MagAOXApp.hpp"
#include "../indiMacros.hpp"

// Include FITS utilities for mode loading
#include <mx/ioutils/fits/fitsUtils.hpp>
#include <mx/improc/imageUtils.hpp>

// Include CFITSIO for direct FITS file access
#include <fitsio.h>

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
    double m_dmPokeAmplitude;             ///< Amplitude of DM pokes (volts or microns)
    double m_dmPokeDelay;                 ///< Delay between DM commands in microseconds
    
    // Camera configuration
    std::string m_wfsCameraName;          ///< Device name for the wavefront sensing camera
    std::string m_wfsShmimName;           ///< Shared memory image name for WFS camera
    std::string m_wfsDarkShmimName;       ///< Shared memory image name for WFS dark frames
    
    // Wavefront sensing parameters
    double m_psfCoreRadiusPixels;         ///< PSF core radius in pixels for optimization
    double m_searchRange;                 ///< Search range for optimization
    
    // Timing and latency (DM-specific, not camera-specific)
    double m_mlat;                        ///< Round-trip DM->camera latency (microseconds)
    double m_dmCommandLatency;            ///< Time from command to DM response
    double m_cameraResponseLatency;       ///< Time from DM response to camera frame

    
    ///@}

    /** \name DM Metadata and Modesets
      * @{
      */
    
    // DM actuator information
    struct DMActuator {
        int id;                     ///< Actuator ID
        double x, y;                ///< Physical position (mm)
        double gain;                ///< Individual actuator gain
        double minValue, maxValue;  ///< Actuator limits
        bool isDead;                ///< Dead actuator flag
        double couplingFactor;      ///< Cross-coupling with neighbors
    };

    struct DMInfo {
        std::string name;           ///< DM identifier
        int numActuators;           ///< Total number of actuators
        int width, height;          ///< Actuator array dimensions
        double actuatorSpacing;     ///< Physical spacing (mm)
        double maxStroke;           ///< Maximum stroke (microns)
        
        std::vector<DMActuator> actuators;
        mx::improc::eigenImage<bool> actuatorMask;      ///< Valid actuator mask
        mx::improc::eigenImage<double> actuatorGains;   ///< Per-actuator gains
    };

    // Modeset information
    struct ModeSet {
        std::string name;                    ///< e.g., "zernike", "hadamard"
        std::string filename;                ///< e.g., "zernike_modes.fits"
        mx::improc::eigenCube<float> modes;  ///< 3D cube: [modes][height][width]
        std::vector<std::string> modeNames;  ///< e.g., ["Z1", "Z2", "Z3", ...]
        std::vector<double> modeScales;      ///< Physical units (microns/coefficient)
        std::vector<double> modalGains;      ///< Control loop gains
        std::vector<double> modeMin;         ///< Min allowed values
        std::vector<double> modeMax;         ///< Max allowed values
    };

    // DM metadata
    DMInfo m_dmInfo;                        ///< DM information and actuator mapping
    std::vector<ModeSet> m_modeSets;        ///< Multiple modesets
    std::map<std::string, size_t> m_modeSetMap; ///< Quick lookup by name
    std::string m_defaultModeSet;           ///< Default modeset to use
    
    // Mode optimization
    std::vector<std::string> m_modesToOptimize; ///< Specific modes to optimize (format: "modeset:mode")
    
    // Configuration files
    std::string m_dmMetadataFile;           ///< DM metadata configuration file (optional, if empty use config params)
    std::vector<std::string> m_modeSetFiles; ///< List of modeset FITS files
    std::vector<std::string> m_modeSetNames; ///< Names for each modeset
    
    // DM metadata parameters (loaded from config if metadata_file is empty)
    std::string m_configDeviceName;        ///< DM device/shared memory name (e.g., dm01disp06) from config
    int m_configNumActuators;               ///< Number of actuators from config
    int m_configGridWidth;                  ///< Grid width from config
    int m_configGridHeight;                 ///< Grid height from config
    double m_configActuatorSpacing;        ///< Actuator spacing from config (mm)
    double m_configMaxStroke;               ///< Max stroke from config (microns)
    std::string m_configDmType;            ///< DM type from config
    std::vector<int> m_configDeadActuators; ///< Dead actuators from config
    std::string m_configCouplingMatrix;    ///< Path to coupling matrix file from config
    std::string m_configActuatorGains;     ///< Path to actuator gains file from config
    std::string m_configActuatorLimits;    ///< Path to actuator limits file from config
    
    // Timing buffers for MLAT
    std::vector<double> m_mlatBuffer;       ///< Circular buffer for MLAT measurements
    double m_mlatAvg{ 0 };                  ///< Average MLAT over recent measurements
    double m_mlatStd{ 0 };                  ///< Standard deviation of MLAT
    
    ///@}

    /** \name INDI Interface
      * @{
      */
    
    pcf::IndiProperty m_indiP_dmPokeAmplitude;
    INDI_NEWCALLBACK_DECL(derivedT, m_indiP_dmPokeAmplitude);
    
    pcf::IndiProperty m_indiP_dmPokeDelay;
    INDI_NEWCALLBACK_DECL(derivedT, m_indiP_dmPokeDelay);
    
    pcf::IndiProperty m_indiP_wfsCamera;
    INDI_NEWCALLBACK_DECL(derivedT, m_indiP_wfsCamera);
    
    pcf::IndiProperty m_indiP_psfCoreRadius;
    INDI_NEWCALLBACK_DECL(derivedT, m_indiP_psfCoreRadius);
    
    pcf::IndiProperty m_indiP_searchRange;
    INDI_NEWCALLBACK_DECL(derivedT, m_indiP_searchRange);
    

    
    // New properties for enhanced system
    pcf::IndiProperty m_indiP_modesets;
    pcf::IndiProperty m_indiP_defaultModeSet;
    pcf::IndiProperty m_indiP_modesToOptimize;
    pcf::IndiProperty m_indiP_mlat;
    pcf::IndiProperty m_indiP_dmCommandLatency;
    pcf::IndiProperty m_indiP_cameraResponseLatency;
    
    // INDI callback declarations
    static constexpr const char* st_newCallBack_dmPokeAmplitude = "dmPokeAmplitude";
    static constexpr const char* st_newCallBack_dmPokeDelay = "dmPokeDelay";
    static constexpr const char* st_newCallBack_wfsCamera = "wfsCamera";
    static constexpr const char* st_newCallBack_psfCoreRadius = "psfCoreRadius";
    static constexpr const char* st_newCallBack_searchRange = "searchRange";

    
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

    
    // Timing statistics
    static constexpr size_t m_timingBufferSize{ 100 }; ///< Size of circular buffer for timing statistics
    std::vector<double> m_latencyBuffer;               ///< Circular buffer for latency measurements
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
    

    
    /// Get current optimization status
    /** Returns the current optimization status
      * 
      * \returns Current optimization metric value
      */
    double getOptimizationStatus() const { return m_currentMetric; }
    
    /// Get PSF core radius
    /** Returns the current PSF core radius in pixels
      * 
      * \returns PSF core radius in pixels
      */
    double getPSFCoreRadius() const { return m_psfCoreRadiusPixels; }
    
    /// Set PSF core radius
    /** Sets the PSF core radius in pixels
      * 
      * \param radius [in] PSF core radius in pixels
      */
    void setPSFCoreRadius(double radius) { m_psfCoreRadiusPixels = radius; }
    
    /// Get DM poke delay
    /** Returns the current DM poke delay in microseconds
      * 
      * \returns DM poke delay in microseconds
      */
    double getDMPokeDelay() const { return m_dmPokeDelay; }
    
    /// Set DM poke delay
    /** Sets the DM poke delay in microseconds
      * 
      * \param delay [in] DM poke delay in microseconds
      */
    void setDMPokeDelay(double delay) { m_dmPokeDelay = delay; }
    

    
    ///@}

    /** \name Enhanced DM Control Interface
      * @{
      */
    
    /// Load DM metadata from configuration file
    int loadDMMetadata();
    
    /// Load modesets from FITS files
    int loadModeSets();
    
    /// Load a single modeset from FITS file
    int loadModeSet(const std::string& filename, const std::string& name);
    
    /// Apply individual actuator poke
    int pokeActuator(int actuatorId, double amplitude);
    
    /// Apply modal mode
    int applyModalMode(const std::string& modesetName, int modeIndex, double amplitude);
    
    /// Apply mixed modal commands
    int applyMixedModes(const std::vector<std::pair<std::string, double>>& commands);
    
    /// Convert modal commands to actuator commands
    std::vector<double> modalToActuator(const std::vector<std::pair<std::string, double>>& modalCommands);
    
    /// Update MLAT measurement
    int updateMLAT(double dmCommandTime, double cameraFrameTime);
    
    ///@}

    // Image processing and PSF analysis methods
    /**
     * @brief Subtract background from image using various methods
     * @param image Input image
     * @param method Background subtraction method (0=full median, 1=edge median, 2=mode, 3=row/column)
     * @return Background-subtracted image
     */
    eigenImage<float> subtractBackground(const eigenImage<float>& image, int method = 0);
    
    /**
     * @brief Find peak in image using various methods
     * @param image Input image
     * @param method Peak finding method (0=naive max, 1=Gaussian fit)
     * @param clipping Clipping radius for Gaussian fit
     * @return Peak value
     */
    double findPeak(const eigenImage<float>& image, int method = 0, int clipping = 0);
    
    /**
     * @brief Compute core sum metric (sum within radius around peak)
     * @param image Input image
     * @param radius Core radius in pixels
     * @param center Optional center coordinates (y, x)
     * @return Core sum value (negative for minimization)
     */
    double computeCoreSum(const eigenImage<float>& image, double radius, std::pair<double, double> center = {-1, -1});
    
    /**
     * @brief Compute core/ring ratio metric
     * @param image Input image
     * @param radius1 Core radius in pixels
     * @param radius2 Annulus radius in pixels
     * @return Core/ring ratio
     */
    double computeCoreRingRatio(const eigenImage<float>& image, double radius1, double radius2);
    
    /**
     * @brief Fit 2D Gaussian to image
     * @param image Input image
     * @param clipping Clipping radius for fit
     * @return Vector of [fwhm, peak, center_y, center_x]
     */
    std::vector<double> fitGaussian(const eigenImage<float>& image, int clipping = 0);
    
    /**
     * @brief Generate 2D Gaussian
     * @param fwhm Full width at half maximum
     * @param center Center coordinates (y, x)
     * @param size Image size (height, width)
     * @return 2D Gaussian image
     */
    eigenImage<float> generateGaussian(double fwhm, std::pair<double, double> center, std::pair<int, int> size);
    
    /**
     * @brief Compute image peak from multiple images
     * @param images Vector of images
     * @return Average peak value
     */
    double computeImagePeak(const std::vector<eigenImage<float>>& images);
    
    /**
     * @brief Compute root sum square of images
     * @param images Vector of images
     * @return RSS value
     */
    double computeRSS(const std::vector<eigenImage<float>>& images);
    
    /**
     * @brief Generate obscured Airy disk pattern
     * @param I0 Peak intensity
     * @param wavelength Wavelength in meters
     * @param fnum F-number
     * @param pixscale Pixel scale in radians
     * @param center Center coordinates (y, x)
     * @param size Image size (height, width)
     * @return Airy disk image
     */
    eigenImage<float> generateObscuredAiryDisk(double I0, double wavelength, double fnum, double pixscale, 
                                              std::pair<double, double> center, std::pair<int, int> size);
    
    /**
     * @brief Fit Airy disk to PSF
     * @param psf Input PSF image
     * @param wavelength Wavelength in meters
     * @param fnum F-number
     * @param pixscale Pixel scale in radians
     * @param cutout Cutout size for fitting
     * @return Vector of [center_y, center_x, background]
     */
    std::vector<double> fitAiryDisk(const eigenImage<float>& psf, double wavelength, double fnum, double pixscale, int cutout = 100);
    
    /**
     * @brief Compute Airy disk metric
     * @param measured Measured image
     * @param model Model image
     * @param penalty Penalty factor
     * @return Metric value
     */
    double computeAiryMetric(const eigenImage<float>& measured, const eigenImage<float>& model, double penalty = 0.0);

    // Optimization algorithms
    /**
     * @brief Move DM, measure, and compute metric (core optimization function)
     * @param modeValue Mode amplitude value
     * @param modeIndex Mode index to apply
     * @param nImages Number of images to collect
     * @param metricType Type of metric to compute
     * @param metricParams Metric parameters
     * @return Metric value to minimize
     */
    double moveMeasureMetric(double modeValue, int modeIndex, int nImages, const std::string& metricType, 
                            const std::map<std::string, double>& metricParams);
    
    /**
     * @brief Optimize single mode using Brent method
     * @param modeIndex Mode index to optimize
     * @param bounds Search bounds [min, max]
     * @param nImages Number of images per measurement
     * @param metricType Type of metric to compute
     * @param metricParams Metric parameters
     * @param tolerance Optimization tolerance
     * @return Optimized mode amplitude
     */
    double optimizeModeBrent(int modeIndex, std::pair<double, double> bounds, int nImages, 
                            const std::string& metricType, const std::map<std::string, double>& metricParams, 
                            double tolerance = 1e-5);
    
    /**
     * @brief Grid sweep optimization for single mode
     * @param modeIndex Mode index to optimize
     * @param bounds Search bounds [min, max]
     * @param nSteps Number of grid steps
     * @param nRepeats Number of measurement repeats
     * @param nImages Number of images per measurement
     * @param metricType Type of metric to compute
     * @param metricParams Metric parameters
     * @param fitMethod Fit method ('fit' or 'mean')
     * @return Optimized mode amplitude
     */
    double gridSweepOptimization(int modeIndex, std::pair<double, double> bounds, int nSteps, int nRepeats, int nImages,
                                 const std::string& metricType, const std::map<std::string, double>& metricParams,
                                 const std::string& fitMethod = "fit");

    /**
     * @brief Compute metric from images
     * @param images Vector of images
     * @param metricType Type of metric to compute
     * @param metricParams Metric parameters
     * @return Metric value
     */
    double computeMetric(const std::vector<eigenImage<float>>& images, const std::string& metricType, 
                        const std::map<std::string, double>& metricParams);

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

    // Helper methods for optimization
    /**
     * @brief Apply mode to DM and wait for completion
     * @param modeIndex Mode index
     * @param amplitude Mode amplitude
     * @return 0 on success, -1 on failure
     */
    int applyModeAndWait(int modeIndex, double amplitude);
    
    /**
     * @brief Collect images from camera
     * @param nImages Number of images to collect
     * @return Vector of images
     */
    std::vector<eigenImage<float>> collectImages(int nImages);
    
    /**
     * @brief Find centroid using center of mass
     * @param image Input image
     * @param mask Optional mask for centroid calculation
     * @return Centroid coordinates (y, x)
     */
    std::pair<double, double> findCentroid(const eigenImage<float>& image, const eigenImage<bool>& mask = eigenImage<bool>());
    
    /**
     * @brief Create circular mask
     * @param center Center coordinates (y, x)
     * @param radius Radius in pixels
     * @param size Image size (height, width)
     * @return Binary mask
     */
    eigenImage<bool> createCircularMask(std::pair<double, double> center, double radius, std::pair<int, int> size);
    
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
    // Initialize optimization state
    m_optimizationRunning = false;
    m_currentMetric = 0.0;
    m_bestMetric = std::numeric_limits<double>::max();
    
    // Initialize enhanced system
    m_dmInfo.name = "default";
    m_dmInfo.numActuators = 0;
    m_dmInfo.width = 0;
    m_dmInfo.height = 0;
    m_dmInfo.actuatorSpacing = 0.0;
    m_dmInfo.maxStroke = 0.0;
    
    // Initialize config parameters
    m_configDeviceName = "";
    m_configNumActuators = 0;
    m_configGridWidth = 0;
    m_configGridHeight = 0;
    m_configActuatorSpacing = 0.0;
    m_configMaxStroke = 0.0;
    m_configDmType = "";
    m_configDeadActuators.clear();
    m_configCouplingMatrix = "";
    m_configActuatorGains = "";
    m_configActuatorLimits = "";
    
    // Initialize timing buffers
    m_mlatBuffer.reserve(m_timingBufferSize);
}

template<class derivedT>
dmWavefrontControl<derivedT>::~dmWavefrontControl()
{
    // Cleanup if needed
}

// Implementation of enhanced DM control methods

template<class derivedT>
int dmWavefrontControl<derivedT>::loadDMMetadata()
{
    try {
        // Trim whitespace from metadata file name
        std::string metadataFileTrimmed = m_dmMetadataFile;
        if (!metadataFileTrimmed.empty()) {
            // Trim leading/trailing whitespace
            size_t first = metadataFileTrimmed.find_first_not_of(" \t\n\r");
            if (first != std::string::npos) {
                metadataFileTrimmed.erase(0, first);
            } else {
                metadataFileTrimmed.clear();
            }
            size_t last = metadataFileTrimmed.find_last_not_of(" \t\n\r");
            if (last != std::string::npos && last + 1 < metadataFileTrimmed.length()) {
                metadataFileTrimmed.erase(last + 1);
            }
        }
        
        // Log the metadata file value for debugging
        derived().template log<text_log>("DM metadata_file config value: '" + m_dmMetadataFile + "' (trimmed: '" + metadataFileTrimmed + "')");
        
        // Load DM metadata from configuration file if specified and not empty
        if (!metadataFileTrimmed.empty()) {
            std::ifstream file(metadataFileTrimmed);
            if (!file.is_open()) {
                derived().template log<software_error>({__FILE__, __LINE__, "Could not open DM metadata file: " + metadataFileTrimmed});
                return -1;
            }
            
            std::string line;
            while (std::getline(file, line)) {
                if (line.empty() || line[0] == '#') continue;
                
                std::istringstream iss(line);
                std::string key, value;
                if (std::getline(iss, key, '=') && std::getline(iss, value)) {
                    // Parse key-value pairs
                    if (key == "numActuators") {
                        m_dmInfo.numActuators = std::stoi(value);
                    } else if (key == "width") {
                        m_dmInfo.width = std::stoi(value);
                    } else if (key == "height") {
                        m_dmInfo.height = std::stoi(value);
                    } else if (key == "actuatorSpacing") {
                        m_dmInfo.actuatorSpacing = std::stod(value);
                    } else if (key == "maxStroke") {
                        m_dmInfo.maxStroke = std::stod(value);
                    }
                }
            }
        } else if (metadataFileTrimmed.empty()) {
            // Use config parameters instead of file
            if (!m_configDeviceName.empty()) {
                m_dmInfo.name = m_configDeviceName;
            }
            if (m_configNumActuators > 0) {
                m_dmInfo.numActuators = m_configNumActuators;
            }
            if (m_configGridWidth > 0) {
                m_dmInfo.width = m_configGridWidth;
            }
            if (m_configGridHeight > 0) {
                m_dmInfo.height = m_configGridHeight;
            }
            if (m_configActuatorSpacing > 0) {
                m_dmInfo.actuatorSpacing = m_configActuatorSpacing;
            }
            if (m_configMaxStroke > 0) {
                m_dmInfo.maxStroke = m_configMaxStroke;
            }
            if (!m_configDmType.empty() && m_dmInfo.name.empty()) {
                m_dmInfo.name = m_configDmType;
            }
        }
        
        // Initialize default values if not loaded
        if (m_dmInfo.numActuators == 0) m_dmInfo.numActuators = 1024; // Default
        if (m_dmInfo.width == 0) m_dmInfo.width = 32; // Default
        if (m_dmInfo.height == 0) m_dmInfo.height = 32; // Default
        if (m_dmInfo.actuatorSpacing == 0) m_dmInfo.actuatorSpacing = 0.5; // Default 0.5mm
        if (m_dmInfo.maxStroke == 0) m_dmInfo.maxStroke = 10.0; // Default 10 microns
        
        // Initialize actuator arrays
        m_dmInfo.actuators.resize(m_dmInfo.numActuators);
        m_dmInfo.actuatorMask.resize(m_dmInfo.height, m_dmInfo.width);
        m_dmInfo.actuatorGains.resize(m_dmInfo.height, m_dmInfo.width);
        
        // Set default values
        m_dmInfo.actuatorMask.setConstant(true); // All actuators valid by default
        m_dmInfo.actuatorGains.setConstant(1.0); // Unity gain by default
        
        // Initialize actuators
        for (int i = 0; i < m_dmInfo.numActuators; ++i) {
            m_dmInfo.actuators[i].id = i;
            m_dmInfo.actuators[i].x = (i % m_dmInfo.width) * m_dmInfo.actuatorSpacing;
            m_dmInfo.actuators[i].y = (i / m_dmInfo.width) * m_dmInfo.actuatorSpacing;
            m_dmInfo.actuators[i].gain = 1.0;
            m_dmInfo.actuators[i].minValue = -m_dmInfo.maxStroke;
            m_dmInfo.actuators[i].maxValue = m_dmInfo.maxStroke;
            m_dmInfo.actuators[i].isDead = false;
            m_dmInfo.actuators[i].couplingFactor = 0.0;
        }
        
        // Mark dead actuators (from config or file)
        if (!m_configDeadActuators.empty()) {
            for (int deadId : m_configDeadActuators) {
                if (deadId >= 0 && deadId < m_dmInfo.numActuators) {
                    m_dmInfo.actuators[deadId].isDead = true;
                    m_dmInfo.actuatorMask(deadId / m_dmInfo.width, deadId % m_dmInfo.width) = false;
                }
            }
        }
        
        derived().template log<text_log>("Loaded DM metadata: " + std::to_string(m_dmInfo.numActuators) + " actuators");
        return 0;
        
    } catch (const std::exception& e) {
        derived().template log<software_error>({__FILE__, __LINE__, "Error loading DM metadata: " + std::string(e.what())});
        return -1;
    }
}

template<class derivedT>
int dmWavefrontControl<derivedT>::loadModeSets()
{
    try {
        if (m_modeSetFiles.empty()) {
            derived().template log<text_log>("No modeset files configured");
            return 0;
        }
        
        // Load each modeset
        for (size_t i = 0; i < m_modeSetFiles.size(); ++i) {
            std::string name = (i < m_modeSetNames.size()) ? m_modeSetNames[i] : "modeset_" + std::to_string(i);
            if (loadModeSet(m_modeSetFiles[i], name) < 0) {
                derived().template log<software_error>({__FILE__, __LINE__, "Failed to load modeset: " + m_modeSetFiles[i]});
                return -1;
            }
        }
        
        // Set default modeset if specified
        if (!m_defaultModeSet.empty()) {
            if (m_modeSetMap.find(m_defaultModeSet) == m_modeSetMap.end()) {
                derived().template log<software_warning>({__FILE__, __LINE__, "Default modeset not found: " + m_defaultModeSet});
                if (!m_modeSets.empty()) {
                    m_defaultModeSet = m_modeSets[0].name;
                    derived().template log<text_log>("Using first modeset as default: " + m_defaultModeSet);
                }
            }
        } else if (!m_modeSets.empty()) {
            m_defaultModeSet = m_modeSets[0].name;
            derived().template log<text_log>("Using first modeset as default: " + m_defaultModeSet);
        }
        
        derived().template log<text_log>("Loaded " + std::to_string(m_modeSets.size()) + " modesets");
        return 0;
        
    } catch (const std::exception& e) {
        derived().template log<software_error>({__FILE__, __LINE__, "Error loading modesets: " + std::string(e.what())});
        return -1;
    }
}

template<class derivedT>
int dmWavefrontControl<derivedT>::loadModeSet(const std::string& filename, const std::string& name)
{
    try {
        // Load the FITS file - simplified for now
        // TODO: Implement proper FITS loading when mx::fits API is available
        
        // Create new modeset with placeholder data
        ModeSet modeset;
        modeset.name = name;
        modeset.filename = filename;
        
        // Try to load from FITS file first
        fitsfile *fptr;
        int status = 0;
        
        if (!fits_open_image(&fptr, filename.c_str(), READONLY, &status)) {
            int hdutype, naxis;
            long naxes[3];
            
            if (fits_get_hdu_type(fptr, &hdutype, &status) || hdutype != IMAGE_HDU) {
                derived().template log<software_warning>({__FILE__, __LINE__, "FITS file is not an image: " + filename});
            } else {
                fits_get_img_dim(fptr, &naxis, &status);
                fits_get_img_size(fptr, 3, naxes, &status);
                
                if (status == 0 && naxis == 3) {
                    // 3D FITS file: FITS stores as [width, height, planes] in FORTRAN order
                    // NAXIS1 = width, NAXIS2 = height, NAXIS3 = number of modes/planes
                    int width = naxes[0];   // NAXIS1
                    int height = naxes[1];  // NAXIS2
                    int numModes = naxes[2]; // NAXIS3
                    
                    modeset.modes.resize(numModes, height, width);
                    
                    // Read the data plane by plane
                    // FITS pixel coordinates: [x, y, z] = [width, height, plane]
                    long fpixel[3];
                    std::vector<float> buffer(width * height);
                    
                    for (int p = 0; p < numModes; ++p) {
                        // Set starting pixel: x=1, y=1, z=plane+1 (1-indexed)
                        fpixel[0] = 1;
                        fpixel[1] = 1;
                        fpixel[2] = p + 1;
                        
                        if (fits_read_pix(fptr, TFLOAT, fpixel, width * height, 0, buffer.data(), 0, &status) == 0) {
                            // Copy to eigen matrix
                            // Buffer is in row-major order: buffer[i] = data[y*width + x]
                            for (int y = 0; y < height; ++y) {
                                for (int x = 0; x < width; ++x) {
                                    modeset.modes.image(p)(y, x) = buffer[y * width + x];
                                }
                            }
                        } else {
                            status = 0; // Reset status for next iteration
                        }
                    }
                    
                    // Try to read mode names from header keywords
                    modeset.modeNames.resize(numModes);
                    for (int p = 0; p < numModes; ++p) {
                        std::string keyword = "MODE" + std::to_string(p + 1);
                        char value[FLEN_VALUE];
                        if (fits_read_key(fptr, TSTRING, keyword.c_str(), value, NULL, &status) == 0) {
                            modeset.modeNames[p] = value;
                        } else {
                            modeset.modeNames[p] = "Mode" + std::to_string(p + 1);
                        }
                        status = 0; // Reset status for next keyword
                    }
                    
                    // Try to read mode scales and gains
                    modeset.modeScales.resize(numModes, 1.0);
                    modeset.modalGains.resize(numModes, 1.0);
                    modeset.modeMin.resize(numModes, -10.0);
                    modeset.modeMax.resize(numModes, 10.0);
                    
                    for (int p = 0; p < numModes; ++p) {
                        std::string scaleKey = "SCALE" + std::to_string(p + 1);
                        std::string gainKey = "GAIN" + std::to_string(p + 1);
                        std::string minKey = "MIN" + std::to_string(p + 1);
                        std::string maxKey = "MAX" + std::to_string(p + 1);
                        
                        float value;
                        char comment[FLEN_COMMENT];
                        if (fits_read_key(fptr, TFLOAT, scaleKey.c_str(), &value, comment, &status) == 0) {
                            modeset.modeScales[p] = value;
                        }
                        if (fits_read_key(fptr, TFLOAT, gainKey.c_str(), &value, comment, &status) == 0) {
                            modeset.modalGains[p] = value;
                        }
                        if (fits_read_key(fptr, TFLOAT, minKey.c_str(), &value, comment, &status) == 0) {
                            modeset.modeMin[p] = value;
                        }
                        if (fits_read_key(fptr, TFLOAT, maxKey.c_str(), &value, comment, &status) == 0) {
                            modeset.modeMax[p] = value;
                        }
                        status = 0; // Reset status for next keyword
                    }
                    
                    fits_close_file(fptr, &status);
                    
                    // Ensure name is set correctly
                    modeset.name = name;
                    modeset.filename = filename;
                    
                    // Add to modesets list
                    m_modeSets.push_back(modeset);
                    size_t modesetIndex = m_modeSets.size() - 1;
                    m_modeSetMap[name] = modesetIndex;
                    
                    derived().template log<text_log>("Loaded modeset '" + name + "' with " + std::to_string(numModes) + " modes from FITS file: " + filename);
                    derived().template log<text_log>("Modeset added to map at index " + std::to_string(modesetIndex) + ", map size: " + std::to_string(m_modeSetMap.size()));
                    return 0;
                } else if (status == 0 && naxis == 2) {
                    // 2D FITS file: treat as single mode
                    int height = naxes[0];
                    int width = naxes[1];
                    
                    modeset.modes.resize(1, height, width);
                    
                    // Read the data
                    long fpixel[2] = {1, 1};
                    std::vector<float> buffer(width * height);
                    
                    if (fits_read_pix(fptr, TFLOAT, fpixel, width * height, 0, buffer.data(), 0, &status) == 0) {
                        // Copy to eigen matrix
                        for (int y = 0; y < height; ++y) {
                            for (int x = 0; x < width; ++x) {
                                modeset.modes.image(0)(y, x) = buffer[y * width + x];
                            }
                        }
                    }
                    
                    modeset.modeNames.resize(1);
                    modeset.modeNames[0] = name;
                    modeset.modeScales.resize(1, 1.0);
                    modeset.modalGains.resize(1, 1.0);
                    modeset.modeMin.resize(1, -10.0);
                    modeset.modeMax.resize(1, 10.0);
                    
                    fits_close_file(fptr, &status);
                    
                    // Add to modesets list
                    m_modeSets.push_back(modeset);
                    m_modeSetMap[name] = m_modeSets.size() - 1;
                    
                    derived().template log<text_log>("Loaded modeset '" + name + "' with 1 mode from 2D FITS file: " + filename);
                    return 0;
                }
            }
            fits_close_file(fptr, &status);
        }
        
        // Fallback: create a simple modeset if FITS loading fails
        derived().template log<text_log>("FITS loading failed, creating default modeset for: " + name);
        int width = 32, height = 32, numModes = 10;
        modeset.modes.resize(numModes, height, width);
        
        // Initialize with zeros
        for (int p = 0; p < modeset.modes.planes(); ++p) {
            for (int y = 0; y < modeset.modes.rows(); ++y) {
                for (int x = 0; x < modeset.modes.cols(); ++x) {
                    modeset.modes.image(p)(y, x) = 0.0f;
                }
            }
        }
        
        // Generate mode names
        modeset.modeNames.resize(numModes);
        for (int m = 0; m < numModes; ++m) {
            if (name == "zernike") {
                // Zernike mode naming convention
                int n = static_cast<int>(std::sqrt(2 * m + 0.25) - 0.5);
                int l = m - (n * (n + 1)) / 2;
                modeset.modeNames[m] = "Z" + std::to_string(m + 1) + "(" + std::to_string(n) + "," + std::to_string(l) + ")";
            } else if (name == "hadamard") {
                modeset.modeNames[m] = "H" + std::to_string(m + 1);
            } else {
                modeset.modeNames[m] = name.substr(0, 1) + std::to_string(m + 1);
            }
        }
        
        // Set default scales and gains
        modeset.modeScales.resize(numModes, 1.0); // Default 1 micron/coefficient
        modeset.modalGains.resize(numModes, 1.0); // Default unity gain
        modeset.modeMin.resize(numModes, -10.0);  // Default -10 microns
        modeset.modeMax.resize(numModes, 10.0);   // Default +10 microns
        
        // Add to modesets list
        m_modeSets.push_back(modeset);
        m_modeSetMap[name] = m_modeSets.size() - 1;
        
        derived().template log<text_log>("Loaded modeset '" + name + "' with " + std::to_string(numModes) + " modes from " + filename);
        return 0;
        
    } catch (const std::exception& e) {
        derived().template log<software_error>({__FILE__, __LINE__, "Error loading modeset " + name + " from " + filename + ": " + std::string(e.what())});
        return -1;
    }
}

template<class derivedT>
int dmWavefrontControl<derivedT>::pokeActuator(int actuatorId, double amplitude)
{
    try {
        // Validate actuator ID
        if (actuatorId < 0 || actuatorId >= m_dmInfo.numActuators) {
            derived().template log<software_error>({__FILE__, __LINE__, "Invalid actuator ID: " + std::to_string(actuatorId)});
            return -1;
        }
        
        // Check if actuator is dead
        if (m_dmInfo.actuators[actuatorId].isDead) {
            derived().template log<text_log>("Skipping dead actuator: " + std::to_string(actuatorId));
            return 0;
        }
        
        // Apply limits and gain
        double limitedAmplitude = std::max(m_dmInfo.actuators[actuatorId].minValue,
                                         std::min(m_dmInfo.actuators[actuatorId].maxValue, amplitude));
        double finalAmplitude = limitedAmplitude * m_dmInfo.actuators[actuatorId].gain;
        
        // TODO: Send command to actual DM hardware
        // This would interface with the DM driver
        derived().template log<text_log>("Poking actuator " + std::to_string(actuatorId) + " with amplitude " + std::to_string(finalAmplitude));
        
        return 0;
        
    } catch (const std::exception& e) {
        derived().template log<software_error>({__FILE__, __LINE__, "Error poking actuator: " + std::string(e.what())});
        return -1;
    }
}

template<class derivedT>
int dmWavefrontControl<derivedT>::applyModalMode(const std::string& modesetName, int modeIndex, double amplitude)
{
    try {
        // Find the modeset
        auto it = m_modeSetMap.find(modesetName);
        if (it == m_modeSetMap.end()) {
            derived().template log<software_error>({__FILE__, __LINE__, "Modeset not found: " + modesetName});
            return -1;
        }
        
        const ModeSet& modeset = m_modeSets[it->second];
        
        // Validate mode index
        if (modeIndex < 0 || modeIndex >= modeset.modes.planes()) {
            derived().template log<software_error>({__FILE__, __LINE__, "Invalid mode index: " + std::to_string(modeIndex) + " for modeset " + modesetName});
            return -1;
        }
        
        // Apply limits and scaling
        double limitedAmplitude = std::max(modeset.modeMin[modeIndex],
                                         std::min(modeset.modeMax[modeIndex], amplitude));
        double scaledAmplitude = limitedAmplitude * modeset.modeScales[modeIndex] * modeset.modalGains[modeIndex];
        
        // Convert modal command to actuator commands
        std::vector<double> actuatorCommands(m_dmInfo.numActuators, 0.0);
        
        for (int y = 0; y < modeset.modes.rows(); ++y) {
            for (int x = 0; x < modeset.modes.cols(); ++x) {
                int actuatorId = y * modeset.modes.cols() + x;
                if (actuatorId < m_dmInfo.numActuators) {
                    actuatorCommands[actuatorId] = modeset.modes.image(modeIndex)(y, x) * scaledAmplitude;
                }
            }
        }
        
        // Apply actuator commands
        for (int i = 0; i < m_dmInfo.numActuators; ++i) {
            if (std::abs(actuatorCommands[i]) > 1e-6) { // Only apply non-zero commands
                if (pokeActuator(i, actuatorCommands[i]) < 0) {
                    derived().template log<software_error>({__FILE__, __LINE__, "Failed to apply actuator command for actuator " + std::to_string(i)});
                    return -1;
                }
            }
        }
        
        derived().template log<text_log>("Applied modal mode " + modeset.modeNames[modeIndex] + " with amplitude " + std::to_string(amplitude));
        return 0;
        
    } catch (const std::exception& e) {
        derived().template log<software_error>({__FILE__, __LINE__, "Error applying modal mode: " + std::string(e.what())});
        return -1;
    }
}

template<class derivedT>
int dmWavefrontControl<derivedT>::applyMixedModes(const std::vector<std::pair<std::string, double>>& commands)
{
    try {
        // Convert all modal commands to actuator commands
        std::vector<double> actuatorCommands = modalToActuator(commands);
        
        // Apply the combined actuator commands
        for (int i = 0; i < m_dmInfo.numActuators; ++i) {
            if (std::abs(actuatorCommands[i]) > 1e-6) { // Only apply non-zero commands
                if (pokeActuator(i, actuatorCommands[i]) < 0) {
                    derived().template log<software_error>({__FILE__, __LINE__, "Failed to apply actuator command for actuator " + std::to_string(i)});
                    return -1;
                }
            }
        }
        
        derived().template log<text_log>("Applied mixed modes with " + std::to_string(commands.size()) + " commands");
        return 0;
        
    } catch (const std::exception& e) {
        derived().template log<software_error>({__FILE__, __LINE__, "Error applying mixed modes: " + std::string(e.what())});
        return -1;
    }
}

template<class derivedT>
std::vector<double> dmWavefrontControl<derivedT>::modalToActuator(const std::vector<std::pair<std::string, double>>& modalCommands)
{
    std::vector<double> actuatorCommands(m_dmInfo.numActuators, 0.0);
    
    try {
        for (const auto& command : modalCommands) {
            const std::string& modesetName = command.first;
            double amplitude = command.second;
            
            // Parse modeset:mode format
            size_t colonPos = modesetName.find(':');
            if (colonPos == std::string::npos) {
                derived().template log<software_error>({__FILE__, __LINE__, "Invalid modal command format: " + modesetName + " (expected 'modeset:mode')"});
                continue;
            }
            
            std::string modeset = modesetName.substr(0, colonPos);
            int modeIndex = std::stoi(modesetName.substr(colonPos + 1)) - 1; // Convert to 0-based index
            
            // Find the modeset
            auto it = m_modeSetMap.find(modeset);
            if (it == m_modeSetMap.end()) {
                derived().template log<software_error>({__FILE__, __LINE__, "Modeset not found: " + modeset});
                continue;
            }
            
            const ModeSet& modesetData = m_modeSets[it->second];
            
            // Validate mode index
            if (modeIndex < 0 || modeIndex >= modesetData.modes.planes()) {
                derived().template log<software_error>({__FILE__, __LINE__, "Invalid mode index: " + std::to_string(modeIndex + 1) + " for modeset " + modeset});
                continue;
            }
            
            // Apply limits and scaling
            double limitedAmplitude = std::max(modesetData.modeMin[modeIndex],
                                             std::min(modesetData.modeMax[modeIndex], amplitude));
            double scaledAmplitude = limitedAmplitude * modesetData.modeScales[modeIndex] * modesetData.modalGains[modeIndex];
            
            // Add to actuator commands
            for (int y = 0; y < modesetData.modes.rows(); ++y) {
                for (int x = 0; x < modesetData.modes.cols(); ++x) {
                    int actuatorId = y * modesetData.modes.cols() + x;
                    if (actuatorId < m_dmInfo.numActuators) {
                        actuatorCommands[actuatorId] += modesetData.modes.image(modeIndex)(y, x) * scaledAmplitude;
                    }
                }
            }
        }
        
    } catch (const std::exception& e) {
        derived().template log<software_error>({__FILE__, __LINE__, "Error converting modal commands: " + std::string(e.what())});
    }
    
    return actuatorCommands;
}

template<class derivedT>
int dmWavefrontControl<derivedT>::updateMLAT(double dmCommandTime, double cameraFrameTime)
{
    try {
        // Calculate round-trip latency
        m_mlat = (cameraFrameTime - dmCommandTime) * 1000.0; // Convert to microseconds
        
        // Update timing buffers and statistics
        if (m_mlatBuffer.size() < m_timingBufferSize) {
            m_mlatBuffer.push_back(m_mlat);
        } else {
            m_mlatBuffer[m_timingBufferIndex] = m_mlat;
        }
        
        // Update buffer index
        m_timingBufferIndex = (m_timingBufferIndex + 1) % m_timingBufferSize;
        
        // Calculate running statistics
        if (!m_mlatBuffer.empty()) {
            double sum = 0.0, sumSq = 0.0;
            for (double val : m_mlatBuffer) {
                sum += val;
                sumSq += val * val;
            }
            m_mlatAvg = sum / m_mlatBuffer.size();
            m_mlatStd = sqrt((sumSq / m_mlatBuffer.size()) - (m_mlatAvg * m_mlatAvg));
        }
        
        // Update INDI properties
        derived().updateIfChanged(m_indiP_mlat, "current", m_mlat, INDI_IDLE);
        derived().updateIfChanged(m_indiP_mlat, "average", m_mlatAvg, INDI_IDLE);
        derived().updateIfChanged(m_indiP_mlat, "stddev", m_mlatStd, INDI_IDLE);
        

        
        return 0;
        
    } catch (const std::exception& e) {
        derived().template log<software_error>({__FILE__, __LINE__, "Error updating MLAT: " + std::string(e.what())});
        return -1;
    }
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
    

    
    // Enhanced configuration for modesets and DM metadata
    config.add("dmWavefrontControl.modesets", "", "dmWavefrontControl.modesets", 
               argType::Required, "dmWavefrontControl", "modesets", false, "vector<string>", 
               "List of modeset FITS files");
    
    config.add("dmWavefrontControl.modeset_names", "", "dmWavefrontControl.modeset_names", 
               argType::Required, "dmWavefrontControl", "modeset_names", false, "vector<string>", 
               "Names for each modeset");
    
    config.add("dmWavefrontControl.default_modeset", "", "dmWavefrontControl.default_modeset", 
               argType::Required, "dmWavefrontControl", "default_modeset", false, "string", 
               "Default modeset to use");
    
    config.add("dmWavefrontControl.modes_to_optimize", "", "dmWavefrontControl.modes_to_optimize", 
               argType::Required, "dmWavefrontControl", "modes_to_optimize", false, "vector<string>", 
               "Specific modes to optimize (format: modeset:mode)");
    
    // DM metadata configuration (optional - if empty, use config parameters)
    config.add("dmWavefrontControl.metadata_file", "", "dmWavefrontControl.metadata_file", 
               argType::Optional, "dmWavefrontControl", "metadata_file", false, "string", 
               "DM metadata configuration file (optional, leave empty to use config parameters)");
    
    config.add("dmWavefrontControl.actuator_spacing", "", "dmWavefrontControl.actuator_spacing", 
               argType::Required, "dmWavefrontControl", "actuator_spacing", false, "float", 
               "Actuator spacing in mm");
    
    config.add("dmWavefrontControl.max_stroke", "", "dmWavefrontControl.max_stroke", 
               argType::Required, "dmWavefrontControl", "max_stroke", false, "float", 
               "Maximum stroke in microns");
    
    // DM metadata parameters (used when metadata_file is empty)
    config.add("dmWavefrontControl.numActuators", "", "dmWavefrontControl.numActuators", 
               argType::Required, "dmWavefrontControl", "numActuators", false, "int", 
               "Number of actuators in the DM");
    config.add("dmWavefrontControl.gridWidth", "", "dmWavefrontControl.gridWidth", 
               argType::Required, "dmWavefrontControl", "gridWidth", false, "int", 
               "Actuator grid width");
    config.add("dmWavefrontControl.gridHeight", "", "dmWavefrontControl.gridHeight", 
               argType::Required, "dmWavefrontControl", "gridHeight", false, "int", 
               "Actuator grid height");
    config.add("dmWavefrontControl.dmType", "", "dmWavefrontControl.dmType", 
               argType::Required, "dmWavefrontControl", "dmType", false, "string", 
               "DM type/manufacturer");
    config.add("dmWavefrontControl.deadActuators", "", "dmWavefrontControl.deadActuators", 
               argType::Required, "dmWavefrontControl", "deadActuators", false, "vector<int>", 
               "List of dead actuator indices (comma-separated)");
    config.add("dmWavefrontControl.deviceName", "", "dmWavefrontControl.deviceName", 
               argType::Optional, "dmWavefrontControl", "deviceName", false, "string", 
               "DM device/shared memory name (e.g., dm01disp06)");
    config.add("dmWavefrontControl.couplingMatrix", "", "dmWavefrontControl.couplingMatrix", 
               argType::Optional, "dmWavefrontControl", "couplingMatrix", false, "string", 
               "Path to coupling matrix file (optional)");
    config.add("dmWavefrontControl.actuatorGains", "", "dmWavefrontControl.actuatorGains", 
               argType::Optional, "dmWavefrontControl", "actuatorGains", false, "string", 
               "Path to actuator gains file (optional)");
    config.add("dmWavefrontControl.actuatorLimits", "", "dmWavefrontControl.actuatorLimits", 
               argType::Optional, "dmWavefrontControl", "actuatorLimits", false, "string", 
               "Path to actuator limits file (optional)");
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

    
    // Load enhanced configuration
    config(m_modeSetFiles, "dmWavefrontControl.modesets");
    config(m_modeSetNames, "dmWavefrontControl.modeset_names");
    config(m_defaultModeSet, "dmWavefrontControl.default_modeset");
    config(m_modesToOptimize, "dmWavefrontControl.modes_to_optimize");
    
    // Load DM metadata configuration (optional)
    config(m_dmMetadataFile, "dmWavefrontControl.metadata_file");
    // If empty string, clear it
    if (m_dmMetadataFile == "") {
        m_dmMetadataFile.clear();
    }
    
    // Load DM metadata parameters (used if metadata_file is empty)
    config(m_configDeviceName, "dmWavefrontControl.deviceName");
    config(m_configNumActuators, "dmWavefrontControl.numActuators");
    config(m_configGridWidth, "dmWavefrontControl.gridWidth");
    config(m_configGridHeight, "dmWavefrontControl.gridHeight");
    config(m_configActuatorSpacing, "dmWavefrontControl.actuator_spacing");
    config(m_configMaxStroke, "dmWavefrontControl.max_stroke");
    config(m_configDmType, "dmWavefrontControl.dmType");
    config(m_configDeadActuators, "dmWavefrontControl.deadActuators");
    config(m_configCouplingMatrix, "dmWavefrontControl.couplingMatrix");
    config(m_configActuatorGains, "dmWavefrontControl.actuatorGains");
    config(m_configActuatorLimits, "dmWavefrontControl.actuatorLimits");
}

template<class derivedT>
int dmWavefrontControl<derivedT>::appStartup()
{
    // Initialize timing infrastructure
    if(initializeTiming() < 0)
    {
        return -1;
    }
    
    // Load DM metadata and modesets
    if(loadDMMetadata() < 0)
    {
        return -1;
    }
    
    if(loadModeSets() < 0)
    {
        return -1;
    }
    
    // Create INDI properties
    derived().createStandardIndiNumber(m_indiP_dmPokeAmplitude, "dmPokeAmplitude", 0.001, 10.0, 0.001, "%0.3f");
    derived().createStandardIndiNumber(m_indiP_dmPokeDelay, "dmPokeDelay", 100, 100000, 100, "%0.0f");
    derived().createStandardIndiText(m_indiP_wfsCamera, "wfsCamera", "current", "WFS Camera");
    derived().createStandardIndiNumber(m_indiP_psfCoreRadius, "psfCoreRadius", 1.0, 100.0, 0.1, "%0.1f");
    derived().createStandardIndiNumber(m_indiP_searchRange, "searchRange", 0.01, 10.0, 0.01, "%0.2f");

    
    // Create enhanced INDI properties
    derived().createStandardIndiText(m_indiP_modesets, "modesets", "current", "Available Modesets");
    derived().createStandardIndiText(m_indiP_defaultModeSet, "defaultModeSet", "current", "Default Modeset");
    derived().createStandardIndiText(m_indiP_modesToOptimize, "modesToOptimize", "current", "Modes to Optimize");
    derived().createStandardIndiNumber(m_indiP_mlat, "mlat", 0, 10000, 1, "%0.1f");
    derived().createStandardIndiNumber(m_indiP_dmCommandLatency, "dmCommandLatency", 0, 10000, 1, "%0.1f");
    derived().createStandardIndiNumber(m_indiP_cameraResponseLatency, "cameraResponseLatency", 0, 10000, 1, "%0.1f");
    
            // Register INDI properties
        // Register properties that require callbacks (derived classes must implement these)
        if(derived().registerIndiPropertyNew(m_indiP_dmPokeAmplitude, &derivedT::st_newCallBack_m_indiP_dmPokeAmplitude) < 0) return -1;
        if(derived().registerIndiPropertyNew(m_indiP_dmPokeDelay, &derivedT::st_newCallBack_m_indiP_dmPokeDelay) < 0) return -1;
        if(derived().registerIndiPropertyNew(m_indiP_wfsCamera, &derivedT::st_newCallBack_m_indiP_wfsCamera) < 0) return -1;
        if(derived().registerIndiPropertyNew(m_indiP_psfCoreRadius, &derivedT::st_newCallBack_m_indiP_psfCoreRadius) < 0) return -1;
        if(derived().registerIndiPropertyNew(m_indiP_searchRange, &derivedT::st_newCallBack_m_indiP_searchRange) < 0) return -1;

        
        // Register enhanced INDI properties
        if(derived().registerIndiPropertyReadOnly(m_indiP_modesets) < 0) return -1;
        if(derived().registerIndiPropertyReadOnly(m_indiP_defaultModeSet) < 0) return -1;
        if(derived().registerIndiPropertyReadOnly(m_indiP_modesToOptimize) < 0) return -1;
        if(derived().registerIndiPropertyReadOnly(m_indiP_mlat) < 0) return -1;
        if(derived().registerIndiPropertyReadOnly(m_indiP_dmCommandLatency) < 0) return -1;
        if(derived().registerIndiPropertyReadOnly(m_indiP_cameraResponseLatency) < 0) return -1;
    
    return 0;
}

template<class derivedT>
int dmWavefrontControl<derivedT>::appLogic()
{
    // Basic optimization framework - derived classes should override this
    if(m_optimizationRunning)
    {
        // This would be implemented by derived classes
        // For now, just provide the framework
    }
    
    // Update modeset INDI properties
    std::string modesetList;
    for (const auto& modeset : m_modeSets) {
        if (!modesetList.empty()) modesetList += ",";
        modesetList += modeset.name + "(" + std::to_string(modeset.modes.planes()) + " modes)";
    }
    
    derived().updateIfChanged(m_indiP_modesets, "current", modesetList, INDI_IDLE);
    derived().updateIfChanged(m_indiP_defaultModeSet, "current", m_defaultModeSet, INDI_IDLE);
    
    // Update modes to optimize
    std::string optimizeList;
    for (const auto& mode : m_modesToOptimize) {
        if (!optimizeList.empty()) optimizeList += ",";
        optimizeList += mode;
    }
    derived().updateIfChanged(m_indiP_modesToOptimize, "current", optimizeList, INDI_IDLE);
    
    return 0;
}

template<class derivedT>
int dmWavefrontControl<derivedT>::appShutdown()
{
    // Stop any running optimization
    if(m_optimizationRunning)
    {
        // Derived classes should implement their own stop logic
        m_optimizationRunning = false;
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
    try {
        if (m_modeSets.empty()) {
            derived().template log<software_error>({__FILE__, __LINE__, "No modesets loaded"});
            return -1;
        }
        
        if (m_defaultModeSet.empty()) {
            derived().template log<software_error>({__FILE__, __LINE__, "No default modeset specified"});
            return -1;
        }
        
        // Find the default modeset
        auto it = m_modeSetMap.find(m_defaultModeSet);
        if (it == m_modeSetMap.end()) {
            derived().template log<software_error>({__FILE__, __LINE__, "Default modeset not found: " + m_defaultModeSet});
            return -1;
        }
        
        const ModeSet& modeset = m_modeSets[it->second];
        
        // Validate mode count
        if (static_cast<mx::improc::eigenCube<float>::Index>(modes.size()) > modeset.modes.planes()) {
            derived().template log<software_error>({__FILE__, __LINE__, "Too many modes: " + std::to_string(modes.size()) + " > " + std::to_string(modeset.modes.planes())});
            return -1;
        }
        
        // Apply each mode
        for (size_t i = 0; i < modes.size(); ++i) {
            if (std::abs(modes[i]) > 1e-6) { // Only apply non-zero modes
                if (applyModalMode(m_defaultModeSet, i, modes[i]) < 0) {
                    derived().template log<software_error>({__FILE__, __LINE__, "Failed to apply mode " + std::to_string(i)});
                    return -1;
                }
            }
        }
        
        derived().template log<text_log>("Applied " + std::to_string(modes.size()) + " modes using modeset: " + m_defaultModeSet);
        return 0;
        
    } catch (const std::exception& e) {
        derived().template log<software_error>({__FILE__, __LINE__, "Error applying modes: " + std::string(e.what())});
        return -1;
    }
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
    // Store for derived classes to use if needed
    // m_currentLatency = static_cast<double>(latency.count());
    
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

// Image processing and PSF analysis implementations
template<class derivedT>
eigenImage<float> dmWavefrontControl<derivedT>::subtractBackground(const eigenImage<float>& image, int method)
{
    eigenImage<float> result = image;
    
    switch(method) {
        case 0: // full image median
            result.array() -= image.mean();
            break;
            
        case 1: { // edge median
            int edgeSize = 5;
            std::vector<float> edgeValues;
            
            // Collect edge values
            for(int y = 0; y < edgeSize; ++y) {
                for(int x = 0; x < image.cols(); ++x) {
                    edgeValues.push_back(image(y, x));
                    edgeValues.push_back(image(image.rows()-1-y, x));
                }
            }
            for(int y = edgeSize; y < image.rows()-edgeSize; ++y) {
                for(int x = 0; x < edgeSize; ++x) {
                    edgeValues.push_back(image(y, x));
                    edgeValues.push_back(image(y, image.cols()-1-x));
                }
            }
            
            // Calculate median
            std::sort(edgeValues.begin(), edgeValues.end());
            float median = edgeValues[edgeValues.size()/2];
            result.array() -= median;
            break;
        }
        
        case 2: // mode (most frequent value)
            // For simplicity, use median as approximation
            result.array() -= image.mean();
            break;
            
        case 3: { // row by row and column by column
            // Row median
            for(int x = 0; x < image.cols(); ++x) {
                float rowMedian = 0;
                std::vector<float> rowValues;
                for(int y = 0; y < image.rows(); ++y) {
                    rowValues.push_back(image(y, x));
                }
                std::sort(rowValues.begin(), rowValues.end());
                rowMedian = rowValues[rowValues.size()/2];
                for(int y = 0; y < image.rows(); ++y) {
                    result(y, x) -= rowMedian;
                }
            }
            
            // Column median
            for(int y = 0; y < image.rows(); ++y) {
                float colMedian = 0;
                std::vector<float> colValues;
                for(int x = 0; x < image.cols(); ++x) {
                    colValues.push_back(result(y, x));
                }
                std::sort(colValues.begin(), colValues.end());
                colMedian = colValues[colValues.size()/2];
                for(int x = 0; x < image.cols(); ++x) {
                    result(y, x) -= colMedian;
                }
            }
            
            // Global median
            float globalMedian = 0;
            std::vector<float> allValues;
            for(int y = 0; y < result.rows(); ++y) {
                for(int x = 0; x < result.cols(); ++x) {
                    allValues.push_back(result(y, x));
                }
            }
            std::sort(allValues.begin(), allValues.end());
            globalMedian = allValues[allValues.size()/2];
            result.array() -= globalMedian;
            break;
        }
    }
    
    return result;
}

template<class derivedT>
double dmWavefrontControl<derivedT>::findPeak(const eigenImage<float>& image, int method, int clipping)
{
    if(method == 0) {
        // Naive maximum
        return image.maxCoeff();
    } else {
        // Gaussian fit (simplified - return max for now)
        // TODO: Implement proper Gaussian fitting
        return image.maxCoeff();
    }
}

template<class derivedT>
double dmWavefrontControl<derivedT>::computeCoreSum(const eigenImage<float>& image, double radius, std::pair<double, double> center)
{
    eigenImage<float> bgSub = subtractBackground(image, 1); // edge median
    
    // Find center if not provided
    double ceny, cenx;
    if(center.first < 0 || center.second < 0) {
        // Find peak
        int maxY, maxX;
        bgSub.maxCoeff(&maxY, &maxX);
        ceny = static_cast<double>(maxY);
        cenx = static_cast<double>(maxX);
    } else {
        ceny = center.first;
        cenx = center.second;
    }
    
    // Create core mask
    eigenImage<bool> coreMask = createCircularMask({ceny, cenx}, radius, {image.rows(), image.cols()});
    
    // Compute core sum
    double coreSum = 0;
    int count = 0;
    for(int y = 0; y < image.rows(); ++y) {
        for(int x = 0; x < image.cols(); ++x) {
            if(coreMask(y, x)) {
                coreSum += bgSub(y, x);
                count++;
            }
        }
    }
    
    return count > 0 ? coreSum / count : 0;
}

template<class derivedT>
double dmWavefrontControl<derivedT>::computeCoreRingRatio(const eigenImage<float>& image, double radius1, double radius2)
{
    eigenImage<float> bgSub = subtractBackground(image, 1); // edge median
    
    // Find center
    int maxY, maxX;
    bgSub.maxCoeff(&maxY, &maxX);
    double ceny = static_cast<double>(maxY);
    double cenx = static_cast<double>(maxX);
    
    // Create masks
    eigenImage<bool> coreMask = createCircularMask({ceny, cenx}, radius1, {image.rows(), image.cols()});
    eigenImage<bool> annulusMask = createCircularMask({ceny, cenx}, radius2, {image.rows(), image.cols()});
    
    // Remove core from annulus
    for(int y = 0; y < image.rows(); ++y) {
        for(int x = 0; x < image.cols(); ++x) {
            if(coreMask(y, x)) {
                annulusMask(y, x) = false;
            }
        }
    }
    
    // Compute sums
    double coreSum = 0, annulusSum = 0;
    int coreCount = 0, annulusCount = 0;
    
    for(int y = 0; y < image.rows(); ++y) {
        for(int x = 0; x < image.cols(); ++x) {
            if(coreMask(y, x)) {
                coreSum += bgSub(y, x);
                coreCount++;
            }
            if(annulusMask(y, x)) {
                annulusSum += bgSub(y, x);
                annulusCount++;
            }
        }
    }
    
    if(coreCount == 0 || annulusCount == 0) return 999.0;
    
    double ratio = (annulusSum / annulusCount) / (coreSum / coreCount);
    return std::isinf(ratio) || std::isnan(ratio) ? 999.0 : ratio;
}

template<class derivedT>
std::vector<double> dmWavefrontControl<derivedT>::fitGaussian(const eigenImage<float>& image, int clipping)
{
    // Simplified Gaussian fit - return basic parameters
    std::vector<double> result(4);
    
    int maxY, maxX;
    image.maxCoeff(&maxY, &maxX);
    
    result[0] = 10.0; // FWHM (default)
    result[1] = image(maxY, maxX); // peak
    result[2] = static_cast<double>(maxY); // center_y
    result[3] = static_cast<double>(maxX); // center_x
    
    return result;
}

template<class derivedT>
eigenImage<float> dmWavefrontControl<derivedT>::generateGaussian(double fwhm, std::pair<double, double> center, std::pair<int, int> size)
{
    eigenImage<float> result(size.first, size.second);
    
    double sigma = fwhm / (2.0 * sqrt(2.0 * log(2.0)));
    double y0 = center.first;
    double x0 = center.second;
    
    for(int y = 0; y < size.first; ++y) {
        for(int x = 0; x < size.second; ++x) {
            double dy = static_cast<double>(y) - y0;
            double dx = static_cast<double>(x) - x0;
            double r2 = dx*dx + dy*dy;
            result(y, x) = exp(-r2 / (2.0 * sigma * sigma)) / (2.0 * M_PI * sigma * sigma);
        }
    }
    
    return result;
}

template<class derivedT>
double dmWavefrontControl<derivedT>::computeImagePeak(const std::vector<eigenImage<float>>& images)
{
    if(images.empty()) return 0.0;
    
    double totalPeak = 0.0;
    for(const auto& img : images) {
        totalPeak += findPeak(img, 0);
    }
    
    return totalPeak / images.size();
}

template<class derivedT>
double dmWavefrontControl<derivedT>::computeRSS(const std::vector<eigenImage<float>>& images)
{
    if(images.empty()) return 0.0;
    
    double totalRSS = 0.0;
    for(const auto& img : images) {
        double sum = 0.0;
        for(int y = 0; y < img.rows(); ++y) {
            for(int x = 0; x < img.cols(); ++x) {
                sum += img(y, x) * img(y, x);
            }
        }
        totalRSS += sqrt(sum);
    }
    
    return totalRSS / images.size();
}

template<class derivedT>
eigenImage<float> dmWavefrontControl<derivedT>::generateObscuredAiryDisk(double I0, double wavelength, double fnum, double pixscale, 
                                                                        std::pair<double, double> center, std::pair<int, int> size)
{
    eigenImage<float> result(size.first, size.second);
    
    double eta = 0.29; // 29% obscured pupil
    
    for(int y = 0; y < size.first; ++y) {
        for(int x = 0; x < size.second; ++x) {
            double dy = static_cast<double>(y) - center.first;
            double dx = static_cast<double>(x) - center.second;
            double r = sqrt(dx*dx + dy*dy);
            
            double arg = r * M_PI / (wavelength * fnum) * pixscale;
            if(arg == 0) arg = 1e-16;
            
            // Simplified Bessel function approximation
            double t1 = 2.0 * std::cyl_bessel_j(1, arg) / arg;
            double t2 = 2.0 * eta * std::cyl_bessel_j(1, eta * arg) / arg;
            
            result(y, x) = I0 * (t1 - t2) * (t1 - t2) / sqrt(1.0 - eta);
        }
    }
    
    return result;
}

template<class derivedT>
std::vector<double> dmWavefrontControl<derivedT>::fitAiryDisk(const eigenImage<float>& psf, double wavelength, double fnum, double pixscale, int cutout)
{
    // Simplified Airy disk fit
    std::vector<double> result(3);
    
    int maxY, maxX;
    psf.maxCoeff(&maxY, &maxX);
    
    result[0] = static_cast<double>(maxY); // center_y
    result[1] = static_cast<double>(maxX); // center_x
    result[2] = psf.mean(); // background
    
    return result;
}

template<class derivedT>
double dmWavefrontControl<derivedT>::computeAiryMetric(const eigenImage<float>& measured, const eigenImage<float>& model, double penalty)
{
    double sum = 0.0;
    double modelSum = 0.0;
    
    for(int y = 0; y < measured.rows(); ++y) {
        for(int x = 0; x < measured.cols(); ++x) {
            double diff = measured(y, x) - model(y, x);
            sum += diff * diff;
            modelSum += model(y, x) * model(y, x);
        }
    }
    
    return sqrt(sum) + penalty / sqrt(modelSum);
}

// Optimization algorithms
template<class derivedT>
double dmWavefrontControl<derivedT>::moveMeasureMetric(double modeValue, int modeIndex, int nImages, const std::string& metricType, 
                                                      const std::map<std::string, double>& metricParams)
{
    // Apply mode to DM
    if(applyModeAndWait(modeIndex, modeValue) != 0) {
        derived().template log<software_error>({__FILE__, __LINE__, "Failed to apply mode " + std::to_string(modeIndex)});
        return 1e6; // Return large value on failure
    }
    
    // Wait for DM motion to settle
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Collect images
    auto images = collectImages(nImages);
    if(images.empty()) {
        derived().template log<software_error>({__FILE__, __LINE__, "Failed to collect images"});
        return 1e6;
    }
    
    // Compute metric
    return computeMetric(images, metricType, metricParams);
}

template<class derivedT>
double dmWavefrontControl<derivedT>::optimizeModeBrent(int modeIndex, std::pair<double, double> bounds, int nImages, 
                                                      const std::string& metricType, const std::map<std::string, double>& metricParams, 
                                                      double tolerance)
{
    // Simplified Brent optimization - grid search for now
    // TODO: Implement proper Brent optimization
    return gridSweepOptimization(modeIndex, bounds, 20, 3, nImages, metricType, metricParams, "fit");
}

template<class derivedT>
double dmWavefrontControl<derivedT>::gridSweepOptimization(int modeIndex, std::pair<double, double> bounds, int nSteps, int nRepeats, int nImages,
                                                          const std::string& metricType, const std::map<std::string, double>& metricParams,
                                                          const std::string& fitMethod)
{
    std::vector<double> steps;
    for(int i = 0; i < nSteps; ++i) {
        double t = static_cast<double>(i) / (nSteps - 1);
        steps.push_back(bounds.first + t * (bounds.second - bounds.first));
    }
    
    std::vector<std::vector<double>> curves(nRepeats, std::vector<double>(nSteps));
    
    // Collect measurements
    for(int repeat = 0; repeat < nRepeats; ++repeat) {
        for(int step = 0; step < nSteps; ++step) {
            // Apply mode multiple times for stability
            for(int l = 0; l < 3; ++l) {
                applyModeAndWait(modeIndex, steps[step]);
            }
            
            // Collect images
            auto images = collectImages(nImages);
            if(!images.empty()) {
                curves[repeat][step] = computeMetric(images, metricType, metricParams);
            } else {
                curves[repeat][step] = 1e6; // Large value on failure
            }
        }
    }
    
    if(fitMethod == "mean") {
        // Find mean minimum
        double minVal = 1e6;
        int minStep = 0;
        for(int step = 0; step < nSteps; ++step) {
            double avgVal = 0;
            for(int repeat = 0; repeat < nRepeats; ++repeat) {
                avgVal += curves[repeat][step];
            }
            avgVal /= nRepeats;
            if(avgVal < minVal) {
                minVal = avgVal;
                minStep = step;
            }
        }
        return steps[minStep];
    } else {
        // Fit quadratic
        std::vector<double> allSteps, allCurves;
        for(int repeat = 0; repeat < nRepeats; ++repeat) {
            for(int step = 0; step < nSteps; ++step) {
                allSteps.push_back(steps[step]);
                allCurves.push_back(curves[repeat][step]);
            }
        }
        
        // Simple quadratic fit: y = ax^2 + bx + c
        // TODO: Implement proper least squares fit
        // For now, return the step with minimum value
        double minVal = 1e6;
        int minStep = 0;
        for(int step = 0; step < nSteps; ++step) {
            double avgVal = 0;
            for(int repeat = 0; repeat < nRepeats; ++repeat) {
                avgVal += curves[repeat][step];
            }
            avgVal /= nRepeats;
            if(avgVal < minVal) {
                minVal = avgVal;
                minStep = step;
            }
        }
        return steps[minStep];
    }
}

template<class derivedT>
double dmWavefrontControl<derivedT>::computeMetric(const std::vector<eigenImage<float>>& images, const std::string& metricType, 
                                                  const std::map<std::string, double>& metricParams)
{
    if(images.empty()) return 1e6;
    
    if(metricType == "coreSum") {
        double radius = metricParams.count("radius") ? metricParams.at("radius") : 10.0;
        std::pair<double, double> center = {-1, -1};
        if(metricParams.count("ceny") && metricParams.count("cenx")) {
            center = {metricParams.at("ceny"), metricParams.at("cenx")};
        }
        
        double totalMetric = 0;
        for(const auto& img : images) {
            totalMetric += computeCoreSum(img, radius, center);
        }
        return totalMetric / images.size();
        
    } else if(metricType == "coreRingRatio") {
        double radius1 = metricParams.count("radius1") ? metricParams.at("radius1") : 10.0;
        double radius2 = metricParams.count("radius2") ? metricParams.at("radius2") : 20.0;
        
        double totalMetric = 0;
        for(const auto& img : images) {
            totalMetric += computeCoreRingRatio(img, radius1, radius2);
        }
        return totalMetric / images.size();
        
    } else if(metricType == "peak") {
        return computeImagePeak(images);
        
    } else if(metricType == "rss") {
        return computeRSS(images);
        
    } else {
        derived().template log<software_error>({__FILE__, __LINE__, "Unknown metric type: " + metricType});
        return 1e6;
    }
}

// Helper methods
template<class derivedT>
int dmWavefrontControl<derivedT>::applyModeAndWait(int modeIndex, double amplitude)
{
    // Apply modal mode using default modeset
    if(applyModalMode(m_defaultModeSet, modeIndex, amplitude) != 0) {
        return -1;
    }
    
    // Wait for DM to settle
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    return 0;
}

template<class derivedT>
std::vector<eigenImage<float>> dmWavefrontControl<derivedT>::collectImages(int nImages)
{
    std::vector<eigenImage<float>> images;
    
    // TODO: Implement actual image collection from camera
    // For now, create dummy images
    for(int i = 0; i < nImages; ++i) {
        eigenImage<float> img(100, 100);
        img.setRandom();
        images.push_back(img);
    }
    
    return images;
}

template<class derivedT>
std::pair<double, double> dmWavefrontControl<derivedT>::findCentroid(const eigenImage<float>& image, const eigenImage<bool>& mask)
{
    if(mask.size() == 0) {
        // No mask, use full image
        double sumY = 0, sumX = 0, sum = 0;
        for(int y = 0; y < image.rows(); ++y) {
            for(int x = 0; x < image.cols(); ++x) {
                sumY += y * image(y, x);
                sumX += x * image(y, x);
                sum += image(y, x);
            }
        }
        return {sumY / sum, sumX / sum};
    } else {
        // Use mask
        double sumY = 0, sumX = 0, sum = 0;
        for(int y = 0; y < image.rows(); ++y) {
            for(int x = 0; x < image.cols(); ++x) {
                if(mask(y, x)) {
                    sumY += y * image(y, x);
                    sumX += x * image(y, x);
                    sum += image(y, x);
                }
            }
        }
        return {sumY / sum, sumX / sum};
    }
}

template<class derivedT>
eigenImage<bool> dmWavefrontControl<derivedT>::createCircularMask(std::pair<double, double> center, double radius, std::pair<int, int> size)
{
    eigenImage<bool> mask(size.first, size.second);
    mask.setConstant(false);
    
    double y0 = center.first;
    double x0 = center.second;
    double r2 = radius * radius;
    
    for(int y = 0; y < size.first; ++y) {
        for(int x = 0; x < size.second; ++x) {
            double dy = static_cast<double>(y) - y0;
            double dx = static_cast<double>(x) - x0;
            if(dy*dy + dx*dx <= r2) {
                mask(y, x) = true;
            }
        }
    }
    
    return mask;
}

} // namespace dev
} // namespace app
} // namespace MagAOX

#endif // dmWavefrontControl_hpp
