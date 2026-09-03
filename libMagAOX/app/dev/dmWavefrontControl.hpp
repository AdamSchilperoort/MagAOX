/** \file dmWavefrontControl.hpp
  * \brief Utility class for DM wavefront sensing and control algorithms
  *
  * \ingroup dmWavefrontControl_files
  */

#ifndef dmWavefrontControl_hpp
#define dmWavefrontControl_hpp

#include <mx/improc/eigenImage.hpp>
#include <mx/improc/eigenCube.hpp>
using namespace mx::improc;

#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <map>
#include <algorithm>
#include <utility>

// Include CFITSIO for FITS file access
#include <fitsio.h>

/** \defgroup dmWavefrontControl
  * \brief Utility class providing DM wavefront sensing and control algorithms
  *
  * This is a pure utility class with no INDI integration or MagAOXApp dependencies.
  * It can be used by any MagAOX app that needs wavefront sensing algorithms.
  *
  * \ingroup appdev
  */

namespace MagAOX
{
namespace app
{
namespace dev
{

/// Utility class for DM wavefront sensing and control algorithms
/** This class provides algorithms and data structures for deformable mirror
  * wavefront control, including:
  * - Data structures for DM metadata and modesets
  * - Image processing (background subtraction, peak finding, etc.)
  * - PSF metrics (core sum, core/ring ratio, etc.)
  * - FITS modeset loading
  * 
  * This is a pure utility class with no INDI or MagAOXApp dependencies.
  * Applications should create an instance and call methods as needed.
  * 
  * \ingroup appdev
  */
class dmWavefrontControl 
{

public:

    /** \name Public Data Structures
      *@{
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
        eigenImage<bool> actuatorMask;      ///< Valid actuator mask
        eigenImage<double> actuatorGains;   ///< Per-actuator gains
    };

    // Modeset information
    struct ModeSet {
        std::string name;                    ///< e.g., "zernike", "hadamard"
        std::string filename;                ///< e.g., "zernike_modes.fits"
        eigenCube<float> modes;              ///< 3D cube: [height][width][nmodes]
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
    
    ///@}

public:

    /// Default constructor
    dmWavefrontControl();

    /// Destructor
    ~dmWavefrontControl();

    /** \name Modeset and DM Metadata Management
      * @{
      */
    
    /// Initialize DM metadata from parameters
    /** Call this after setting up m_dmInfo parameters manually
      * 
      * \returns 0 on success
      * \returns < 0 on error
      */
    int initializeDMMetadata();
    
    /// Load a single modeset from FITS file
    /** Loads a FITS file containing mode shapes
      * 
      * \param filename [in] Path to FITS file
      * \param name [in] Name for this modeset
      * 
      * \returns 0 on success
      * \returns < 0 on error (error code indicates failure type)
      */
    int loadModeSet(const std::string& filename, const std::string& name);
    
    /// Convert modal commands to actuator commands
    /** Given a list of modal commands in format "modeset:mode_index", amplitude,
      * compute the actuator commands by combining the modal basis functions
      * 
      * \param modalCommands [in] Vector of (mode_name, amplitude) pairs
      * 
      * \returns Vector of actuator commands
      */
    std::vector<double> modalToActuator(const std::vector<std::pair<std::string, double>>& modalCommands);
    
    ///@}

    /** \name Image Processing Utilities
      * @{
      */
    
    /**
     * @brief Subtract background from image using various methods
     * @param image Input image
     * @param method Background subtraction method (0=full median, 1=edge median, 2=mode, 3=row/column)
     * @return Background-subtracted image
     */
    static eigenImage<float> subtractBackground(const eigenImage<float>& image, int method = 0);
    
    /**
     * @brief Find peak in image
     * @param image Input image
     * @return Peak value
     */
    static double findPeak(const eigenImage<float>& image);
    
    /**
     * @brief Compute core sum metric (sum within radius around peak)
     * @param image Input image
     * @param radius Core radius in pixels
     * @param center Optional center coordinates (y, x). If negative, finds peak automatically
     * @return Core sum value
     */
    static double computeCoreSum(const eigenImage<float>& image, double radius, 
                                  std::pair<double, double> center = {-1, -1});
    
    /**
     * @brief Compute core/ring ratio metric
     * @param image Input image
     * @param radius1 Core radius in pixels
     * @param radius2 Annulus radius in pixels
     * @return Core/ring ratio
     */
    static double computeCoreRingRatio(const eigenImage<float>& image, double radius1, double radius2);
    
    /**
     * @brief Find centroid using center of mass
     * @param image Input image
     * @param mask Optional mask for centroid calculation
     * @return Centroid coordinates (y, x)
     */
    static std::pair<double, double> findCentroid(const eigenImage<float>& image, 
                                                   const eigenImage<bool>& mask = eigenImage<bool>());
    
    /**
     * @brief Create circular mask
     * @param center Center coordinates (y, x)
     * @param radius Radius in pixels
     * @param size Image size (height, width)
     * @return Binary mask
     */
    static eigenImage<bool> createCircularMask(std::pair<double, double> center, double radius, 
                                               std::pair<int, int> size);
    
    /**
     * @brief Compute metric from images
     * @param images Vector of images
     * @param metricType Type of metric to compute ("coreSum", "coreRingRatio", "peak", "rss")
     * @param metricParams Metric parameters (e.g., {"radius": 5.0})
     * @return Metric value
     */
    static double computeMetric(const std::vector<eigenImage<float>>& images, 
                                const std::string& metricType, 
                                const std::map<std::string, double>& metricParams);
    
    /**
     * @brief Compute root sum square of images
     * @param images Vector of images
     * @return RSS value
     */
    static double computeRSS(const std::vector<eigenImage<float>>& images);
    
    ///@}

}; // class dmWavefrontControl

// Implementation

inline dmWavefrontControl::dmWavefrontControl()
{
    // Initialize DM info with defaults
    m_dmInfo.name = "default";
    m_dmInfo.numActuators = 0;
    m_dmInfo.width = 0;
    m_dmInfo.height = 0;
    m_dmInfo.actuatorSpacing = 0.0;
    m_dmInfo.maxStroke = 0.0;
}

inline dmWavefrontControl::~dmWavefrontControl()
{
    // Default destructors handle cleanup
}

inline int dmWavefrontControl::initializeDMMetadata()
{
    if (m_dmInfo.numActuators == 0 || m_dmInfo.width == 0 || m_dmInfo.height == 0)
    {
        return -1; // Error: DM info not configured
    }
    
    // Initialize actuator arrays
    m_dmInfo.actuators.resize(m_dmInfo.numActuators);
    m_dmInfo.actuatorMask.resize(m_dmInfo.height, m_dmInfo.width);
    m_dmInfo.actuatorGains.resize(m_dmInfo.height, m_dmInfo.width);
    
    // Set default values
    m_dmInfo.actuatorMask.setConstant(true); // All actuators valid by default
    m_dmInfo.actuatorGains.setConstant(1.0); // Unity gain by default
    
    // Initialize actuators with default grid layout
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
    
    return 0;
}

inline int dmWavefrontControl::loadModeSet(const std::string& filename, const std::string& name)
{
    try {
        ModeSet modeset;
        modeset.name = name;
        modeset.filename = filename;
        
        // Open FITS file
        fitsfile *fptr;
        int status = 0;
        
        if (fits_open_image(&fptr, filename.c_str(), READONLY, &status)) {
            return -1; // Could not open file
        }
        
        int hdutype, naxis;
        long naxes[3];
        
        if (fits_get_hdu_type(fptr, &hdutype, &status) || hdutype != IMAGE_HDU) {
            fits_close_file(fptr, &status);
            return -2; // Not an image HDU
        }
        
        fits_get_img_dim(fptr, &naxis, &status);
        fits_get_img_size(fptr, 3, naxes, &status);
        
        if (status != 0 || naxis != 3) {
            fits_close_file(fptr, &status);
            return -3; // Not a 3D cube
        }
        
        // FITS stores as [width, height, planes] in FORTRAN order
        // NAXIS1 = width, NAXIS2 = height, NAXIS3 = number of modes/planes
        int width = naxes[0];   // NAXIS1
        int height = naxes[1];  // NAXIS2
        int numModes = naxes[2]; // NAXIS3
        
        // eigenCube::resize takes (rows, cols, planes)
        modeset.modes.resize(height, width, numModes);
        
        // Read the data plane by plane
        long fpixel[3];
        std::vector<float> buffer(width * height);
        
        for (int p = 0; p < numModes; ++p) {
            // Set starting pixel: x=1, y=1, z=plane+1 (1-indexed)
            fpixel[0] = 1;
            fpixel[1] = 1;
            fpixel[2] = p + 1;
            
            if (fits_read_pix(fptr, TFLOAT, fpixel, width * height, 0, buffer.data(), 0, &status) == 0) {
                // Copy to eigen matrix
                // fits_read_pix reads data in FORTRAN order: x varies fastest
                for (int y = 0; y < height; ++y) {
                    for (int x = 0; x < width; ++x) {
                        int bufferIndex = y * width + x;
                        modeset.modes.image(p)(y, x) = buffer[bufferIndex];
                    }
                }
            } else {
                fits_close_file(fptr, &status);
                return -4; // Failed to read pixels
            }
        }
        
        // Initialize mode metadata
        modeset.modeNames.resize(numModes);
        modeset.modeScales.resize(numModes, 1.0);
        modeset.modalGains.resize(numModes, 1.0);
        modeset.modeMin.resize(numModes, -10.0);
        modeset.modeMax.resize(numModes, 10.0);
        
        // Try to read mode names from FITS header
        for (int p = 0; p < numModes; ++p) {
            std::string keyword = "MODE" + std::to_string(p + 1);
            char value[FLEN_VALUE];
            int tmpStatus = 0;
            if (fits_read_key(fptr, TSTRING, keyword.c_str(), value, NULL, &tmpStatus) == 0) {
                modeset.modeNames[p] = value;
            } else {
                modeset.modeNames[p] = "Mode" + std::to_string(p + 1);
            }
        }
        
        fits_close_file(fptr, &status);
        
        // Add to modesets list
        m_modeSets.push_back(modeset);
        m_modeSetMap[name] = m_modeSets.size() - 1;
        
        return 0; // Success
        
    } catch (...) {
        return -5; // Unknown error
    }
}

inline std::vector<double> dmWavefrontControl::modalToActuator(
    const std::vector<std::pair<std::string, double>>& modalCommands)
{
    std::vector<double> actuatorCommands(m_dmInfo.numActuators, 0.0);
    
    try {
        for (const auto& command : modalCommands) {
            const std::string& modesetName = command.first;
            double amplitude = command.second;
            
            // Parse modeset:mode format
            size_t colonPos = modesetName.find(':');
            if (colonPos == std::string::npos) {
                continue; // Skip invalid format
            }
            
            std::string modeset = modesetName.substr(0, colonPos);
            int modeIndex = std::stoi(modesetName.substr(colonPos + 1)) - 1; // Convert to 0-based
            
            // Find the modeset
            auto it = m_modeSetMap.find(modeset);
            if (it == m_modeSetMap.end()) {
                continue; // Modeset not found
            }
            
            const ModeSet& modesetData = m_modeSets[it->second];
            
            // Validate mode index
            if (modeIndex < 0 || modeIndex >= modesetData.modes.planes()) {
                continue; // Invalid mode index
            }
            
            // Apply limits and scaling
            double limitedAmplitude = std::max(modesetData.modeMin[modeIndex],
                                             std::min(modesetData.modeMax[modeIndex], amplitude));
            double scaledAmplitude = limitedAmplitude * modesetData.modeScales[modeIndex] * 
                                    modesetData.modalGains[modeIndex];
            
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
    } catch (...) {
        // Return partial result on error
    }
    
    return actuatorCommands;
}

// Static image processing implementations

inline eigenImage<float> dmWavefrontControl::subtractBackground(const eigenImage<float>& image, int method)
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
            for(int y = 0; y < edgeSize && y < image.rows(); ++y) {
                for(int x = 0; x < image.cols(); ++x) {
                    edgeValues.push_back(image(y, x));
                    if (image.rows()-1-y >= 0)
                        edgeValues.push_back(image(image.rows()-1-y, x));
                }
            }
            for(int y = edgeSize; y < image.rows()-edgeSize; ++y) {
                for(int x = 0; x < edgeSize && x < image.cols(); ++x) {
                    edgeValues.push_back(image(y, x));
                    if (image.cols()-1-x >= 0)
                        edgeValues.push_back(image(y, image.cols()-1-x));
                }
            }
            
            // Calculate median
            if (!edgeValues.empty()) {
                std::sort(edgeValues.begin(), edgeValues.end());
                float median = edgeValues[edgeValues.size()/2];
                result.array() -= median;
            }
            break;
        }
        
        default:
            // Default to mean subtraction
            result.array() -= image.mean();
            break;
    }
    
    return result;
}

inline double dmWavefrontControl::findPeak(const eigenImage<float>& image)
{
    return image.maxCoeff();
}

inline double dmWavefrontControl::computeCoreSum(const eigenImage<float>& image, double radius, 
                                                 std::pair<double, double> center)
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

inline double dmWavefrontControl::computeCoreRingRatio(const eigenImage<float>& image, 
                                                       double radius1, double radius2)
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

inline std::pair<double, double> dmWavefrontControl::findCentroid(const eigenImage<float>& image, 
                                                                   const eigenImage<bool>& mask)
{
    if(mask.size() == 0) {
        // No mask, use full image
        double sumY = 0, sumX = 0, sum = 0;
        for(int y = 0; y < image.rows(); ++y) {
            for(int x = 0; x < image.cols(); ++x) {
                double val = image(y, x);
                if (val > 0) {  // Only use positive values
                    sumY += y * val;
                    sumX += x * val;
                    sum += val;
                }
            }
        }
        return sum > 0 ? std::make_pair(sumY / sum, sumX / sum) : std::make_pair(0.0, 0.0);
    } else {
        // Use mask
        double sumY = 0, sumX = 0, sum = 0;
        for(int y = 0; y < image.rows(); ++y) {
            for(int x = 0; x < image.cols(); ++x) {
                if(mask(y, x)) {
                    double val = image(y, x);
                    if (val > 0) {
                        sumY += y * val;
                        sumX += x * val;
                        sum += val;
                    }
                }
            }
        }
        return sum > 0 ? std::make_pair(sumY / sum, sumX / sum) : std::make_pair(0.0, 0.0);
    }
}

inline eigenImage<bool> dmWavefrontControl::createCircularMask(std::pair<double, double> center, 
                                                               double radius, std::pair<int, int> size)
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

inline double dmWavefrontControl::computeMetric(const std::vector<eigenImage<float>>& images, 
                                                const std::string& metricType, 
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
        return -(totalMetric / images.size()); // Negative for minimization
        
    } else if(metricType == "coreRingRatio") {
        double radius1 = metricParams.count("radius1") ? metricParams.at("radius1") : 10.0;
        double radius2 = metricParams.count("radius2") ? metricParams.at("radius2") : 20.0;
        
        double totalMetric = 0;
        for(const auto& img : images) {
            totalMetric += computeCoreRingRatio(img, radius1, radius2);
        }
        return totalMetric / images.size();
        
    } else if(metricType == "peak") {
        double totalPeak = 0;
        for(const auto& img : images) {
            totalPeak += findPeak(img);
        }
        return -(totalPeak / images.size()); // Negative for minimization
        
    } else if(metricType == "rss") {
        return computeRSS(images);
        
    } else {
        return 1e6; // Unknown metric type
    }
}

inline double dmWavefrontControl::computeRSS(const std::vector<eigenImage<float>>& images)
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

} // namespace dev
} // namespace app
} // namespace MagAOX

#endif // dmWavefrontControl_hpp
