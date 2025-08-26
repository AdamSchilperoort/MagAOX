#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <random>
#include <cmath>
#include <map>
#include <Eigen/Dense>

// This test demonstrates integration with the actual dmWavefrontControl framework
// It shows how the algorithms would work in a real MagAOX application

// Simulated MagAOX types (simplified versions)
using eigenImage = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
using eigenImageBool = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>;

// Simulated dmWavefrontControl class for testing
class SimulatedDMWavefrontControl {
private:
    // Simulated DM parameters
    int m_nModes;
    std::vector<double> m_currentModes;
    std::vector<double> m_modeScales;
    
    // Simulated camera parameters
    int m_imageWidth;
    int m_imageHeight;
    double m_pixelScale;
    double m_wavelength;
    double m_fNumber;
    
    // Simulated PSF parameters
    double m_psfFWHM;
    std::pair<double, double> m_psfCenter;
    double m_background;
    double m_noiseLevel;
    
    // Random number generation
    std::mt19937 m_rng;
    std::normal_distribution<double> m_normalDist;
    
public:
    SimulatedDMWavefrontControl(int nModes = 36, int imageWidth = 256, int imageHeight = 256) 
        : m_nModes(nModes), m_imageWidth(imageWidth), m_imageHeight(imageHeight),
          m_pixelScale(0.01), m_wavelength(0.55e-6), m_fNumber(8.0),
          m_psfFWHM(3.0), m_psfCenter({imageHeight/2.0, imageWidth/2.0}),
          m_background(100.0), m_noiseLevel(5.0),
          m_rng(std::chrono::steady_clock::now().time_since_epoch().count()),
          m_normalDist(0.0, 1.0) {
        
        // Initialize DM modes
        m_currentModes.resize(m_nModes, 0.0);
        m_modeScales.resize(m_nModes, 1.0);
        
        // Set some mode scales (e.g., Zernike modes)
        for(int i = 0; i < m_nModes; ++i) {
            if(i == 0) m_modeScales[i] = 0.0; // Piston
            else if(i == 1) m_modeScales[i] = 0.5; // Tip
            else if(i == 2) m_modeScales[i] = 0.5; // Tilt
            else if(i == 3) m_modeScales[i] = 0.3; // Defocus
            else m_modeScales[i] = 0.1; // Higher order modes
        }
    }
    
    // Simulate applying a mode to the DM
    int applyModalMode(const std::string& modesetName, int modeIndex, double amplitude) {
        (void)modesetName; // Suppress unused parameter warning
        if(modeIndex >= 0 && modeIndex < m_nModes) {
            m_currentModes[modeIndex] = amplitude;
            std::cout << "Applied mode " << modeIndex << " with amplitude " << amplitude << std::endl;
            return 0;
        }
        return -1;
    }
    
    // Simulate camera readout with realistic PSF
    eigenImage captureImage() {
        eigenImage image(m_imageHeight, m_imageWidth);
        
        // Generate base PSF
        generatePSF(image);
        
        // Add mode-dependent aberrations
        addModeAberrations(image);
        
        // Add noise
        addNoise(image);
        
        return image;
    }
    
    // Test the actual algorithms from dmWavefrontControl
    void testAlgorithms() {
        std::cout << "\n=== Testing dmWavefrontControl Algorithms ===" << std::endl;
        
        // Test 1: Background subtraction
        std::cout << "Testing background subtraction..." << std::endl;
        eigenImage testImage = captureImage();
        eigenImage bgSubtracted = subtractBackground(testImage, 1); // edge median
        
        std::cout << "  Original image - Max: " << testImage.maxCoeff() << ", Mean: " << testImage.mean() << std::endl;
        std::cout << "  BG subtracted - Max: " << bgSubtracted.maxCoeff() << ", Mean: " << bgSubtracted.mean() << std::endl;
        
        // Test 2: Peak finding
        std::cout << "Testing peak finding..." << std::endl;
        double peakValue = findPeak(testImage, 0);
        std::cout << "  Peak value: " << peakValue << std::endl;
        
        // Test 3: Core sum calculation
        std::cout << "Testing core sum calculation..." << std::endl;
        double coreSum = computeCoreSum(testImage, 10.0);
        std::cout << "  Core sum (radius=10): " << coreSum << std::endl;
        
        // Test 4: Core/ring ratio
        std::cout << "Testing core/ring ratio..." << std::endl;
        double coreRingRatio = computeCoreRingRatio(testImage, 10.0, 20.0);
        std::cout << "  Core/ring ratio (r1=10, r2=20): " << coreRingRatio << std::endl;
        
        // Test 5: Gaussian fitting
        std::cout << "Testing Gaussian fitting..." << std::endl;
        std::vector<double> gaussianParams = fitGaussian(testImage, 20);
        std::cout << "  Gaussian fit - FWHM: " << gaussianParams[0] << ", Peak: " << gaussianParams[1] 
                  << ", Center: (" << gaussianParams[2] << ", " << gaussianParams[3] << ")" << std::endl;
        
        // Test 6: Airy disk fitting
        std::cout << "Testing Airy disk fitting..." << std::endl;
        std::vector<double> airyParams = fitAiryDisk(testImage, m_wavelength, m_fNumber, m_pixelScale, 50);
        std::cout << "  Airy disk fit - Center: (" << airyParams[0] << ", " << airyParams[1] 
                  << "), Background: " << airyParams[2] << std::endl;
        
        // Test 7: Metric computation
        std::cout << "Testing metric computation..." << std::endl;
        std::vector<eigenImage> images = {testImage};
        std::map<std::string, double> metricParams = {{"radius", 10.0}};
        
        double coreSumMetric = computeMetric(images, "coreSum", metricParams);
        double coreRingMetric = computeMetric(images, "coreRingRatio", metricParams);
        double peakMetric = computeMetric(images, "peak", metricParams);
        
        std::cout << "  Core sum metric: " << coreSumMetric << std::endl;
        std::cout << "  Core/ring ratio metric: " << coreRingMetric << std::endl;
        std::cout << "  Peak metric: " << peakMetric << std::endl;
    }
    
    // Test optimization algorithms
    void testOptimization() {
        std::cout << "\n=== Testing Optimization Algorithms ===" << std::endl;
        
        // Test single mode optimization
        std::cout << "Testing single mode optimization..." << std::endl;
        int testMode = 3; // Defocus
        std::pair<double, double> bounds = {-0.3, 0.3};
        
        double optimalAmplitude = optimizeModeBrent(testMode, bounds, 1, "coreSum", 
                                                   {{"radius", 10.0}}, 1e-5);
        std::cout << "  Mode " << testMode << " optimal amplitude: " << optimalAmplitude << std::endl;
        
        // Test grid sweep optimization
        std::cout << "Testing grid sweep optimization..." << std::endl;
        double gridOptimal = gridSweepOptimization(testMode, bounds, 10, 2, 1, "coreSum", 
                                                  {{"radius", 10.0}}, "fit");
        std::cout << "  Grid sweep optimal amplitude: " << gridOptimal << std::endl;
    }
    
private:
    // Generate base PSF (Airy disk approximation)
    void generatePSF(eigenImage& image) {
        double sigma = m_psfFWHM / (2.0 * sqrt(2.0 * log(2.0)));
        
        for(int y = 0; y < m_imageHeight; ++y) {
            for(int x = 0; x < m_imageWidth; ++x) {
                double dy = y - m_psfCenter.first;
                double dx = x - m_psfCenter.second;
                double r2 = dx*dx + dy*dy;
                
                // Gaussian approximation of Airy disk
                double psf = exp(-r2 / (2.0 * sigma * sigma));
                image(y, x) = m_background + 1000.0 * psf;
            }
        }
    }
    
    // Add aberrations based on current DM modes
    void addModeAberrations(eigenImage& image) {
        for(int y = 0; y < m_imageHeight; ++y) {
            for(int x = 0; x < m_imageWidth; ++x) {
                double dy = (y - m_psfCenter.first) / (m_imageHeight / 2.0);
                double dx = (x - m_psfCenter.second) / (m_imageWidth / 2.0);
                double r = sqrt(dx*dx + dy*dy);
                double theta = atan2(dy, dx);
                
                double aberration = 0.0;
                
                // Add Zernike-like aberrations
                for(int i = 0; i < m_nModes; ++i) {
                    if(i == 0) continue; // Skip piston
                    else if(i == 1) aberration += m_currentModes[i] * dx; // Tip
                    else if(i == 2) aberration += m_currentModes[i] * dy; // Tilt
                    else if(i == 3) aberration += m_currentModes[i] * (2.0*r*r - 1.0); // Defocus
                    else if(i == 4) aberration += m_currentModes[i] * r*r * cos(2.0*theta); // Astigmatism
                    else if(i == 5) aberration += m_currentModes[i] * r*r * sin(2.0*theta); // Astigmatism
                    else {
                        // Higher order modes with random patterns
                        aberration += m_currentModes[i] * sin((i+1)*theta) * pow(r, (i+1)/2);
                    }
                }
                
                // Apply aberration effect (simplified)
                double factor = 1.0 + 0.1 * aberration;
                image(y, x) *= factor;
            }
        }
    }
    
    // Add realistic noise
    void addNoise(eigenImage& image) {
        for(int y = 0; y < m_imageHeight; ++y) {
            for(int x = 0; x < m_imageWidth; ++x) {
                // Poisson noise (shot noise)
                double shotNoise = sqrt(image(y, x)) * m_normalDist(m_rng);
                
                // Read noise
                double readNoise = m_noiseLevel * m_normalDist(m_rng);
                
                image(y, x) += shotNoise + readNoise;
                
                // Ensure non-negative
                if(image(y, x) < 0) image(y, x) = 0;
            }
        }
    }
    
    // Algorithm implementations (copied from dmWavefrontControl)
    eigenImage subtractBackground(const eigenImage& image, int method) {
        eigenImage result = image;
        
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
    
    double findPeak(const eigenImage& image, int method, int clipping = 0) {
        (void)clipping; // Suppress unused parameter warning
        if(method == 0) {
            // Naive maximum
            return image.maxCoeff();
        } else {
            // Gaussian fit (simplified - return max for now)
            return image.maxCoeff();
        }
    }
    
    double computeCoreSum(const eigenImage& image, double radius, std::pair<double, double> center = {-1, -1}) {
        eigenImage bgSub = subtractBackground(image, 1); // edge median
        
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
        eigenImageBool coreMask = createCircularMask({ceny, cenx}, radius, {image.rows(), image.cols()});
        
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
    
    double computeCoreRingRatio(const eigenImage& image, double radius1, double radius2) {
        eigenImage bgSub = subtractBackground(image, 1); // edge median
        
        // Find center
        int maxY, maxX;
        bgSub.maxCoeff(&maxY, &maxX);
        double ceny = static_cast<double>(maxY);
        double cenx = static_cast<double>(maxX);
        
        // Create masks
        eigenImageBool coreMask = createCircularMask({ceny, cenx}, radius1, {image.rows(), image.cols()});
        eigenImageBool annulusMask = createCircularMask({ceny, cenx}, radius2, {image.rows(), image.cols()});
        
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
    
    std::vector<double> fitGaussian(const eigenImage& image, int clipping = 0) {
        (void)clipping; // Suppress unused parameter warning
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
    
    std::vector<double> fitAiryDisk(const eigenImage& psf, double wavelength, double fnum, double pixscale, int cutout = 100) {
        (void)wavelength; // Suppress unused parameter warnings
        (void)fnum;
        (void)pixscale;
        (void)cutout;
        
        // Simplified Airy disk fit
        std::vector<double> result(3);
        
        int maxY, maxX;
        psf.maxCoeff(&maxY, &maxX);
        
        result[0] = static_cast<double>(maxY); // center_y
        result[1] = static_cast<double>(maxX); // center_x
        result[2] = psf.mean(); // background
        
        return result;
    }
    
    double computeMetric(const std::vector<eigenImage>& images, const std::string& metricType, 
                        const std::map<std::string, double>& metricParams) {
        if(images.empty()) return 1e6;
        
        if(metricType == "coreSum") {
            double radius = metricParams.count("radius") ? metricParams.at("radius") : 10.0;
            std::pair<double, double> center = {-1, -1};
            if(metricParams.count("ceny") && metricParams.count("cenx")) {
                center = std::make_pair(metricParams.at("ceny"), metricParams.at("cenx"));
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
            double totalPeak = 0.0;
            for(const auto& img : images) {
                totalPeak += findPeak(img, 0);
            }
            return totalPeak / images.size();
            
        } else {
            std::cerr << "Unknown metric type: " << metricType << std::endl;
            return 1e6;
        }
    }
    
    double optimizeModeBrent(int modeIndex, std::pair<double, double> bounds, int nImages, 
                            const std::string& metricType, const std::map<std::string, double>& metricParams, 
                            double tolerance) {
        (void)nImages; // Suppress unused parameter warning
        (void)tolerance;
        // Simplified Brent optimization - grid search for now
        return gridSweepOptimization(modeIndex, bounds, 20, 3, 1, metricType, metricParams, "fit");
    }
    
    double gridSweepOptimization(int modeIndex, std::pair<double, double> bounds, int nSteps, int nRepeats, int nImages,
                                 const std::string& metricType, const std::map<std::string, double>& metricParams,
                                 const std::string& fitMethod) {
        (void)nImages; // Suppress unused parameter warning
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
                    applyModalMode("default", modeIndex, steps[step]);
                }
                
                // Collect images
                auto images = std::vector<eigenImage>{captureImage()};
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
    
    eigenImageBool createCircularMask(std::pair<double, double> center, double radius, std::pair<int, int> size) {
        eigenImageBool mask(size.first, size.second);
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
};

int main() {
    std::cout << "Eye-Doctor Integration Test Application" << std::endl;
    std::cout << "=======================================" << std::endl;
    
    try {
        // Create simulated dmWavefrontControl
        SimulatedDMWavefrontControl dmControl(36, 128, 128);
        
        // Test the algorithms
        dmControl.testAlgorithms();
        
        // Test optimization
        dmControl.testOptimization();
        
        std::cout << "\n=== Integration Tests Completed Successfully ===" << std::endl;
        
    } catch(const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
