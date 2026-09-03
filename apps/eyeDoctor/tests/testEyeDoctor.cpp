#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <random>
#include <cmath>
#include <Eigen/Dense>

// Test framework for eye-doctor algorithms
class TestEyeDoctor {
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
    TestEyeDoctor(int nModes = 36, int imageWidth = 256, int imageHeight = 256) 
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
    void applyMode(int modeIndex, double amplitude) {
        if(modeIndex >= 0 && modeIndex < m_nModes) {
            m_currentModes[modeIndex] = amplitude;
            std::cout << "Applied mode " << modeIndex << " with amplitude " << amplitude << std::endl;
        }
    }
    
    // Simulate camera readout with realistic PSF
    Eigen::MatrixXd captureImage() {
        Eigen::MatrixXd image(m_imageHeight, m_imageWidth);
        
        // Generate base PSF
        generatePSF(image);
        
        // Add mode-dependent aberrations
        addModeAberrations(image);
        
        // Add noise
        addNoise(image);
        
        return image;
    }
    
    // Get current mode values
    std::vector<double> getCurrentModes() const {
        return m_currentModes;
    }
    
    // Reset all modes to zero
    void resetModes() {
        std::fill(m_currentModes.begin(), m_currentModes.end(), 0.0);
        std::cout << "Reset all DM modes to zero" << std::endl;
    }
    
    // Print current mode status
    void printModeStatus() const {
        std::cout << "Current DM modes:" << std::endl;
        for(int i = 0; i < std::min(10, m_nModes); ++i) {
            std::cout << "  Mode " << i << ": " << m_currentModes[i] << std::endl;
        }
        if(m_nModes > 10) {
            std::cout << "  ... and " << (m_nModes - 10) << " more modes" << std::endl;
        }
    }
    
private:
    // Generate base PSF (Airy disk approximation)
    void generatePSF(Eigen::MatrixXd& image) {
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
    void addModeAberrations(Eigen::MatrixXd& image) {
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
    void addNoise(Eigen::MatrixXd& image) {
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
};

// Test the image processing algorithms
void testImageProcessing() {
    std::cout << "\n=== Testing Image Processing Algorithms ===" << std::endl;
    
    TestEyeDoctor testApp(36, 128, 128);
    
    // Test 1: Capture baseline image
    std::cout << "Capturing baseline image..." << std::endl;
    Eigen::MatrixXd baselineImage = testApp.captureImage();
    
    // Test 2: Apply some aberrations
    std::cout << "Applying aberrations..." << std::endl;
    testApp.applyMode(1, 0.1);  // Tip
    testApp.applyMode(2, 0.1);  // Tilt
    testApp.applyMode(3, 0.2);  // Defocus
    
    Eigen::MatrixXd aberratedImage = testApp.captureImage();
    
    // Test 3: Analyze images
    std::cout << "Analyzing images..." << std::endl;
    
    // Basic statistics
    double baselineMax = baselineImage.maxCoeff();
    double aberratedMax = aberratedImage.maxCoeff();
    double baselineMean = baselineImage.mean();
    double aberratedMean = aberratedImage.mean();
    
    std::cout << "Baseline - Max: " << baselineMax << ", Mean: " << baselineMean << std::endl;
    std::cout << "Aberrated - Max: " << aberratedMax << ", Mean: " << aberratedMean << std::endl;
    
    // Peak finding
    int maxY, maxX;
    baselineImage.maxCoeff(&maxY, &maxX);
    std::cout << "Baseline peak at: (" << maxY << ", " << maxX << ")" << std::endl;
    
    aberratedImage.maxCoeff(&maxY, &maxX);
    std::cout << "Aberrated peak at: (" << maxY << ", " << maxX << ")" << std::endl;
    
    // Core sum calculation (simplified)
    double baselineCoreSum = 0, aberratedCoreSum = 0;
    int coreRadius = 5;
    
    for(int y = maxY - coreRadius; y <= maxY + coreRadius; ++y) {
        for(int x = maxX - coreRadius; x <= maxX + coreRadius; ++x) {
            if(y >= 0 && y < baselineImage.rows() && x >= 0 && x < baselineImage.cols()) {
                double r2 = (y - maxY)*(y - maxY) + (x - maxX)*(x - maxX);
                if(r2 <= coreRadius*coreRadius) {
                    baselineCoreSum += baselineImage(y, x);
                    aberratedCoreSum += aberratedImage(y, x);
                }
            }
        }
    }
    
    std::cout << "Core sum - Baseline: " << baselineCoreSum << ", Aberrated: " << aberratedCoreSum << std::endl;
    std::cout << "Core sum ratio: " << baselineCoreSum / aberratedCoreSum << std::endl;
}

// Test the optimization loop
void testOptimizationLoop() {
    std::cout << "\n=== Testing Optimization Loop ===" << std::endl;
    
    TestEyeDoctor testApp(36, 128, 128);
    
    // Test optimization of a single mode
    int testMode = 3; // Defocus
    std::vector<double> testAmplitudes = {-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3};
    std::vector<double> metrics;
    
    std::cout << "Testing mode " << testMode << " optimization..." << std::endl;
    
    for(double amp : testAmplitudes) {
        // Apply mode
        testApp.applyMode(testMode, amp);
        
        // Wait for settling
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // Capture image
        Eigen::MatrixXd image = testApp.captureImage();
        
        // Calculate metric (core sum - negative for minimization)
        int maxY, maxX;
        image.maxCoeff(&maxY, &maxX);
        
        double coreSum = 0;
        int coreRadius = 5;
        for(int y = maxY - coreRadius; y <= maxY + coreRadius; ++y) {
            for(int x = maxX - coreRadius; x <= maxX + coreRadius; ++x) {
                if(y >= 0 && y < image.rows() && x >= 0 && x < image.cols()) {
                    double r2 = (y - maxY)*(y - maxY) + (x - maxX)*(x - maxX);
                    if(r2 <= coreRadius*coreRadius) {
                        coreSum += image(y, x);
                    }
                }
            }
        }
        
        metrics.push_back(-coreSum); // Negative for minimization
        
        std::cout << "  Amplitude: " << amp << ", Metric: " << -coreSum << std::endl;
    }
    
    // Find optimal amplitude
    auto minIt = std::min_element(metrics.begin(), metrics.end());
    int minIndex = std::distance(metrics.begin(), minIt);
    double optimalAmplitude = testAmplitudes[minIndex];
    
    std::cout << "Optimal amplitude: " << optimalAmplitude << " (metric: " << *minIt << ")" << std::endl;
    
    // Apply optimal mode
    testApp.applyMode(testMode, optimalAmplitude);
    std::cout << "Applied optimal mode " << testMode << " with amplitude " << optimalAmplitude << std::endl;
}

// Test multiple mode optimization
void testMultiModeOptimization() {
    std::cout << "\n=== Testing Multi-Mode Optimization ===" << std::endl;
    
    TestEyeDoctor testApp(36, 128, 128);
    
    // Test optimization of multiple modes
    std::vector<int> modesToOptimize = {1, 2, 3, 4, 5}; // Tip, Tilt, Defocus, Astigmatism
    std::vector<double> optimalAmplitudes;
    
    for(int mode : modesToOptimize) {
        std::cout << "Optimizing mode " << mode << "..." << std::endl;
        
        // Simple grid search
        std::vector<double> testAmplitudes = {-0.2, -0.1, 0.0, 0.1, 0.2};
        std::vector<double> metrics;
        
        for(double amp : testAmplitudes) {
            // Apply mode
            testApp.applyMode(mode, amp);
            
            // Wait for settling
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            
            // Capture image
            Eigen::MatrixXd image = testApp.captureImage();
            
            // Calculate metric (core sum - negative for minimization)
            int maxY, maxX;
            image.maxCoeff(&maxY, &maxX);
            
            double coreSum = 0;
            int coreRadius = 5;
            for(int y = maxY - coreRadius; y <= maxY + coreRadius; ++y) {
                for(int x = maxX - coreRadius; x <= maxX + coreRadius; ++x) {
                    if(y >= 0 && y < image.rows() && x >= 0 && x < image.cols()) {
                        double r2 = (y - maxY)*(y - maxY) + (x - maxX)*(x - maxX);
                        if(r2 <= coreRadius*coreRadius) {
                            coreSum += image(y, x);
                        }
                    }
                }
            }
            
            metrics.push_back(-coreSum);
        }
        
        // Find optimal amplitude
        auto minIt = std::min_element(metrics.begin(), metrics.end());
        int minIndex = std::distance(metrics.begin(), minIt);
        double optimalAmplitude = testAmplitudes[minIndex];
        
        optimalAmplitudes.push_back(optimalAmplitude);
        
        std::cout << "  Mode " << mode << " optimal amplitude: " << optimalAmplitude << std::endl;
        
        // Apply optimal mode
        testApp.applyMode(mode, optimalAmplitude);
    }
    
    std::cout << "Multi-mode optimization complete!" << std::endl;
    std::cout << "Final mode values:" << std::endl;
    for(size_t i = 0; i < modesToOptimize.size(); ++i) {
        std::cout << "  Mode " << modesToOptimize[i] << ": " << optimalAmplitudes[i] << std::endl;
    }
}

int main() {
    std::cout << "Eye-Doctor Test Application" << std::endl;
    std::cout << "===========================" << std::endl;
    
    try {
        // Test 1: Image processing algorithms
        testImageProcessing();
        
        // Test 2: Single mode optimization
        testOptimizationLoop();
        
        // Test 3: Multi-mode optimization
        testMultiModeOptimization();
        
        std::cout << "\n=== All Tests Completed Successfully ===" << std::endl;
        
    } catch(const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}

