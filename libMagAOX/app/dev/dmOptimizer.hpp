/** \file dmOptimizer.hpp
  * \brief The MagAO-X DM Optimizer header file
  *
  * \ingroup dmOptimizer_files
  */

#ifndef dmOptimizer_hpp
#define dmOptimizer_hpp

#include <mx/improc/eigenImage.hpp>
#include <mx/improc/milkImage.hpp>
#include <mx/improc/eigenCube.hpp>
using namespace mx::improc;

#include <vector>
#include <string>
#include <functional>

/** \defgroup dmOptimizer
  * \brief The MagAO-X device to coordinate DM optimization algorithms
  *
  * <a href="../handbook/operating/software/apps/dmOptimizer.html">Application Documentation</a>
  *
  * \ingroup apps
  *
  */

/** \defgroup dmOptimizer_files
  * \ingroup dmOptimizer
  */

namespace MagAOX
{
namespace app
{
namespace dev
{

/// A base class to coordinate DM optimization algorithms
/** CRTP class `derivedT` has the following requirements:
  * 
  * - Must be derived from MagAOXApp<true>
  * 
  * - Must be derived from `dev::dmPokeWFS<DERIVEDNAME>` (replace DERIVEDNAME with derivedT class name)
  * 
  * - Must contain the following friend declarations (replace DERIVEDNAME with derivedT class name):
  *   \code
  *      friend class dev::dmPokeWFS<DERIVEDNAME>;
  *      friend class dev::dmOptimizer<DERIVEDNAME>;
  *   \endcode
  * 
  * - Must contain the following typedefs (replace DERIVEDNAME with derivedT class name):
  *   \code
  *       typedef dev::dmPokeWFS<DERIVEDNAME> dmPokeWFST;
  *       typedef dev::dmOptimizer<DERIVEDNAME> dmOptimizerT;
  *   \endcode
  * 
  * - Must provide the following interfaces:
  *   \code
  *       // Run the optimization algorithm
  *       // This coordinates the optimization process
  *       // 
  *       // returns 0 on success
  *       // returns < 0 on an error
  *       int runOptimization();
  *   \endcode
  * 
  * - Must provide the following interface:
  *   \code 
  *       // Analyze the optimization results
  *       // This analyzes the optimization results and updates metrics
  *       //
  *       // returns 0 on success
  *       // returns < 0 on an error
  *       int analyzeOptimization();
  *   \endcode
  * 
  * - Must call this base class's setupConfig(), loadConfig(), appStartup(), appStartup(), and appShutdown() in the 
  *    appropriate functions.  For convenience the following macros are defined to provide error checking:
  *    \code  
  *       DMOPTIMIZER_SETUP_CONFIG( cfig )
  *       DMOPTIMIZER_LOAD_CONFIG( cfig )
  *       DMOPTIMIZER_APP_STARTUP
  *       DMOPTIMIZER_APP_LOGIC
  *       DMOPTIMIZER_APP_SHUTDOWN
  *    \endcode
  * 
  * \ingroup appdev
  */
template<class derivedT>
class dmOptimizer 
{

public:

protected:

    /** \name Configurable Parameters
      *@{
      */
   
    std::string m_optimizationAlgorithm; ///< The optimization algorithm to use
    double m_convergenceThreshold; ///< Convergence threshold for optimization
    unsigned m_maxIterations; ///< Maximum number of optimization iterations
    double m_learningRate; ///< Learning rate for gradient-based optimization
    bool m_useAdaptiveStepSize; ///< Whether to use adaptive step size
    
    ///@}

    // Optimization state
    bool m_optimizationInProgress;
    unsigned m_currentIteration;
    double m_currentCost;
    double m_previousCost;
    std::vector<double> m_optimizationHistory;
    
    // Results storage
    std::vector<double> m_optimalParameters;
    std::vector<double> m_costHistory;
    std::vector<double> m_gradientHistory;

public:

    /**\name MagAOXApp Interface
      *
      * @{ 
      */

    /// Setup the configuration system
    /**
     * This should be called in `derivedT::setupConfig` as
     * \code
       dmOptimizer<derivedT>::setupConfig(config);
       \endcode
     * with appropriate error checking.
     */
    int setupConfig( mx::app::appConfigurator & config /**< [in] an application configuration to load values to*/);

    /// load the configuration system results
    /**
      * This should be called in `derivedT::loadConfig` as
      * \code
        dmOptimizer<derivedT,realT>::loadConfig(config);
        \endcode
      * with appropriate error checking.
      */
    int loadConfig( mx::app::appConfigurator & config /**< [in] an application configuration from which to load values */);

   /// Startup function
   /** 
     * This should be called in `derivedT::appStartup` as
     * \code
       dmOptimizer<derivedT>::appStartup();
       \endcode
     * with appropriate error checking.
     * 
     * \returns 0 on success
     * \returns -1 on error, which is logged.
     */
    int appStartup();

   /// dmOptimizer application logic
   /** This should be called in `derivedT::appLogic` as
     * \code
       dmOptimizer<derivedT>::appLogic();
       \endcode
     * with appropriate error checking.
     * 
     * \returns 0 on success
     * \returns -1 on error, which is logged.
     */
    int appLogic();

   /// dmOptimizer shutdown
   /** This should be called in `derivedT::appShutdown` as
     * \code
       dmOptimizer<derivedT>::appShutdown();
       \endcode
     * with appropriate error checking.
     * 
     * \returns 0 on success
     * \returns -1 on error, which is logged.
     */
    int appShutdown();

    ///@}

    /** \name Optimization Interface
      * @{
      */

    /// Start the optimization process
    /** 
      * \returns 0 on success
      * \returns < 0 on error
      */
    int startOptimization();

    /// Stop the optimization process
    /** 
      * \returns 0 on success
      * \returns < 0 on error
      */
    int stopOptimization();

    /// Check if optimization has converged
    /** 
      * \returns true if converged
      * \returns false if not converged
      */
    bool isConverged() const;

    /// Get the current optimization status
    /** 
      * \returns string describing current status
      */
    std::string getOptimizationStatus() const;

    ///@}

    /** \name INDI Interface 
      * @{ 
      */
protected:

    pcf::IndiProperty m_indiP_optimizationAlgorithm;
    INDI_NEWCALLBACK_DECL(derivedT, m_indiP_optimizationAlgorithm);

    pcf::IndiProperty m_indiP_convergenceThreshold;
    INDI_NEWCALLBACK_DECL(derivedT, m_indiP_convergenceThreshold);

    pcf::IndiProperty m_indiP_maxIterations;
    INDI_NEWCALLBACK_DECL(derivedT, m_indiP_maxIterations);

    pcf::IndiProperty m_indiP_learningRate;
    INDI_NEWCALLBACK_DECL(derivedT, m_indiP_learningRate);

    pcf::IndiProperty m_indiP_useAdaptiveStepSize;
    INDI_NEWCALLBACK_DECL(derivedT, m_indiP_useAdaptiveStepSize);

    pcf::IndiProperty m_indiP_startOptimization;
    INDI_NEWCALLBACK_DECL(derivedT, m_indiP_startOptimization);

    pcf::IndiProperty m_indiP_stopOptimization;
    INDI_NEWCALLBACK_DECL(derivedT, m_indiP_stopOptimization);

    pcf::IndiProperty m_indiP_optimizationStatus;
    INDI_NEWCALLBACK_DECL(derivedT, m_indiP_optimizationStatus);

    pcf::IndiProperty m_indiP_results;
    INDI_NEWCALLBACK_DECL(derivedT, m_indiP_results);

    ///@}

    /** \name Telemeter Interface 
      * @{ 
      */

    int recordTelem(const telem_optimization *);

    int recordOptimization(bool force = false);

    ///@}

private:
    derivedT & derived()
    {
        return *static_cast<derivedT *>(this);
    }

    int updateOptimizationStatus();
    int updateResults();
};

template<class derivedT>
int dmOptimizer<derivedT>::setupConfig(mx::app::appConfigurator & config)
{
    config.add("optimizer.algorithm", "", "optimizer.algorithm", argType::Required, "optimizer", "algorithm", false, "string", "The optimization algorithm to use");
    config.add("optimizer.convergenceThreshold", "", "optimizer.convergenceThreshold", argType::Required, "optimizer", "convergenceThreshold", false, "float", "Convergence threshold for optimization");
    config.add("optimizer.maxIterations", "", "optimizer.maxIterations", argType::Required, "optimizer", "maxIterations", false, "int", "Maximum number of optimization iterations");
    config.add("optimizer.learningRate", "", "optimizer.learningRate", argType::Required, "optimizer", "learningRate", false, "float", "Learning rate for gradient-based optimization");
    config.add("optimizer.useAdaptiveStepSize", "", "optimizer.useAdaptiveStepSize", argType::Required, "optimizer", "useAdaptiveStepSize", false, "bool", "Whether to use adaptive step size");

    return 0;    
}

template<class derivedT>
int dmOptimizer<derivedT>::loadConfig( mx::app::appConfigurator & config)
{
    config(m_optimizationAlgorithm, "optimizer.algorithm");
    config(m_convergenceThreshold, "optimizer.convergenceThreshold");
    config(m_maxIterations, "optimizer.maxIterations");
    config(m_learningRate, "optimizer.learningRate");
    config(m_useAdaptiveStepSize, "optimizer.useAdaptiveStepSize");

    return 0;
}

template<class derivedT>
int dmOptimizer<derivedT>::appStartup()
{
    // Initialize optimization state
    m_optimizationInProgress = false;
    m_currentIteration = 0;
    m_currentCost = std::numeric_limits<double>::max();
    m_previousCost = std::numeric_limits<double>::max();

    // Setup INDI properties
    CREATE_REG_INDI_NEW_TEXT_DERIVED(m_indiP_optimizationAlgorithm, "optimizationAlgorithm", "", "");
    m_indiP_optimizationAlgorithm["current"].setValue(m_optimizationAlgorithm);

    CREATE_REG_INDI_NEW_NUMBERF_DERIVED(m_indiP_convergenceThreshold, "convergenceThreshold", 1e-6, 1e-3, 1e-6, "%0.2e", "", "");
    m_indiP_convergenceThreshold["current"].setValue(m_convergenceThreshold);

    CREATE_REG_INDI_NEW_NUMBERI_DERIVED(m_indiP_maxIterations, "maxIterations", 1, 10000, 1, "%d", "", "");
    m_indiP_maxIterations["current"].setValue(m_maxIterations);

    CREATE_REG_INDI_NEW_NUMBERF_DERIVED(m_indiP_learningRate, "learningRate", 0.001, 1.0, 0.001, "%0.3f", "", "");
    m_indiP_learningRate["current"].setValue(m_learningRate);

    CREATE_REG_INDI_NEW_TOGGLESWITCH_DERIVED(m_indiP_useAdaptiveStepSize, "useAdaptiveStepSize");

    CREATE_REG_INDI_NEW_TOGGLESWITCH_DERIVED(m_indiP_startOptimization, "startOptimization");
    CREATE_REG_INDI_NEW_TOGGLESWITCH_DERIVED(m_indiP_stopOptimization, "stopOptimization");

    derived().template registerIndiPropertyReadOnly(m_indiP_optimizationStatus, "optimizationStatus", pcf::IndiProperty::Text, pcf::IndiProperty::ReadOnly, pcf::IndiProperty::Idle);
    m_indiP_optimizationStatus.add({"status", "Idle"});

    derived().template registerIndiPropertyReadOnly(m_indiP_results, "results", pcf::IndiProperty::Number, pcf::IndiProperty::ReadOnly, pcf::IndiProperty::Idle);
    m_indiP_results.add({"iteration", 0});
    m_indiP_results.add({"cost", 0.0});
    m_indiP_results.add({"converged", 0});

    return 0;
}

template<class derivedT>
int dmOptimizer<derivedT>::appLogic()
{
    // Update INDI properties
    derived().template updateIfChanged(m_indiP_optimizationAlgorithm, "current", m_optimizationAlgorithm);
    derived().template updateIfChanged(m_indiP_convergenceThreshold, "current", m_convergenceThreshold);
    derived().template updateIfChanged(m_indiP_maxIterations, "current", m_maxIterations);
    derived().template updateIfChanged(m_indiP_learningRate, "current", m_learningRate);

    return 0;
}

template<class derivedT>
int dmOptimizer<derivedT>::appShutdown()
{
    return 0;
}

template<class derivedT>
int dmOptimizer<derivedT>::startOptimization()
{
    if(m_optimizationInProgress)
    {
        return 0; // Already running
    }

    m_optimizationInProgress = true;
    m_currentIteration = 0;
    m_currentCost = std::numeric_limits<double>::max();
    m_previousCost = std::numeric_limits<double>::max();
    
    m_optimizationHistory.clear();
    m_costHistory.clear();
    m_gradientHistory.clear();

    updateOptimizationStatus();
    return 0;
}

template<class derivedT>
int dmOptimizer<derivedT>::stopOptimization()
{
    m_optimizationInProgress = false;
    updateOptimizationStatus();
    return 0;
}

template<class derivedT>
bool dmOptimizer<derivedT>::isConverged() const
{
    if(m_currentIteration == 0) return false;
    
    double costDiff = std::abs(m_currentCost - m_previousCost);
    return costDiff < m_convergenceThreshold;
}

template<class derivedT>
std::string dmOptimizer<derivedT>::getOptimizationStatus() const
{
    if(!m_optimizationInProgress) return "Idle";
    if(isConverged()) return "Converged";
    return "Running";
}

template<class derivedT>
int dmOptimizer<derivedT>::updateOptimizationStatus()
{
    std::string status = getOptimizationStatus();
    m_indiP_optimizationStatus["status"] = status;
    derived().template updateIfChanged(m_indiP_optimizationStatus, "status", status);
    return 0;
}

template<class derivedT>
int dmOptimizer<derivedT>::updateResults()
{
    derived().template updateIfChanged(m_indiP_results, "iteration", m_currentIteration);
    derived().template updateIfChanged(m_indiP_results, "cost", m_currentCost);
    derived().template updateIfChanged(m_indiP_results, "converged", isConverged() ? 1 : 0);
    return 0;
}

// INDI Callbacks
template<class derivedT>
INDI_NEWCALLBACK_DEFN( dmOptimizer<derivedT>, m_indiP_optimizationAlgorithm )(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS_DERIVED(m_indiP_optimizationAlgorithm, ipRecv)
   
    std::string target;
    if(derived().template indiTargetUpdate(m_indiP_optimizationAlgorithm, target, ipRecv, false) < 0)
    {
        return derivedT::template log<software_error,-1>({__FILE__, __LINE__});
    }

    m_optimizationAlgorithm = target;
    return 0;
}

template<class derivedT>
INDI_NEWCALLBACK_DEFN( dmOptimizer<derivedT>, m_indiP_convergenceThreshold )(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS_DERIVED(m_indiP_convergenceThreshold, ipRecv)
   
    float target;
    if(derived().template indiTargetUpdate(m_indiP_convergenceThreshold, target, ipRecv, false) < 0)
    {
        return derivedT::template log<software_error,-1>({__FILE__, __LINE__});
    }

    m_convergenceThreshold = target;
    return 0;
}

template<class derivedT>
INDI_NEWCALLBACK_DEFN( dmOptimizer<derivedT>, m_indiP_maxIterations )(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS_DERIVED(m_indiP_maxIterations, ipRecv)
   
    float target;
    if(derived().template indiTargetUpdate(m_indiP_maxIterations, target, ipRecv, false) < 0)
    {
        return derivedT::template log<software_error,-1>({__FILE__, __LINE__});
    }

    m_maxIterations = target;
    return 0;
}

template<class derivedT>
INDI_NEWCALLBACK_DEFN( dmOptimizer<derivedT>, m_indiP_learningRate )(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS_DERIVED(m_indiP_learningRate, ipRecv)
   
    float target;
    if(derived().template indiTargetUpdate(m_indiP_learningRate, target, ipRecv, false) < 0)
    {
        return derivedT::template log<software_error,-1>({__FILE__, __LINE__});
    }

    m_learningRate = target;
    return 0;
}

template<class derivedT>
INDI_NEWCALLBACK_DEFN( dmOptimizer<derivedT>, m_indiP_useAdaptiveStepSize )(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS_DERIVED(m_indiP_useAdaptiveStepSize, ipRecv)
   
    if(ipRecv.find("toggle") != true)
    {
        return -1;
    }

    m_useAdaptiveStepSize = (ipRecv["toggle"].getSwitchState() == pcf::IndiElement::On);
    return 0;
}

template<class derivedT>
INDI_NEWCALLBACK_DEFN( dmOptimizer<derivedT>, m_indiP_startOptimization )(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS_DERIVED(m_indiP_startOptimization, ipRecv)
   
    if(ipRecv.find("toggle") != true)
    {
        return -1;
    }

    if(ipRecv["toggle"].getSwitchState() == pcf::IndiElement::On)
    {
        if(startOptimization() < 0)
        {
            return derivedT::template log<software_error,-1>({__FILE__, __LINE__});
        }
    }

    return 0;
}

template<class derivedT>
INDI_NEWCALLBACK_DEFN( dmOptimizer<derivedT>, m_indiP_stopOptimization )(const pcf::IndiProperty &ipRecv)
{
    INDI_VALIDATE_CALLBACK_PROPS_DERIVED(m_indiP_stopOptimization, ipRecv)
   
    if(ipRecv.find("toggle") != true)
    {
        return -1;
    }

    if(ipRecv["toggle"].getSwitchState() == pcf::IndiElement::On)
    {
        if(stopOptimization() < 0)
        {
            return derivedT::template log<software_error,-1>({__FILE__, __LINE__});
        }
    }

    return 0;
}

// Telemeter interface
template<class derivedT>
int dmOptimizer<derivedT>::recordTelem(const telem_optimization *)
{
    return recordOptimization(true);
}

template<class derivedT>
int dmOptimizer<derivedT>::recordOptimization(bool force)
{
    // Placeholder for telemetry recording
    return 0;
}

/// Call dmOptimizer::setupConfig with error checking
/**
  * \param cfig the application configurator 
  */
#define DMOPTIMIZER_SETUP_CONFIG( cfig )                                                   \
    if(dmOptimizerT::setupConfig(cfig) < 0)                                                \
    {                                                                                    \
        log<software_error>({__FILE__, __LINE__, "Error from dmOptimizerT::setupConfig"}); \
        m_shutdown = true;                                                               \
        return;                                                                          \
    }

/// Call dmOptimizer::loadConfig with error checking
/** This must be inside a function that returns int, e.g. the standard loadConfigImpl.
  * \param cfig the application configurator 
  */
#define DMOPTIMIZER_LOAD_CONFIG( cfig )                                                             \
    if(dmOptimizerT::loadConfig(cfig) < 0)                                                          \
    {                                                                                             \
        return log<software_error,-1>({__FILE__, __LINE__, "Error from dmOptimizerT::loadConfig"}); \
    } 

/// Call dmOptimizer::appStartup with error checking
#define DMOPTIMIZER_APP_STARTUP                                \
    if( dmOptimizerT::appStartup() < 0)                        \
    {                                                        \
        return log<software_error, -1>({__FILE__,__LINE__}); \
    }

/// Call dmOptimizer::appLogic with error checking
#define DMOPTIMIZER_APP_LOGIC                                  \
    if( dmOptimizerT::appLogic() < 0)                          \
    {                                                        \
        return log<software_error, -1>({__FILE__,__LINE__}); \
    }

/// Call dmOptimizer::appShutdown with error checking
#define DMOPTIMIZER_APP_SHUTDOWN                                                           \
    if(dmOptimizerT::appShutdown() < 0)                                                    \
    {                                                                                    \
        log<software_error>({__FILE__, __LINE__, "error from dmOptimizerT::appShutdown"}); \
    }

} //namespace dev
} //namespace app
} //namespace MagAOX

#endif //dmOptimizer_hpp
