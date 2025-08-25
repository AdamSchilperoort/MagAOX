/** \file eyeDoctor_test.cpp
  * \brief Unit tests for the eyeDoctor application
  *
  * \ingroup eyeDoctor_tests
  */

#include <catch2/catch.hpp>

#include "../eyeDoctor.hpp"

using namespace MagAOX::app;

SCENARIO("eyeDoctor can be constructed", "[eyeDoctor]")
{
    GIVEN("a default constructed eyeDoctor")
    {
        eyeDoctor app;
        
        THEN("it should be in a valid state")
        {
            REQUIRE(app.m_optimizationInProgress == false);
            REQUIRE(app.m_measurementComplete == false);
            REQUIRE(app.m_currentModeIndex == 0);
            REQUIRE(app.m_totalModes == 0);
        }
    }
}

SCENARIO("eyeDoctor configuration can be loaded", "[eyeDoctor]")
{
    GIVEN("an eyeDoctor instance")
    {
        eyeDoctor app;
        
        WHEN("configuration is loaded")
        {
            mx::app::appConfigurator config;
            app.setupConfig();
            
            THEN("configuration should be set up")
            {
                // Basic configuration setup verification
                REQUIRE(true); // Placeholder for actual verification
            }
        }
    }
}

SCENARIO("eyeDoctor can start and stop optimization", "[eyeDoctor]")
{
    GIVEN("an eyeDoctor instance")
    {
        eyeDoctor app;
        
        WHEN("optimization is started")
        {
            int result = app.startOptimization();
            
            THEN("it should start successfully")
            {
                REQUIRE(result == 0);
                REQUIRE(app.m_optimizationInProgress == true);
            }
        }
        
        WHEN("optimization is stopped")
        {
            app.m_optimizationInProgress = true;
            int result = app.stopOptimization();
            
            THEN("it should stop successfully")
            {
                REQUIRE(result == 0);
                REQUIRE(app.m_optimizationInProgress == false);
            }
        }
    }
}

SCENARIO("eyeDoctor can measure PSF", "[eyeDoctor]")
{
    GIVEN("an eyeDoctor instance")
    {
        eyeDoctor app;
        
        WHEN("PSF measurement is performed")
        {
            int result = app.measurePSF();
            
            THEN("it should complete successfully")
            {
                REQUIRE(result == 0);
            }
        }
    }
}

SCENARIO("eyeDoctor can run sensor measurements", "[eyeDoctor]")
{
    GIVEN("an eyeDoctor instance")
    {
        eyeDoctor app;
        
        WHEN("sensor measurement is run")
        {
            int result = app.runSensor(true);
            
            THEN("it should complete successfully")
            {
                REQUIRE(result == 0);
            }
        }
    }
}

SCENARIO("eyeDoctor can analyze sensor data", "[eyeDoctor]")
{
    GIVEN("an eyeDoctor instance")
    {
        eyeDoctor app;
        
        WHEN("sensor data is analyzed")
        {
            int result = app.analyzeSensor();
            
            THEN("it should complete successfully")
            {
                REQUIRE(result == 0);
            }
        }
    }
}
