/** \file eyeDoctor.hpp
  * \brief MagAO-X Eye Doctor: modal DM grid-search to maximize PSF core flux.
  *
  * C++ port of magpyx `eye_doctor_comprehensive`. Commands a remote INDI
  * modeset device (typically a `dmMode` instance such as `alpaoModes`) via
  * `target_amps` / `current_amps`. That app owns the modeset, converts
  * amplitudes to a DM shape, and writes the cacao channel. This app only
  * sends mode amplitudes and measures PSF core flux on a camera shmim.
  *
  * \ingroup eyeDoctor_files
  */

#ifndef eyeDoctor_hpp
#define eyeDoctor_hpp

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstddef>
#include <ctime>
#include <iomanip>
#include <limits>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include <mx/improc/eigenImage.hpp>
#include <mx/sys/timeUtils.hpp>

#include "../../libMagAOX/libMagAOX.hpp"
#include "../../magaox_git_version.h"
#include "../../libMagAOX/app/dev/dmWavefrontControl.hpp"
#include <lina/dark_library.h>

/** \defgroup eyeDoctor
  * \brief Modal DM PSF optimization (eye doctor)
  *
  * \ingroup apps
  */

/** \defgroup eyeDoctor_files
  * \ingroup eyeDoctor
  */

namespace MagAOX
{
namespace app
{

/// MagAO-X Eye Doctor application
/** INDI front-end for magpyx `eye_doctor_comprehensive`.
  *
  * Hardware:
  *  - \c modes_device : INDI dmMode app (`alpaoModes`, `wooferModes`, ...)
  *  - \c shm_cam      : WFS / science-camera image shmim
  *  - \c cam_name     : INDI device of that camera
  *  - \c shm_dm_flat / \c shm_dm_sum : optional, used only by save_flat
  *
  * Equivalent CLI:
  * `dm_eye_doctor <port> alpaoModes camsci 8 2...10 0.1 --skip 1`
  *
  * \ingroup eyeDoctor
  */
class eyeDoctor : public MagAOXApp<true>
{
  public:
    ~eyeDoctor() noexcept
    {
    }

  protected:
    /** \name Hardware names
      *@{
      */
    std::string m_modesDevice{ "alpaoModes" }; ///< INDI dmMode device (target_amps / current_amps)
    std::string m_shmDmFlat{ "dm01disp00" };
    std::string m_shmDmSum{ "dm01disp" };
    std::string m_shmCam{ "camsci" };
    std::string m_camName{ "camsci" };
    std::string m_flatDir{ "/opt/MagAOX/calib/dm/bmc_1k/flats" };
    std::string m_lastFlatPath;
    std::string m_darkLibPath; ///< darkCtrl library (dark_metadata.txt + dark_NNN.fits)
    ///@}

    /** \name Algorithm parameters (magpyx eye_doctor_comprehensive / dm_eye_doctor)
      *@{
      */
    int m_modeStart{ 2 };
    int m_modeEnd{ 10 };
    int m_focusModeIndex{ 2 }; ///< magpyx focus-first mode (default 2)
    double m_coreRadius{ 8.0 };
    double m_searchRange{ 0.1 }; ///< Total span; sweep is [-range/2, +range/2] about baseline
    double m_searchStep{ 0.0 };  ///< Amplitude spacing; 0 = use n_steps
    int m_nSteps{ 20 };
    int m_nRepeats{ 3 };
    int m_nCluster{ 5 };
    int m_nClusterRepeat{ 1 };
    int m_nSeqRepeat{ 1 };
    int m_nImages{ 1 };
    int m_skipFrames{ 1 };
    double m_cenX{ -1.0 }; ///< <0 = auto centroid
    double m_cenY{ -1.0 };
    double m_satThresh{ 55000.0 }; ///< Warn if camera peak >= this (0 = off)
    double m_blankThresh{ 0.0 };   ///< Peak ADU treated as off-camera. 0 = 10% of sweep max.
    double m_exptimeTol{ 1e-4 };    ///< |live exptime - library exptime| allowed [s]
    double m_dmDelay{ 0.1 }; ///< Extra settle after the modes device reports current==target [s]
    double m_ampTol{ 1e-3 }; ///< |current_amps - target| wait tolerance
    double m_ampTimeout{ 10.0 }; ///< Seconds to wait for current_amps
    std::string m_searchKind{ "grid" }; ///< magpyx search_kind: grid or brent
    std::string m_gridKind{ "fit" };    ///< magpyx grid_sweep skind: fit or mean (grid only)
    bool m_baseline{ true }; ///< Center each sweep on the live current_amps value
    bool m_randomize{ true }; ///< Shuffle modes inside each cluster
    bool m_ignoreFocus{ false }; ///< Skip the extra focus-first pass
    ///@}

    /** \name Live camera SET
      *@{
      */
    double m_remoteExp{ std::numeric_limits<double>::quiet_NaN() };
    double m_remoteFps{ std::numeric_limits<double>::quiet_NaN() };
    double m_remoteGain{ std::numeric_limits<double>::quiet_NaN() };
    double m_remoteBlacklevel{ std::numeric_limits<double>::quiet_NaN() };
    ///@}

    /** \name Dark library (matched to live camera SET)
      *@{
      */
    mx::improc::eigenImage<float> m_dark;
    bool m_haveDark{ false };
    std::string m_lastDarkPath;
    double m_darkExptime{ std::numeric_limits<double>::quiet_NaN() };
    double m_darkGain{ std::numeric_limits<double>::quiet_NaN() };
    double m_darkBlacklevel{ std::numeric_limits<double>::quiet_NaN() };
    double m_darkMatchErr{ std::numeric_limits<double>::quiet_NaN() };
    ///@}

    /** \name Worker
      *@{
      */
    std::thread m_worker;
    std::atomic<bool> m_workerShutdown{ false };
    std::atomic<bool> m_runRequested{ false };
    std::atomic<bool> m_saveFlatRequested{ false };
    std::atomic<bool> m_abortRequested{ false };
    std::atomic<bool> m_resetRequested{ false };
    std::atomic<bool> m_darkLibLoadRequested{ false };
    std::atomic<bool> m_busy{ false };
    std::string m_status{ "idle" };
    int m_currentMode{ -1 };
    int m_modesMax{ 0 };
    double m_lastAmp{ 0 };
    double m_lastMetric{ 0 };
    int m_satWarnedMode{ -2 };
    ///@}

    std::mutex m_modesMutex;
    std::vector<double> m_remoteAmps; ///< latest current_amps from modes_device

    dev::wavefrontHardware m_hw;

    /** \name INDI
      *@{
      */
    pcf::IndiProperty m_indiP_modesDevice;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_modesDevice );
    pcf::IndiProperty m_indiP_shmDmFlat;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_shmDmFlat );
    pcf::IndiProperty m_indiP_shmDmSum;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_shmDmSum );
    pcf::IndiProperty m_indiP_shmCam;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_shmCam );
    pcf::IndiProperty m_indiP_camName;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_camName );
    pcf::IndiProperty m_indiP_flatDir;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_flatDir );
    pcf::IndiProperty m_indiP_darkLibPath;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_darkLibPath );

    pcf::IndiProperty m_indiP_modeStart;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_modeStart );
    pcf::IndiProperty m_indiP_modeEnd;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_modeEnd );
    pcf::IndiProperty m_indiP_focusModeIndex;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_focusModeIndex );
    pcf::IndiProperty m_indiP_coreRadius;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_coreRadius );
    pcf::IndiProperty m_indiP_searchRange;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_searchRange );
    pcf::IndiProperty m_indiP_searchStep;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_searchStep );
    pcf::IndiProperty m_indiP_nSteps;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_nSteps );
    pcf::IndiProperty m_indiP_nRepeats;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_nRepeats );
    pcf::IndiProperty m_indiP_nCluster;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_nCluster );
    pcf::IndiProperty m_indiP_nClusterRepeat;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_nClusterRepeat );
    pcf::IndiProperty m_indiP_nSeqRepeat;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_nSeqRepeat );
    pcf::IndiProperty m_indiP_nImages;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_nImages );
    pcf::IndiProperty m_indiP_skipFrames;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_skipFrames );
    pcf::IndiProperty m_indiP_cenX;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_cenX );
    pcf::IndiProperty m_indiP_cenY;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_cenY );
    pcf::IndiProperty m_indiP_satThresh;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_satThresh );
    pcf::IndiProperty m_indiP_blankThresh;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_blankThresh );
    pcf::IndiProperty m_indiP_exptimeTol;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_exptimeTol );
    pcf::IndiProperty m_indiP_dmDelay;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_dmDelay );
    pcf::IndiProperty m_indiP_ampTol;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_ampTol );
    pcf::IndiProperty m_indiP_ampTimeout;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_ampTimeout );
    pcf::IndiProperty m_indiP_searchKind;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_searchKind );
    pcf::IndiProperty m_indiP_baseline;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_baseline );
    pcf::IndiProperty m_indiP_randomize;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_randomize );
    pcf::IndiProperty m_indiP_resetToZero;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_resetToZero );
    pcf::IndiProperty m_indiP_ignoreFocus;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_ignoreFocus );

    pcf::IndiProperty m_indiP_run;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_run );
    pcf::IndiProperty m_indiP_abort;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_abort );
    pcf::IndiProperty m_indiP_saveFlat;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_saveFlat );
    pcf::IndiProperty m_indiP_darkLibLoad;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_darkLibLoad );

    pcf::IndiProperty m_indiP_status;
    pcf::IndiProperty m_indiP_currentMode;
    pcf::IndiProperty m_indiP_modesMax;
    pcf::IndiProperty m_indiP_optAmp;
    pcf::IndiProperty m_indiP_metric;
    pcf::IndiProperty m_indiP_lastFlat;
    pcf::IndiProperty m_indiP_lastDark;

    pcf::IndiProperty m_indiP_remoteExptime;
    INDI_SETCALLBACK_DECL( eyeDoctor, m_indiP_remoteExptime );
    pcf::IndiProperty m_indiP_remoteFps;
    INDI_SETCALLBACK_DECL( eyeDoctor, m_indiP_remoteFps );
    pcf::IndiProperty m_indiP_remoteEmgain;
    INDI_SETCALLBACK_DECL( eyeDoctor, m_indiP_remoteEmgain );
    pcf::IndiProperty m_indiP_remoteBlacklevel;
    INDI_SETCALLBACK_DECL( eyeDoctor, m_indiP_remoteBlacklevel );
    pcf::IndiProperty m_indiP_remoteAmps;
    INDI_SETCALLBACK_DECL( eyeDoctor, m_indiP_remoteAmps );
    ///@}

  public:
    eyeDoctor();

    virtual void setupConfig();
    virtual void loadConfig();
    virtual int appStartup();
    virtual int appLogic();
    virtual int appShutdown();

  protected:
    static void workerStart( eyeDoctor *e );
    void workerExec();
    int runOptimization();
    int optimizeMode( int mi, mx::improc::eigenImage<float> &camIm );
    int saveFlat();
    int abortAndZero();
    int resetToZero();
    int zeroAllModes();
    int sendModeAmp( int mode, double amp );
    int waitModeAmp( int mode, double amp );
    int sendModeAndWait( int mode, double amp );
    int nRemoteModes();
    double currentAmp( int mode );
    int waitForModesDevice();
    std::vector<int> requestedModes() const;
    std::vector<int> allowedModes( const std::vector<int> &req, int nAvail, bool *truncated ) const;
    std::vector<int> buildSequence( const std::vector<int> &modes ) const;
    static std::string modeElementName( int mode );
    static int parseSearchKind( const std::string &in, std::string &searchKind, std::string &gridKind,
                                std::string *err );
    int reloadDarkLib();
    int refreshDark( bool required );
    lina::DarkMatchFilter darkFilter() const;
    std::string formatDarkEntry( const lina::DarkLibraryEntry &e ) const;
    std::string pickDark( double target_exptime, const lina::DarkMatchFilter &filter,
                           lina::DarkLibraryEntry *matched, double *match_err );
    void applyDark( mx::improc::eigenImage<float> &im );
    int measureMetric( mx::improc::eigenImage<float> &im, double &metric );
    void warnIfSaturated( const mx::improc::eigenImage<float> &im );
    void setStatus( const std::string &s );
    void setRunToggle( bool on, pcf::IndiProperty::PropertyStateType st );
    void clearRequest( pcf::IndiProperty &p );
    bool stopping();
    static int ensureDirectory( const std::string &path );
    static std::string timestampNow();
};

eyeDoctor::eyeDoctor() : MagAOXApp( MAGAOX_CURRENT_SHA1, MAGAOX_REPO_MODIFIED )
{
    m_loopPause = 100000000; // 100 ms
}

void eyeDoctor::setupConfig()
{
    config.add( "eyedoctor.modes_device", "", "eyedoctor.modes_device", argType::Required, "eyedoctor",
                "modes_device", false, "string",
                "INDI dmMode device that owns the modeset (e.g. alpaoModes)." );
    config.add( "shmims.shm_dm_flat", "", "shmims.shm_dm_flat", argType::Required, "shmims", "shm_dm_flat", false,
                "string", "cacao flat channel (typically dmXXdisp00). Used by save_flat." );
    config.add( "shmims.shm_dm_sum", "", "shmims.shm_dm_sum", argType::Required, "shmims", "shm_dm_sum", false, "string",
                "cacao summed / total DM command (dmXXdisp). Used by save_flat." );
    config.add( "shmims.shm_cam", "", "shmims.shm_cam", argType::Required, "shmims", "shm_cam", false, "string",
                "WFS / science camera image shmim." );
    config.add( "camera.cam_name", "", "camera.cam_name", argType::Required, "camera", "cam_name", false, "string",
                "INDI device name of the WFS camera (exptime/emgain/blacklevel)." );
    config.add( "eyedoctor.flat_dir", "", "eyedoctor.flat_dir", argType::Required, "eyedoctor", "flat_dir", false,
                "string", "Directory for saved flat FITS." );
    config.add( "eyedoctor.dark_lib_path", "", "eyedoctor.dark_lib_path", argType::Required, "eyedoctor",
                "dark_lib_path", false, "string",
                "darkCtrl library directory (dark_metadata.txt + dark_NNN.fits)." );
    config.add( "eyedoctor.exptime_tol", "", "eyedoctor.exptime_tol", argType::Required, "eyedoctor", "exptime_tol",
                false, "float", "Max |live-library| exptime difference [s] when picking a dark." );
    config.add( "eyedoctor.mode_start", "", "eyedoctor.mode_start", argType::Required, "eyedoctor", "mode_start", false,
                "int", "First 0-based mode index when modes is empty." );
    config.add( "eyedoctor.mode_end", "", "eyedoctor.mode_end", argType::Required, "eyedoctor", "mode_end", false, "int",
                "Last 0-based mode index when modes is empty (inclusive)." );
    config.add( "eyedoctor.focus_mode_index", "", "eyedoctor.focus_mode_index", argType::Required, "eyedoctor",
                "focus_mode_index", false, "int",
                "Mode optimized first unless ignore_focus (magpyx default 2)." );
    config.add( "eyedoctor.core_radius", "", "eyedoctor.core_radius", argType::Required, "eyedoctor", "core_radius",
                false, "float", "PSF core radius [pixels] for coresum metric." );
    config.add( "eyedoctor.search_range", "", "eyedoctor.search_range", argType::Required, "eyedoctor", "search_range",
                false, "float", "Total amplitude span; sweep is +/- range/2 about baseline." );
    config.add( "eyedoctor.search_step", "", "eyedoctor.search_step", argType::Required, "eyedoctor", "search_step",
                false, "float", "Amplitude step size. If >0, n_steps is derived as range/step + 1." );
    config.add( "eyedoctor.n_steps", "", "eyedoctor.n_steps", argType::Required, "eyedoctor", "n_steps", false, "int",
                "Grid samples per sweep." );
    config.add( "eyedoctor.n_repeats", "", "eyedoctor.n_repeats", argType::Required, "eyedoctor", "n_repeats", false,
                "int", "Number of sweep repeats averaged / jointly fit." );
    config.add( "eyedoctor.n_cluster", "", "eyedoctor.n_cluster", argType::Required, "eyedoctor", "n_cluster", false,
                "int", "Modes per shuffled cluster (magpyx ncluster, default 5)." );
    config.add( "eyedoctor.n_cluster_repeat", "", "eyedoctor.n_cluster_repeat", argType::Required, "eyedoctor",
                "n_cluster_repeat", false, "int", "Times to repeat each cluster (magpyx --nclusterrepeats)." );
    config.add( "eyedoctor.n_seq_repeat", "", "eyedoctor.n_seq_repeat", argType::Required, "eyedoctor", "n_seq_repeat",
                false, "int", "Repeat the full mode sequence this many times." );
    config.add( "eyedoctor.n_images", "", "eyedoctor.n_images", argType::Required, "eyedoctor", "n_images", false,
                "int", "Camera frames averaged per metric sample." );
    config.add( "eyedoctor.skip_frames", "", "eyedoctor.skip_frames", argType::Required, "eyedoctor", "skip_frames",
                false, "int", "Camera frames to discard after each mode command." );
    config.add( "eyedoctor.cen_x", "", "eyedoctor.cen_x", argType::Required, "eyedoctor", "cen_x", false, "float",
                "PSF x pixel in the camera image (size[0], 0-based). <0 = auto." );
    config.add( "eyedoctor.cen_y", "", "eyedoctor.cen_y", argType::Required, "eyedoctor", "cen_y", false, "float",
                "PSF y pixel in the camera image (size[1], 0-based). <0 = auto." );
    config.add( "eyedoctor.sat_thresh", "", "eyedoctor.sat_thresh", argType::Required, "eyedoctor", "sat_thresh",
                false, "float", "Warn if camera peak ADU >= this. 0 disables." );
    config.add( "eyedoctor.blank_thresh", "", "eyedoctor.blank_thresh", argType::Required, "eyedoctor", "blank_thresh",
                false, "float", "Peak ADU treated as PSF off-camera. 0 = 10% of the sweep's max peak." );
    config.add( "eyedoctor.dm_delay", "", "eyedoctor.dm_delay", argType::Required, "eyedoctor", "dm_delay", false,
                "float", "Extra settle after current_amps matches target [s]." );
    config.add( "eyedoctor.amp_tol", "", "eyedoctor.amp_tol", argType::Required, "eyedoctor", "amp_tol", false, "float",
                "Wait until |current_amps-target| < this." );
    config.add( "eyedoctor.amp_timeout", "", "eyedoctor.amp_timeout", argType::Required, "eyedoctor", "amp_timeout",
                false, "float", "Seconds to wait for current_amps after sending target_amps." );
    config.add( "eyedoctor.search_kind", "", "eyedoctor.search_kind", argType::Required, "eyedoctor", "search_kind",
                false, "string", "magpyx search: grid (default, quadratic grid sweep) or brent." );
    config.add( "eyedoctor.baseline", "", "eyedoctor.baseline", argType::Required, "eyedoctor", "baseline", false,
                "bool", "Center each sweep on the live current_amps value (magpyx default)." );
    config.add( "eyedoctor.randomize", "", "eyedoctor.randomize", argType::Required, "eyedoctor", "randomize", false,
                "bool", "Shuffle modes inside each cluster." );
    config.add( "eyedoctor.ignore_focus", "", "eyedoctor.ignore_focus", argType::Required, "eyedoctor", "ignore_focus",
                false, "bool", "Skip the extra focus-first pass." );
}

void eyeDoctor::loadConfig()
{
    config( m_modesDevice, "eyedoctor.modes_device" );
    config( m_shmDmFlat, "shmims.shm_dm_flat" );
    config( m_shmDmSum, "shmims.shm_dm_sum" );
    config( m_shmCam, "shmims.shm_cam" );
    config( m_camName, "camera.cam_name" );
    config( m_flatDir, "eyedoctor.flat_dir" );
    config( m_darkLibPath, "eyedoctor.dark_lib_path" );
    config( m_exptimeTol, "eyedoctor.exptime_tol" );
    config( m_modeStart, "eyedoctor.mode_start" );
    config( m_modeEnd, "eyedoctor.mode_end" );
    config( m_focusModeIndex, "eyedoctor.focus_mode_index" );
    config( m_coreRadius, "eyedoctor.core_radius" );
    config( m_searchRange, "eyedoctor.search_range" );
    config( m_searchStep, "eyedoctor.search_step" );
    config( m_nSteps, "eyedoctor.n_steps" );
    config( m_nRepeats, "eyedoctor.n_repeats" );
    config( m_nCluster, "eyedoctor.n_cluster" );
    config( m_nClusterRepeat, "eyedoctor.n_cluster_repeat" );
    config( m_nSeqRepeat, "eyedoctor.n_seq_repeat" );
    config( m_nImages, "eyedoctor.n_images" );
    config( m_skipFrames, "eyedoctor.skip_frames" );
    config( m_cenX, "eyedoctor.cen_x" );
    config( m_cenY, "eyedoctor.cen_y" );
    config( m_satThresh, "eyedoctor.sat_thresh" );
    config( m_blankThresh, "eyedoctor.blank_thresh" );
    config( m_dmDelay, "eyedoctor.dm_delay" );
    config( m_ampTol, "eyedoctor.amp_tol" );
    config( m_ampTimeout, "eyedoctor.amp_timeout" );
    config( m_searchKind, "eyedoctor.search_kind" );
    {
        std::string kind, gkind, err;
        if( parseSearchKind( m_searchKind, kind, gkind, &err ) == 0 )
        {
            m_searchKind = kind;
            m_gridKind = gkind;
        }
        else
        {
            log<text_log>( "eyedoctor.search_kind: " + err + "; using grid", logPrio::LOG_WARNING );
            m_searchKind = "grid";
            m_gridKind = "fit";
        }
    }
    config( m_baseline, "eyedoctor.baseline" );
    config( m_randomize, "eyedoctor.randomize" );
    config( m_ignoreFocus, "eyedoctor.ignore_focus" );
}

int eyeDoctor::appStartup()
{
    CREATE_REG_INDI_NEW_TEXT( m_indiP_modesDevice, "modes_device", "INDI dmMode device (alpaoModes)", "modes" );
    CREATE_REG_INDI_NEW_TEXT( m_indiP_shmDmFlat, "shm_dm_flat", "cacao flat DM channel", "shmims" );
    CREATE_REG_INDI_NEW_TEXT( m_indiP_shmDmSum, "shm_dm_sum", "cacao summed DM command", "shmims" );
    CREATE_REG_INDI_NEW_TEXT( m_indiP_shmCam, "shm_cam", "WFS camera image shmim", "shmims" );
    CREATE_REG_INDI_NEW_TEXT( m_indiP_camName, "cam_name", "INDI WFS camera device", "camera" );
    CREATE_REG_INDI_NEW_TEXT( m_indiP_flatDir, "flat_dir", "Directory for saved flat FITS", "flat" );
    CREATE_REG_INDI_NEW_TEXT( m_indiP_darkLibPath, "dark_lib_path", "darkCtrl library directory", "paths" );

    CREATE_REG_INDI_NEW_NUMBERI( m_indiP_modeStart, "mode_start", 0, 10000, 1, "%d", "First mode index", "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERI( m_indiP_modeEnd, "mode_end", 0, 10000, 1, "%d", "Last mode index", "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERI( m_indiP_focusModeIndex, "focus_mode_index", 0, 10000, 1, "%d", "Focus-first mode index",
                                "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERF( m_indiP_coreRadius, "core_radius", 0.5, 500, 0.5, "%0.2f", "PSF core radius [pix]",
                                "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERF( m_indiP_searchRange, "search_range", 0, 10, 0.01, "%0.4f", "Sweep span (full)",
                                "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERF( m_indiP_searchStep, "search_step", 0, 10, 0.001, "%0.4f",
                                "Amplitude step (0 = use n_steps)", "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERI( m_indiP_nSteps, "n_steps", 3, 500, 1, "%d", "Grid samples", "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERI( m_indiP_nRepeats, "n_repeats", 1, 50, 1, "%d", "Sweep repeats", "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERI( m_indiP_nCluster, "n_cluster", 1, 500, 1, "%d", "Modes per cluster", "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERI( m_indiP_nClusterRepeat, "n_cluster_repeat", 1, 50, 1, "%d", "Cluster repeats",
                                "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERI( m_indiP_nSeqRepeat, "n_seq_repeat", 1, 50, 1, "%d", "Full-sequence repeats",
                                "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERI( m_indiP_nImages, "n_images", 1, 10000, 1, "%d", "Frames averaged", "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERI( m_indiP_skipFrames, "skip_frames", 0, 1000, 1, "%d", "Frames skipped after DM",
                                "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERF( m_indiP_cenX, "cen_x", -1, 10000, 0.01, "%0.2f", "PSF x pixel in ROI (<0 auto)",
                                "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERF( m_indiP_cenY, "cen_y", -1, 10000, 0.01, "%0.2f", "PSF y pixel in ROI (<0 auto)",
                                "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERF( m_indiP_satThresh, "sat_thresh", 0, 1e9, 1, "%0.1f",
                                "Saturation warn threshold [ADU]", "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERF( m_indiP_blankThresh, "blank_thresh", 0, 1e9, 1, "%0.1f",
                                "Off-camera peak [ADU] (0 = 10% of max)", "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERF( m_indiP_exptimeTol, "exptime_tol", 0, 10, 1e-6, "%0.6f",
                                "Dark exptime match tolerance [s]", "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERF( m_indiP_dmDelay, "dm_delay", 0, 10, 0.01, "%0.3f", "Extra settle [s]", "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERF( m_indiP_ampTol, "amp_tol", 0, 1, 1e-6, "%0.6f", "current_amps wait tolerance",
                                "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERF( m_indiP_ampTimeout, "amp_timeout", 0.1, 120, 0.1, "%0.1f",
                                "current_amps wait timeout [s]", "algorithm" );
    CREATE_REG_INDI_NEW_TEXT( m_indiP_searchKind, "search_kind", "grid or brent (magpyx)", "algorithm" );

    if( createStandardIndiToggleSw( m_indiP_baseline, "baseline", "Center sweep on current_amps", "algorithm" ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__, "createStandardIndiToggleSw baseline" } );
    }
    if( registerIndiPropertyNew( m_indiP_baseline, INDI_NEWCALLBACK( m_indiP_baseline ) ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__, "registerIndiPropertyNew baseline" } );
    }
    if( createStandardIndiToggleSw( m_indiP_randomize, "randomize", "Shuffle modes in each cluster", "algorithm" ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__, "createStandardIndiToggleSw randomize" } );
    }
    if( registerIndiPropertyNew( m_indiP_randomize, INDI_NEWCALLBACK( m_indiP_randomize ) ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__, "registerIndiPropertyNew randomize" } );
    }
    if( createStandardIndiToggleSw( m_indiP_ignoreFocus, "ignore_focus", "Skip extra focus-first pass", "algorithm" ) <
        0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__, "createStandardIndiToggleSw ignore_focus" } );
    }
    if( registerIndiPropertyNew( m_indiP_ignoreFocus, INDI_NEWCALLBACK( m_indiP_ignoreFocus ) ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__, "registerIndiPropertyNew ignore_focus" } );
    }
    if( createStandardIndiToggleSw( m_indiP_run, "run", "Run eye doctor", "control" ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__, "createStandardIndiToggleSw run" } );
    }
    if( registerIndiPropertyNew( m_indiP_run, INDI_NEWCALLBACK( m_indiP_run ) ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__, "registerIndiPropertyNew run" } );
    }
    CREATE_REG_INDI_NEW_REQUESTSWITCH( m_indiP_abort, "abort" );
    CREATE_REG_INDI_NEW_REQUESTSWITCH( m_indiP_resetToZero, "reset_to_zero" );
    CREATE_REG_INDI_NEW_REQUESTSWITCH( m_indiP_saveFlat, "save_flat" );
    CREATE_REG_INDI_NEW_REQUESTSWITCH( m_indiP_darkLibLoad, "reload_dark_lib" );

    REG_INDI_NEWPROP_NOCB( m_indiP_status, "status", pcf::IndiProperty::Text );
    m_indiP_status.add( pcf::IndiElement( "current" ) );
    m_indiP_status["current"].set( m_status );

    REG_INDI_NEWPROP_NOCB( m_indiP_currentMode, "current_mode", pcf::IndiProperty::Number );
    m_indiP_currentMode.add( pcf::IndiElement( "current" ) );
    m_indiP_currentMode["current"].set( -1.0 );

    REG_INDI_NEWPROP_NOCB( m_indiP_modesMax, "modes_max", pcf::IndiProperty::Number );
    m_indiP_modesMax.add( pcf::IndiElement( "current" ) );
    m_indiP_modesMax["current"].set( 0.0 );

    REG_INDI_NEWPROP_NOCB( m_indiP_optAmp, "opt_amp", pcf::IndiProperty::Number );
    m_indiP_optAmp.add( pcf::IndiElement( "current" ) );
    m_indiP_optAmp["current"].set( 0.0 );

    REG_INDI_NEWPROP_NOCB( m_indiP_metric, "metric", pcf::IndiProperty::Number );
    m_indiP_metric.add( pcf::IndiElement( "current" ) );
    m_indiP_metric["current"].set( 0.0 );

    REG_INDI_NEWPROP_NOCB( m_indiP_lastFlat, "last_flat", pcf::IndiProperty::Text );
    m_indiP_lastFlat.add( pcf::IndiElement( "current" ) );
    m_indiP_lastFlat["current"].set( m_lastFlatPath );

    REG_INDI_NEWPROP_NOCB( m_indiP_lastDark, "last_dark", pcf::IndiProperty::Text );
    m_indiP_lastDark.add( pcf::IndiElement( "current" ) );
    m_indiP_lastDark["current"].set( m_lastDarkPath );

    m_indiP_modesDevice["current"].setValue( m_modesDevice );
    m_indiP_modesDevice["target"].setValue( m_modesDevice );
    m_indiP_shmDmFlat["current"].setValue( m_shmDmFlat );
    m_indiP_shmDmFlat["target"].setValue( m_shmDmFlat );
    m_indiP_shmDmSum["current"].setValue( m_shmDmSum );
    m_indiP_shmDmSum["target"].setValue( m_shmDmSum );
    m_indiP_shmCam["current"].setValue( m_shmCam );
    m_indiP_shmCam["target"].setValue( m_shmCam );
    m_indiP_camName["current"].setValue( m_camName );
    m_indiP_camName["target"].setValue( m_camName );
    m_indiP_flatDir["current"].setValue( m_flatDir );
    m_indiP_flatDir["target"].setValue( m_flatDir );
    m_indiP_darkLibPath["current"].setValue( m_darkLibPath );
    m_indiP_darkLibPath["target"].setValue( m_darkLibPath );
    m_indiP_modeStart["current"].setValue( m_modeStart );
    m_indiP_modeStart["target"].setValue( m_modeStart );
    m_indiP_modeEnd["current"].setValue( m_modeEnd );
    m_indiP_modeEnd["target"].setValue( m_modeEnd );
    m_indiP_focusModeIndex["current"].setValue( m_focusModeIndex );
    m_indiP_focusModeIndex["target"].setValue( m_focusModeIndex );
    m_indiP_coreRadius["current"].setValue( m_coreRadius );
    m_indiP_coreRadius["target"].setValue( m_coreRadius );
    m_indiP_searchRange["current"].setValue( m_searchRange );
    m_indiP_searchRange["target"].setValue( m_searchRange );
    m_indiP_searchStep["current"].setValue( m_searchStep );
    m_indiP_searchStep["target"].setValue( m_searchStep );
    m_indiP_nSteps["current"].setValue( m_nSteps );
    m_indiP_nSteps["target"].setValue( m_nSteps );
    m_indiP_nRepeats["current"].setValue( m_nRepeats );
    m_indiP_nRepeats["target"].setValue( m_nRepeats );
    m_indiP_nCluster["current"].setValue( m_nCluster );
    m_indiP_nCluster["target"].setValue( m_nCluster );
    m_indiP_nClusterRepeat["current"].setValue( m_nClusterRepeat );
    m_indiP_nClusterRepeat["target"].setValue( m_nClusterRepeat );
    m_indiP_nSeqRepeat["current"].setValue( m_nSeqRepeat );
    m_indiP_nSeqRepeat["target"].setValue( m_nSeqRepeat );
    m_indiP_nImages["current"].setValue( m_nImages );
    m_indiP_nImages["target"].setValue( m_nImages );
    m_indiP_skipFrames["current"].setValue( m_skipFrames );
    m_indiP_skipFrames["target"].setValue( m_skipFrames );
    m_indiP_cenX["current"].setValue( m_cenX );
    m_indiP_cenX["target"].setValue( m_cenX );
    m_indiP_cenY["current"].setValue( m_cenY );
    m_indiP_cenY["target"].setValue( m_cenY );
    m_indiP_satThresh["current"].setValue( m_satThresh );
    m_indiP_satThresh["target"].setValue( m_satThresh );
    m_indiP_blankThresh["current"].setValue( m_blankThresh );
    m_indiP_blankThresh["target"].setValue( m_blankThresh );
    m_indiP_exptimeTol["current"].setValue( m_exptimeTol );
    m_indiP_exptimeTol["target"].setValue( m_exptimeTol );
    m_indiP_dmDelay["current"].setValue( m_dmDelay );
    m_indiP_dmDelay["target"].setValue( m_dmDelay );
    m_indiP_ampTol["current"].setValue( m_ampTol );
    m_indiP_ampTol["target"].setValue( m_ampTol );
    m_indiP_ampTimeout["current"].setValue( m_ampTimeout );
    m_indiP_ampTimeout["target"].setValue( m_ampTimeout );
    m_indiP_searchKind["current"].setValue( m_searchKind );
    m_indiP_searchKind["target"].setValue( m_searchKind );

    updateSwitchIfChanged( m_indiP_baseline, "toggle", m_baseline ? pcf::IndiElement::On : pcf::IndiElement::Off,
                           m_baseline ? INDI_OK : INDI_IDLE );
    updateSwitchIfChanged( m_indiP_randomize, "toggle", m_randomize ? pcf::IndiElement::On : pcf::IndiElement::Off,
                           m_randomize ? INDI_OK : INDI_IDLE );
    updateSwitchIfChanged( m_indiP_ignoreFocus, "toggle", m_ignoreFocus ? pcf::IndiElement::On : pcf::IndiElement::Off,
                           m_ignoreFocus ? INDI_OK : INDI_IDLE );

    REG_INDI_SETPROP( m_indiP_remoteExptime, m_camName, "exptime" );
    REG_INDI_SETPROP( m_indiP_remoteFps, m_camName, "fps" );
    REG_INDI_SETPROP( m_indiP_remoteEmgain, m_camName, "emgain" );
    REG_INDI_SETPROP( m_indiP_remoteBlacklevel, m_camName, "blacklevel" );
    REG_INDI_SETPROP( m_indiP_remoteAmps, m_modesDevice, "current_amps" );

    m_worker = std::thread( workerStart, this );
    state( stateCodes::READY );
    log<text_log>( "eyeDoctor ready (modes_device=" + m_modesDevice + " shm_cam=" + m_shmCam +
                   " cam_name=" + m_camName + ")" );
    return 0;
}

int eyeDoctor::appLogic()
{
    if( m_busy.load() )
    {
        state( stateCodes::OPERATING );
    }
    else if( state() == stateCodes::OPERATING )
    {
        state( stateCodes::READY );
    }
    return 0;
}

int eyeDoctor::appShutdown()
{
    m_workerShutdown = true;
    m_runRequested = false;
    m_saveFlatRequested = false;
    m_abortRequested = false;
    try
    {
        if( m_worker.joinable() )
        {
            m_worker.join();
        }
    }
    catch( ... )
    {
    }
    m_hw.disconnect();
    return 0;
}

void eyeDoctor::workerStart( eyeDoctor *e )
{
    e->workerExec();
}

void eyeDoctor::workerExec()
{
    while( !m_workerShutdown.load() && shutdown() == 0 )
    {
        if( m_abortRequested.load() && !m_busy.load() )
        {
            m_busy = true;
            const int rv = abortAndZero();
            m_busy = false;
            m_abortRequested = false;
            m_runRequested = false;
            setRunToggle( false, pcf::IndiProperty::Idle );
            clearRequest( m_indiP_abort );
            if( rv == 0 )
            {
                setStatus( "aborted" );
            }
            else
            {
                setStatus( "abort error" );
            }
        }
        else if( m_resetRequested.load() && !m_busy.load() )
        {
            m_busy = true;
            const int rv = resetToZero();
            m_busy = false;
            m_resetRequested = false;
            m_runRequested = false;
            setRunToggle( false, pcf::IndiProperty::Idle );
            clearRequest( m_indiP_resetToZero );
            if( rv == 0 )
            {
                setStatus( "reset to zero" );
            }
            else
            {
                setStatus( "reset error" );
            }
        }
        else if( m_darkLibLoadRequested.load() && !m_busy.load() )
        {
            m_busy = true;
            const int rv = reloadDarkLib();
            m_busy = false;
            m_darkLibLoadRequested = false;
            clearRequest( m_indiP_darkLibLoad );
            if( rv != 0 && m_status.find( "reload_dark_lib" ) == std::string::npos )
            {
                setStatus( "reload_dark_lib: failed" );
            }
        }
        else if( m_saveFlatRequested.load() && !m_busy.load() )
        {
            m_busy = true;
            const int rv = saveFlat();
            m_busy = false;
            m_saveFlatRequested = false;
            clearRequest( m_indiP_saveFlat );
            if( rv == 0 )
            {
                setStatus( "flat saved" );
            }
            else
            {
                setStatus( "flat save error" );
            }
        }
        else if( m_runRequested.load() && !m_busy.load() && !m_abortRequested.load() &&
                 !m_resetRequested.load() )
        {
            m_busy = true;
            const int rv = runOptimization();
            m_busy = false;
            m_runRequested = false;
            setRunToggle( false, rv == 0 ? pcf::IndiProperty::Idle : pcf::IndiProperty::Alert );
            if( m_abortRequested.load() || m_resetRequested.load() )
            {
                continue;
            }
            if( rv == 0 )
            {
                setStatus( "completed" );
            }
            else if( rv == -2 )
            {
                setStatus( "stopped" );
            }
            else
            {
                setStatus( "error" );
            }
        }
        std::this_thread::sleep_for( std::chrono::milliseconds( 50 ) );
    }
}

bool eyeDoctor::stopping()
{
    return m_workerShutdown.load() || !m_runRequested.load() || m_abortRequested.load() ||
           m_resetRequested.load() || shutdown() != 0;
}

void eyeDoctor::setStatus( const std::string &s )
{
    m_status = s;
    updateIfChanged( m_indiP_status, "current", m_status );
    log<text_log>( "eyeDoctor: " + m_status );
}

void eyeDoctor::setRunToggle( bool on, pcf::IndiProperty::PropertyStateType st )
{
    updateSwitchIfChanged( m_indiP_run, "toggle", on ? pcf::IndiElement::On : pcf::IndiElement::Off, st );
}

void eyeDoctor::clearRequest( pcf::IndiProperty &p )
{
    updateSwitchIfChanged( p, "request", pcf::IndiElement::Off, INDI_IDLE );
}

int eyeDoctor::ensureDirectory( const std::string &path )
{
    if( path.empty() )
    {
        return -1;
    }
    if( mkdir( path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH ) < 0 && errno != EEXIST )
    {
        return -1;
    }
    return 0;
}

std::string eyeDoctor::timestampNow()
{
    time_t t = time( nullptr );
    struct tm tm{};
    localtime_r( &t, &tm );
    char buf[32];
    std::strftime( buf, sizeof( buf ), "%Y%m%d-%H%M%S", &tm );
    return buf;
}

int eyeDoctor::measureMetric( mx::improc::eigenImage<float> &im, double &metric )
{
    const int rv = m_hw.cam.grabMean(
        static_cast<unsigned>( std::max( 1, m_nImages ) ), static_cast<unsigned>( std::max( 0, m_skipFrames ) ),
        [this]() { return stopping(); }, im );
    if( rv < 0 )
    {
        return rv;
    }
    warnIfSaturated( im );
    applyDark( im );
    metric = dev::psfMetrics::coreSum( im, m_coreRadius, m_cenX, m_cenY );
    return 0;
}

void eyeDoctor::warnIfSaturated( const mx::improc::eigenImage<float> &im )
{
    if( !( m_satThresh > 0.0 ) || im.size() == 0 )
    {
        return;
    }
    const float peak = im.maxCoeff();
    if( peak < static_cast<float>( m_satThresh ) )
    {
        return;
    }
    if( m_satWarnedMode == m_currentMode )
    {
        return;
    }
    m_satWarnedMode = m_currentMode;
    log<text_log>( "saturation warning: mode " + std::to_string( m_currentMode ) + " peak=" +
                       std::to_string( peak ) + " >= sat_thresh " + std::to_string( m_satThresh ) + " ADU",
                   logPrio::LOG_WARNING );
}

std::string eyeDoctor::modeElementName( int mode )
{
    char buf[16];
    std::snprintf( buf, sizeof( buf ), "%04d", mode );
    return buf;
}

int eyeDoctor::nRemoteModes()
{
    std::lock_guard<std::mutex> lock( m_modesMutex );
    return static_cast<int>( m_remoteAmps.size() );
}

double eyeDoctor::currentAmp( int mode )
{
    std::lock_guard<std::mutex> lock( m_modesMutex );
    if( mode < 0 || mode >= static_cast<int>( m_remoteAmps.size() ) )
    {
        return 0.0;
    }
    return m_remoteAmps[static_cast<size_t>( mode )];
}

int eyeDoctor::waitForModesDevice()
{
    if( m_modesDevice.empty() )
    {
        setStatus( "modes_device is empty" );
        return -1;
    }
    const auto t0 = std::chrono::steady_clock::now();
    const double timeout = std::max( 0.1, m_ampTimeout );
    while( !stopping() )
    {
        if( nRemoteModes() > 0 )
        {
            return 0;
        }
        const double elapsed =
            std::chrono::duration<double>( std::chrono::steady_clock::now() - t0 ).count();
        if( elapsed > timeout )
        {
            setStatus( "no current_amps from " + m_modesDevice );
            log<text_log>( "waiting for " + m_modesDevice + ".current_amps timed out", logPrio::LOG_ERROR );
            return -1;
        }
        mx::sys::milliSleep( 20 );
    }
    return -2;
}

int eyeDoctor::sendModeAmp( int mode, double amp )
{
    if( dev::doubleBitsNonFinite( amp ) )
    {
        amp = 0.0;
    }
    const std::string el = modeElementName( mode );
    pcf::IndiProperty ip( pcf::IndiProperty::Number );
    ip.setDevice( m_modesDevice );
    ip.setName( "target_amps" );
    ip.add( pcf::IndiElement( el ) );
    ip[el] = amp;
    if( sendNewProperty( ip ) < 0 )
    {
        log<software_error>( { __FILE__, __LINE__, "sendNewProperty " + m_modesDevice + ".target_amps." + el } );
        return -1;
    }
    return 0;
}

int eyeDoctor::waitModeAmp( int mode, double amp )
{
    const auto t0 = std::chrono::steady_clock::now();
    const double timeout = std::max( 0.1, m_ampTimeout );
    while( !stopping() )
    {
        if( std::fabs( currentAmp( mode ) - amp ) <= m_ampTol )
        {
            return 0;
        }
        const double elapsed =
            std::chrono::duration<double>( std::chrono::steady_clock::now() - t0 ).count();
        if( elapsed > timeout )
        {
            log<text_log>( "timeout waiting for " + m_modesDevice + ".current_amps." + modeElementName( mode ) +
                               " = " + std::to_string( amp ),
                           logPrio::LOG_WARNING );
            return -1;
        }
        mx::sys::milliSleep( 5 );
    }
    return -2;
}

int eyeDoctor::sendModeAndWait( int mode, double amp )
{
    for( int attempt = 0; attempt < 3; ++attempt )
    {
        if( stopping() )
        {
            return -2;
        }
        if( sendModeAmp( mode, amp ) < 0 )
        {
            return -1;
        }
        const int wrv = waitModeAmp( mode, amp );
        if( wrv == 0 )
        {
            if( m_dmDelay > 0 )
            {
                mx::sys::milliSleep( static_cast<unsigned>( m_dmDelay * 1000.0 ) );
            }
            return 0;
        }
        if( wrv == -2 )
        {
            return -2;
        }
    }
    return -1;
}

int eyeDoctor::zeroAllModes()
{
    const int n = nRemoteModes();
    if( n <= 0 )
    {
        setStatus( "modes device has not published current_amps" );
        return -1;
    }

    pcf::IndiProperty ip( pcf::IndiProperty::Number );
    ip.setDevice( m_modesDevice );
    ip.setName( "target_amps" );
    for( int i = 0; i < n; ++i )
    {
        const std::string el = modeElementName( i );
        ip.add( pcf::IndiElement( el ) );
        ip[el] = 0.0;
    }
    if( sendNewProperty( ip ) < 0 )
    {
        setStatus( "failed to send target_amps=0 to " + m_modesDevice );
        return -1;
    }

    const auto t0 = std::chrono::steady_clock::now();
    const double timeout = std::max( 0.1, m_ampTimeout );
    while( !stopping() )
    {
        bool ok = true;
        {
            std::lock_guard<std::mutex> lock( m_modesMutex );
            if( static_cast<int>( m_remoteAmps.size() ) < n )
            {
                ok = false;
            }
            else
            {
                for( int i = 0; i < n; ++i )
                {
                    if( std::fabs( m_remoteAmps[static_cast<size_t>( i )] ) > m_ampTol )
                    {
                        ok = false;
                        break;
                    }
                }
            }
        }
        if( ok )
        {
            m_currentMode = -1;
            updateIfChanged( m_indiP_currentMode, "current", -1.0 );
            return 0;
        }
        const double elapsed =
            std::chrono::duration<double>( std::chrono::steady_clock::now() - t0 ).count();
        if( elapsed > timeout )
        {
            setStatus( "timeout waiting for " + m_modesDevice + " current_amps to reach 0" );
            return -1;
        }
        mx::sys::milliSleep( 5 );
    }
    return -2;
}

int eyeDoctor::abortAndZero()
{
    setStatus( "aborting" );
    if( waitForModesDevice() < 0 )
    {
        return -1;
    }
    if( zeroAllModes() < 0 )
    {
        return -1;
    }
    log<text_log>( "aborted: zeroed " + m_modesDevice + " target_amps" );
    return 0;
}

int eyeDoctor::resetToZero()
{
    setStatus( "resetting to zero" );
    if( waitForModesDevice() < 0 )
    {
        return -1;
    }
    if( zeroAllModes() < 0 )
    {
        return -1;
    }
    log<text_log>( "reset_to_zero: wrote 0 to " + m_modesDevice + ".target_amps" );
    return 0;
}

std::vector<int> eyeDoctor::requestedModes() const
{
    std::vector<int> modes;
    if( m_modeEnd < m_modeStart )
    {
        return modes;
    }
    for( int i = m_modeStart; i <= m_modeEnd; ++i )
    {
        modes.push_back( i );
    }
    return modes;
}

int eyeDoctor::parseSearchKind( const std::string &in, std::string &searchKind, std::string &gridKind,
                                std::string *err )
{
    std::string s = in;
    auto a = s.find_first_not_of( " \t" );
    auto b = s.find_last_not_of( " \t" );
    if( a == std::string::npos )
    {
        if( err )
        {
            *err = "empty search_kind";
        }
        return -1;
    }
    s = s.substr( a, b - a + 1 );
    for( char &c : s )
    {
        if( c >= 'A' && c <= 'Z' )
        {
            c = static_cast<char>( c - 'A' + 'a' );
        }
    }

    // magpyx eye_doctor_comprehensive / dm_eye_doctor
    if( s == "grid" )
    {
        searchKind = "grid";
        gridKind = "fit";
        return 0;
    }
    if( s == "brent" )
    {
        searchKind = "brent";
        gridKind = "fit";
        return 0;
    }
    // Old names were grid_sweep's skind, not magpyx search_kind.
    if( s == "fit" )
    {
        searchKind = "grid";
        gridKind = "fit";
        return 0;
    }
    if( s == "mean" )
    {
        searchKind = "grid";
        gridKind = "mean";
        return 0;
    }
    if( err )
    {
        *err = "search_kind must be grid or brent";
    }
    return -1;
}

std::vector<int> eyeDoctor::allowedModes( const std::vector<int> &req, int nAvail, bool *truncated ) const
{
    std::vector<int> out;
    bool trunc = false;
    for( int m : req )
    {
        if( m >= 0 && m < nAvail )
        {
            out.push_back( m );
        }
        else
        {
            trunc = true;
        }
    }
    if( truncated )
    {
        *truncated = trunc;
    }
    return out;
}

std::vector<int> eyeDoctor::buildSequence( const std::vector<int> &modes ) const
{
    std::vector<int> seq;
    if( modes.empty() )
    {
        return seq;
    }
    const int ncl = std::max( 1, m_nCluster );
    const int ncr = std::max( 1, m_nClusterRepeat );
    const int nsr = std::max( 1, m_nSeqRepeat );
    static thread_local std::mt19937 rng{ std::random_device{}() };

    for( int s = 0; s < nsr; ++s )
    {
        for( size_t i = 0; i < modes.size(); i += static_cast<size_t>( ncl ) )
        {
            std::vector<int> cluster;
            const size_t j1 = std::min( i + static_cast<size_t>( ncl ), modes.size() );
            cluster.assign( modes.begin() + static_cast<std::ptrdiff_t>( i ),
                            modes.begin() + static_cast<std::ptrdiff_t>( j1 ) );
            for( int r = 0; r < ncr; ++r )
            {
                std::vector<int> cur = cluster;
                if( m_randomize && cur.size() > 1 )
                {
                    std::shuffle( cur.begin(), cur.end(), rng );
                }
                seq.insert( seq.end(), cur.begin(), cur.end() );
            }
        }
    }
    return seq;
}

lina::DarkMatchFilter eyeDoctor::darkFilter() const
{
    lina::DarkMatchFilter f;
    f.shm_cam_input = m_shmCam;
    if( m_hw.cam.isOpen() )
    {
        f.width = m_hw.cam.size0();
        f.height = m_hw.cam.size1();
    }
    if( std::isfinite( m_remoteGain ) )
    {
        f.gain = m_remoteGain;
    }
    if( std::isfinite( m_remoteBlacklevel ) )
    {
        f.blacklevel = m_remoteBlacklevel;
    }
    return f;
}

std::string eyeDoctor::formatDarkEntry( const lina::DarkLibraryEntry &e ) const
{
    std::ostringstream ss;
    ss << std::setprecision( 17 );
    ss << ( e.relpath.empty() ? "-" : e.relpath ) << " exptime=";
    if( std::isfinite( e.exptime ) )
    {
        ss << e.exptime;
    }
    else
    {
        ss << "nan";
    }
    ss << " shm_cam_input=" << ( e.shm_cam_input.empty() ? "-" : e.shm_cam_input ) << " emgain=";
    if( std::isfinite( e.gain ) )
    {
        ss << e.gain;
    }
    else
    {
        ss << "nan";
    }
    ss << " blacklevel=";
    if( std::isfinite( e.blacklevel ) )
    {
        ss << e.blacklevel;
    }
    else
    {
        ss << "nan";
    }
    if( e.width > 0 && e.height > 0 )
    {
        ss << " " << e.width << "x" << e.height;
    }
    return ss.str();
}

std::string eyeDoctor::pickDark( double target_exptime, const lina::DarkMatchFilter &filter,
                                  lina::DarkLibraryEntry *matched, double *match_err )
{
    const auto all = lina::load_dark_library_manifest( m_darkLibPath );
    const auto entries = lina::filter_dark_library_entries( all, filter );
    if( entries.empty() )
    {
        return {};
    }

    std::size_t best = 0;
    double best_err = std::numeric_limits<double>::infinity();
    bool found = false;
    for( std::size_t i = 0; i < entries.size(); ++i )
    {
        if( !std::isfinite( entries[i].exptime ) )
        {
            continue;
        }
        if( !std::isfinite( target_exptime ) )
        {
            best = i;
            best_err = 0;
            found = true;
            break;
        }
        const double err = std::fabs( entries[i].exptime - target_exptime );
        if( err < best_err )
        {
            best_err = err;
            best = i;
            found = true;
        }
    }
    if( !found )
    {
        return {};
    }
    if( std::isfinite( target_exptime ) && best_err > m_exptimeTol )
    {
        if( matched )
        {
            *matched = entries[best];
        }
        if( match_err )
        {
            *match_err = best_err;
        }
        return {};
    }
    if( matched )
    {
        *matched = entries[best];
    }
    if( match_err )
    {
        *match_err = best_err;
    }
    const std::string &rel = entries[best].relpath;
    if( !rel.empty() && rel[0] == '/' )
    {
        return rel;
    }
    if( m_darkLibPath.empty() )
    {
        return rel;
    }
    if( m_darkLibPath.back() == '/' )
    {
        return m_darkLibPath + rel;
    }
    return m_darkLibPath + "/" + rel;
}

int eyeDoctor::refreshDark( bool required )
{
    m_haveDark = false;
    m_lastDarkPath.clear();
    m_darkExptime = std::numeric_limits<double>::quiet_NaN();
    m_darkGain = std::numeric_limits<double>::quiet_NaN();
    m_darkBlacklevel = std::numeric_limits<double>::quiet_NaN();
    m_darkMatchErr = std::numeric_limits<double>::quiet_NaN();
    updateIfChanged( m_indiP_lastDark, "current", m_lastDarkPath );

    if( m_darkLibPath.empty() )
    {
        if( required )
        {
            setStatus( "reload_dark_lib: failed" );
            return log<software_error, -1>( { __FILE__, __LINE__, "dark_lib_path is empty" } );
        }
        return 0;
    }

    const auto all = lina::load_dark_library_manifest( m_darkLibPath );
    if( all.empty() )
    {
        const std::string msg = "no dark_metadata.txt entries in " + m_darkLibPath;
        if( required )
        {
            setStatus( "reload_dark_lib: failed" );
            return log<software_error, -1>( { __FILE__, __LINE__, msg } );
        }
        log<text_log>( msg, logPrio::LOG_WARNING );
        return 0;
    }

    const auto filt = lina::filter_dark_library_entries( all, darkFilter() );
    if( filt.empty() )
    {
        std::ostringstream ss;
        ss << std::setprecision( 17 );
        ss << "no darks matching shm_cam=" << m_shmCam << " emgain=";
        if( std::isfinite( m_remoteGain ) )
        {
            ss << m_remoteGain;
        }
        else
        {
            ss << "nan";
        }
        ss << " blacklevel=";
        if( std::isfinite( m_remoteBlacklevel ) )
        {
            ss << m_remoteBlacklevel;
        }
        else
        {
            ss << "nan";
        }
        ss << " in " << m_darkLibPath << " (entries=" << all.size() << ")";
        if( required )
        {
            setStatus( "reload_dark_lib: failed" );
            return log<software_error, -1>( { __FILE__, __LINE__, ss.str() } );
        }
        log<text_log>( ss.str(), logPrio::LOG_WARNING );
        return 0;
    }

    const double target_exptime = m_remoteExp;
    lina::DarkLibraryEntry matched;
    double match_err = std::numeric_limits<double>::quiet_NaN();
    const std::string path = pickDark( target_exptime, darkFilter(), &matched, &match_err );
    if( path.empty() )
    {
        std::ostringstream ss;
        ss << std::setprecision( 17 );
        ss << "no dark within exptime_tol=" << m_exptimeTol << " s of live exptime=";
        if( std::isfinite( target_exptime ) )
        {
            ss << target_exptime;
        }
        else
        {
            ss << "nan";
        }
        if( std::isfinite( match_err ) )
        {
            ss << " (nearest err=" << match_err << " s, " << formatDarkEntry( matched ) << ")";
        }
        if( required )
        {
            setStatus( "reload_dark_lib: failed" );
            return log<software_error, -1>( { __FILE__, __LINE__, ss.str() } );
        }
        log<text_log>( ss.str(), logPrio::LOG_WARNING );
        return 0;
    }

    mx::improc::eigenImage<float> dark;
    if( dev::readFitsImage( path, dark ) < 0 )
    {
        const std::string msg = "failed to read dark FITS " + path;
        if( required )
        {
            setStatus( "reload_dark_lib: failed" );
            return log<software_error, -1>( { __FILE__, __LINE__, msg } );
        }
        log<text_log>( msg, logPrio::LOG_WARNING );
        return 0;
    }
    dev::replaceNonFinite( dark );

    m_dark = dark;
    m_haveDark = true;
    m_lastDarkPath = path;
    m_darkExptime = matched.exptime;
    m_darkGain = matched.gain;
    m_darkBlacklevel = matched.blacklevel;
    m_darkMatchErr = match_err;
    updateIfChanged( m_indiP_lastDark, "current", m_lastDarkPath );

    std::ostringstream oss;
    oss << "dark library match: using " << path << " (" << formatDarkEntry( matched );
    if( std::isfinite( match_err ) )
    {
        oss << ", |err|=" << match_err << " s";
    }
    oss << ")";
    log<text_log>( oss.str() );
    return 0;
}

int eyeDoctor::reloadDarkLib()
{
    setStatus( "reload_dark_lib: starting" );
    if( refreshDark( true ) < 0 )
    {
        return -1;
    }
    setStatus( "reload_dark_lib: done (" + m_lastDarkPath + ")" );
    return 0;
}

void eyeDoctor::applyDark( mx::improc::eigenImage<float> &im )
{
    if( !m_haveDark || im.size() == 0 )
    {
        return;
    }
    if( im.rows() != m_dark.rows() || im.cols() != m_dark.cols() )
    {
        log<text_log>( "dark size " + std::to_string( m_dark.rows() ) + "x" +
                           std::to_string( m_dark.cols() ) + " != camera " +
                           std::to_string( im.rows() ) + "x" + std::to_string( im.cols() ) +
                           "; skipping dark subtraction",
                       logPrio::LOG_WARNING );
        m_haveDark = false;
        return;
    }
    im -= m_dark;
}

int eyeDoctor::saveFlat()
{
    setStatus( "saving flat" );

    if( m_flatDir.empty() )
    {
        setStatus( "flat_dir is empty" );
        return -1;
    }

    m_hw.dmFlatName = m_shmDmFlat;
    m_hw.dmSumName = m_shmDmSum;
    m_hw.disconnect();
    if( m_hw.connectFlatSave() < 0 )
    {
        log<software_error>( { __FILE__, __LINE__, m_hw.error() } );
        setStatus( "connect failed: " + m_hw.error() );
        return -1;
    }

    mx::improc::eigenImage<float> total;
    if( m_hw.dmSum.grabLatest( total ) < 0 )
    {
        setStatus( "failed to read shm_dm_sum" );
        return -1;
    }
    const int nbad = dev::replaceNonFinite( total );
    if( nbad > 0 )
    {
        log<text_log>( "shm_dm_sum had " + std::to_string( nbad ) +
                           " NaN/Inf pixels; writing 0 in those pixels",
                       logPrio::LOG_WARNING );
    }

    if( m_hw.dmFlat.write( total ) < 0 )
    {
        setStatus( "failed to write shm_dm_flat" );
        return -1;
    }

    if( waitForModesDevice() == 0 )
    {
        if( zeroAllModes() < 0 )
        {
            log<text_log>( "saved flat but failed to zero " + m_modesDevice, logPrio::LOG_WARNING );
        }
    }

    if( ensureDirectory( m_flatDir ) < 0 )
    {
        setStatus( "cannot create flat_dir" );
        return -1;
    }

    std::string path = m_flatDir;
    if( path.back() != '/' )
    {
        path += '/';
    }
    path += "flat_eyedoctor_" + timestampNow() + ".fits";

    if( dev::writeFitsImage( path, total ) < 0 )
    {
        setStatus( "failed to write " + path );
        return -1;
    }

    m_lastFlatPath = path;
    updateIfChanged( m_indiP_lastFlat, "current", m_lastFlatPath );
    log<text_log>( "saved flat to " + path + " and wrote " + m_shmDmFlat + " from " + m_shmDmSum );
    return 0;
}

int eyeDoctor::optimizeMode( int mi, mx::improc::eigenImage<float> &camIm )
{
    m_currentMode = mi;
    m_satWarnedMode = -2;
    updateIfChanged( m_indiP_currentMode, "current", static_cast<double>( mi ) );

    const double baseval = m_baseline ? currentAmp( mi ) : 0.0;
    const double lo = -0.5 * m_searchRange;
    const double hi = 0.5 * m_searchRange;
    log<text_log>( "Mode " + std::to_string( mi ) + ": scanning " + std::to_string( lo + baseval ) + " to " +
                   std::to_string( hi + baseval ) + " (baseline " + std::to_string( baseval ) + ", search_kind=" +
                   m_searchKind + ")" );

    double metric0 = 0;
    if( measureMetric( camIm, metric0 ) < 0 )
    {
        if( stopping() )
        {
            return -2;
        }
        setStatus( "camera grab failed" );
        return -1;
    }

    auto applyAmp = [&]( double a ) -> int {
        const int rv = sendModeAndWait( mi, baseval + a );
        if( rv < 0 )
        {
            return rv;
        }
        return 0;
    };
    auto measure = [&]() -> dev::metricSample {
        double m = 0;
        if( measureMetric( camIm, m ) < 0 )
        {
            return { 1e6, 0.0 };
        }
        const double peak = camIm.size() > 0 ? static_cast<double>( camIm.maxCoeff() ) : 0.0;
        return { m, peak };
    };
    auto stopFn = [this]() { return stopping(); };

    double deltaAmp = 0;
    if( m_searchKind == "brent" )
    {
        dev::brentSweep sweep;
        sweep.lo = lo;
        sweep.hi = hi;
        const auto sw = sweep.run( applyAmp, measure, stopFn );
        if( stopping() || sw.stopped )
        {
            sendModeAndWait( mi, baseval );
            return -2;
        }
        if( sw.failed )
        {
            log<text_log>( "mode " + std::to_string( mi ) + ": brent search failed, leaving amp=" +
                               std::to_string( baseval ),
                           logPrio::LOG_WARNING );
            deltaAmp = 0;
        }
        else
        {
            deltaAmp = dev::finiteOrZero( sw.amp );
            log<text_log>( "mode " + std::to_string( mi ) + ": brent search nEval=" + std::to_string( sw.nEval ) );
        }
    }
    else
    {
        dev::gridSweep sweep;
        sweep.lo = lo;
        sweep.hi = hi;
        if( m_searchStep > 0.0 && m_searchRange > 0.0 )
        {
            sweep.nSteps = std::max( 3, static_cast<int>( std::lround( m_searchRange / m_searchStep ) ) + 1 );
        }
        else
        {
            sweep.nSteps = std::max( 3, m_nSteps );
        }
        sweep.nRepeats = std::max( 1, m_nRepeats );
        sweep.kind = m_gridKind;
        sweep.blankThresh = m_blankThresh;

        const auto sw = sweep.run( applyAmp, measure, stopFn );

        if( stopping() || sw.stopped )
        {
            sendModeAndWait( mi, baseval );
            return -2;
        }

        deltaAmp = dev::finiteOrZero( sw.amp );
        if( m_gridKind == "fit" && !sw.usedFit )
        {
            std::string why = "mode " + std::to_string( mi ) + ": quadratic fit rejected";
            if( sw.truncated )
            {
                why += ", truncated to " + std::to_string( sw.nGood ) + "/" + std::to_string( sw.nTotal ) +
                       " on-camera samples";
            }
            if( sw.refined )
            {
                why += ", refined around best sample";
            }
            if( deltaAmp == 0.0 )
            {
                why += ", leaving amp=" + std::to_string( baseval );
            }
            else
            {
                why += ", using best-sample amp=" + std::to_string( baseval + deltaAmp );
            }
            log<text_log>( why, logPrio::LOG_WARNING );
        }
        else if( m_gridKind == "fit" && ( sw.truncated || sw.refined ) )
        {
            log<text_log>( "mode " + std::to_string( mi ) + ": quadratic on " + std::to_string( sw.nGood ) + "/" +
                               std::to_string( sw.nTotal ) + " on-camera samples" +
                               ( sw.refined ? " after refine" : "" ),
                           logPrio::LOG_INFO );
        }
    }

    const double useAmp = baseval + deltaAmp;
    if( sendModeAndWait( mi, useAmp ) < 0 )
    {
        setStatus( "failed to command " + m_modesDevice + " mode " + std::to_string( mi ) );
        return -1;
    }

    double metric1 = 0;
    measureMetric( camIm, metric1 );
    m_lastAmp = useAmp;
    m_lastMetric = dev::finiteOrZero( metric1 );
    updateIfChanged( m_indiP_optAmp, "current", m_lastAmp );
    updateIfChanged( m_indiP_metric, "current", m_lastMetric );
    log<text_log>( "mode " + std::to_string( mi ) + " amp " + std::to_string( baseval ) + " -> " +
                   std::to_string( useAmp ) + " metric " + std::to_string( metric0 ) + " -> " +
                   std::to_string( metric1 ) );
    return 0;
}

int eyeDoctor::runOptimization()
{
    setStatus( "connecting" );

    m_hw.camName = m_shmCam;
    m_hw.camDevice = m_camName;
    m_hw.disconnect();
    if( m_hw.connectCamera() < 0 )
    {
        log<software_error>( { __FILE__, __LINE__, m_hw.error() } );
        setStatus( "connect failed: " + m_hw.error() );
        return -1;
    }
    if( stopping() )
    {
        return -2;
    }

    setStatus( "waiting for " + m_modesDevice );
    if( waitForModesDevice() < 0 )
    {
        return -1;
    }

    const int nAvail = nRemoteModes();
    m_modesMax = nAvail;
    updateIfChanged( m_indiP_modesMax, "current", static_cast<double>( m_modesMax ) );
    log<text_log>( "modes_max on " + m_modesDevice + ": " + std::to_string( nAvail ) );

    bool truncated = false;
    const std::vector<int> allowed = allowedModes( requestedModes(), nAvail, &truncated );
    if( truncated )
    {
        log<text_log>( "mode_start/mode_end includes indices outside 0.." + std::to_string( nAvail - 1 ) +
                           " (modes_max=" + std::to_string( nAvail ) + " on " + m_modesDevice +
                           "); skipping those modes",
                       logPrio::LOG_WARNING );
    }
    if( allowed.empty() )
    {
        setStatus( "no requested modes within " + m_modesDevice + " bounds" );
        return -1;
    }

    if( !m_baseline )
    {
        log<text_log>( "baseline off: resetting all mode coefficients to 0" );
        if( zeroAllModes() < 0 )
        {
            return -1;
        }
    }

    std::ostringstream ms;
    ms << "Optimizing " << allowed.size() << " mode" << ( allowed.size() == 1 ? "" : "s" ) << ":";
    for( int m : allowed )
    {
        ms << " " << m;
    }
    log<text_log>( ms.str() );

    if( !m_darkLibPath.empty() )
    {
        refreshDark( false );
    }

    mx::improc::eigenImage<float> camIm;

    const bool focusFirst = !m_ignoreFocus && allowed.size() > 1;
    if( focusFirst )
    {
        bool haveFocus = false;
        for( int m : allowed )
        {
            if( m == m_focusModeIndex )
            {
                haveFocus = true;
                break;
            }
        }
        if( haveFocus )
        {
            log<text_log>( "Optimizing focus first (mode " + std::to_string( m_focusModeIndex ) + ")" );
            setStatus( "mode " + std::to_string( m_focusModeIndex ) + " (focus first)" );
            const int frv = optimizeMode( m_focusModeIndex, camIm );
            if( frv < 0 )
            {
                return frv;
            }
        }
    }

    const std::vector<int> seq = buildSequence( allowed );
    for( size_t i = 0; i < seq.size() && !stopping(); ++i )
    {
        const int mi = seq[i];
        setStatus( "mode " + std::to_string( mi ) + " (" + std::to_string( i + 1 ) + "/" +
                   std::to_string( seq.size() ) + ")" );
        const int rv = optimizeMode( mi, camIm );
        if( rv < 0 )
        {
            return rv;
        }
    }

    m_currentMode = -1;
    updateIfChanged( m_indiP_currentMode, "current", -1.0 );
    return stopping() ? -2 : 0;
}

// ---------- INDI NEW ----------

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_modesDevice )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_modesDevice, ipRecv );
    std::string target;
    if( indiTargetUpdate( m_indiP_modesDevice, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    log<text_log>( "modes_device: " + m_modesDevice + " -> " + target +
                       " (current_amps SET still bound to startup device; restart to resubscribe)",
                   logPrio::LOG_WARNING );
    m_modesDevice = target;
    updateIfChanged( m_indiP_modesDevice, "current", m_modesDevice );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_shmDmFlat )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_shmDmFlat, ipRecv );
    std::string target;
    if( indiTargetUpdate( m_indiP_shmDmFlat, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_shmDmFlat = target;
    updateIfChanged( m_indiP_shmDmFlat, "current", m_shmDmFlat );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_shmDmSum )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_shmDmSum, ipRecv );
    std::string target;
    if( indiTargetUpdate( m_indiP_shmDmSum, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_shmDmSum = target;
    updateIfChanged( m_indiP_shmDmSum, "current", m_shmDmSum );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_shmCam )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_shmCam, ipRecv );
    std::string target;
    if( indiTargetUpdate( m_indiP_shmCam, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_shmCam = target;
    updateIfChanged( m_indiP_shmCam, "current", m_shmCam );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_camName )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_camName, ipRecv );
    std::string target;
    if( indiTargetUpdate( m_indiP_camName, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    log<text_log>( "cam_name: " + m_camName + " -> " + target + " (exptime/fps SET still bound to startup device)",
                   logPrio::LOG_WARNING );
    m_camName = target;
    updateIfChanged( m_indiP_camName, "current", m_camName );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_flatDir )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_flatDir, ipRecv );
    std::string target;
    if( indiTargetUpdate( m_indiP_flatDir, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_flatDir = target;
    updateIfChanged( m_indiP_flatDir, "current", m_flatDir );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_darkLibPath )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_darkLibPath, ipRecv );
    std::string target;
    if( indiTargetUpdate( m_indiP_darkLibPath, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    if( target != m_darkLibPath )
    {
        log<text_log>( "dark_lib_path: " + m_darkLibPath + " -> " + target );
        m_haveDark = false;
        m_lastDarkPath.clear();
        updateIfChanged( m_indiP_lastDark, "current", m_lastDarkPath );
    }
    m_darkLibPath = target;
    updateIfChanged( m_indiP_darkLibPath, "current", m_darkLibPath );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_modeStart )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_modeStart, ipRecv );
    int target = 0;
    if( indiTargetUpdate( m_indiP_modeStart, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_modeStart = target;
    updateIfChanged( m_indiP_modeStart, "current", m_modeStart );
    if( m_modesMax > 0 && ( m_modeStart < 0 || m_modeStart >= m_modesMax ) )
    {
        log<text_log>( "mode_start=" + std::to_string( m_modeStart ) + " is outside 0.." +
                           std::to_string( m_modesMax - 1 ) + " (modes_max from " + m_modesDevice + ")",
                       logPrio::LOG_WARNING );
    }
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_modeEnd )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_modeEnd, ipRecv );
    int target = 0;
    if( indiTargetUpdate( m_indiP_modeEnd, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_modeEnd = target;
    updateIfChanged( m_indiP_modeEnd, "current", m_modeEnd );
    if( m_modesMax > 0 && ( m_modeEnd < 0 || m_modeEnd >= m_modesMax ) )
    {
        log<text_log>( "mode_end=" + std::to_string( m_modeEnd ) + " is outside 0.." +
                           std::to_string( m_modesMax - 1 ) + " (modes_max from " + m_modesDevice + ")",
                       logPrio::LOG_WARNING );
    }
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_focusModeIndex )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_focusModeIndex, ipRecv );
    int target = 0;
    if( indiTargetUpdate( m_indiP_focusModeIndex, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_focusModeIndex = target;
    updateIfChanged( m_indiP_focusModeIndex, "current", m_focusModeIndex );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_coreRadius )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_coreRadius, ipRecv );
    float target = 0;
    if( indiTargetUpdate( m_indiP_coreRadius, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_coreRadius = target;
    updateIfChanged( m_indiP_coreRadius, "current", m_coreRadius );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_searchRange )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_searchRange, ipRecv );
    float target = 0;
    if( indiTargetUpdate( m_indiP_searchRange, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_searchRange = target;
    updateIfChanged( m_indiP_searchRange, "current", m_searchRange );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_searchStep )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_searchStep, ipRecv );
    float target = 0;
    if( indiTargetUpdate( m_indiP_searchStep, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_searchStep = target;
    updateIfChanged( m_indiP_searchStep, "current", m_searchStep );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_nSteps )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_nSteps, ipRecv );
    int target = 0;
    if( indiTargetUpdate( m_indiP_nSteps, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_nSteps = target;
    updateIfChanged( m_indiP_nSteps, "current", m_nSteps );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_nRepeats )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_nRepeats, ipRecv );
    int target = 0;
    if( indiTargetUpdate( m_indiP_nRepeats, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_nRepeats = target;
    updateIfChanged( m_indiP_nRepeats, "current", m_nRepeats );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_nCluster )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_nCluster, ipRecv );
    int target = 0;
    if( indiTargetUpdate( m_indiP_nCluster, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_nCluster = target;
    updateIfChanged( m_indiP_nCluster, "current", m_nCluster );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_nClusterRepeat )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_nClusterRepeat, ipRecv );
    int target = 0;
    if( indiTargetUpdate( m_indiP_nClusterRepeat, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_nClusterRepeat = target;
    updateIfChanged( m_indiP_nClusterRepeat, "current", m_nClusterRepeat );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_nSeqRepeat )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_nSeqRepeat, ipRecv );
    int target = 0;
    if( indiTargetUpdate( m_indiP_nSeqRepeat, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_nSeqRepeat = target;
    updateIfChanged( m_indiP_nSeqRepeat, "current", m_nSeqRepeat );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_nImages )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_nImages, ipRecv );
    int target = 0;
    if( indiTargetUpdate( m_indiP_nImages, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_nImages = target;
    updateIfChanged( m_indiP_nImages, "current", m_nImages );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_skipFrames )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_skipFrames, ipRecv );
    int target = 0;
    if( indiTargetUpdate( m_indiP_skipFrames, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_skipFrames = target;
    updateIfChanged( m_indiP_skipFrames, "current", m_skipFrames );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_cenX )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_cenX, ipRecv );
    float target = 0;
    if( indiTargetUpdate( m_indiP_cenX, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_cenX = target;
    updateIfChanged( m_indiP_cenX, "current", m_cenX );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_cenY )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_cenY, ipRecv );
    float target = 0;
    if( indiTargetUpdate( m_indiP_cenY, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_cenY = target;
    updateIfChanged( m_indiP_cenY, "current", m_cenY );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_satThresh )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_satThresh, ipRecv );
    float target = 0;
    if( indiTargetUpdate( m_indiP_satThresh, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_satThresh = target;
    updateIfChanged( m_indiP_satThresh, "current", m_satThresh );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_blankThresh )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_blankThresh, ipRecv );
    float target = 0;
    if( indiTargetUpdate( m_indiP_blankThresh, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_blankThresh = target;
    updateIfChanged( m_indiP_blankThresh, "current", m_blankThresh );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_exptimeTol )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_exptimeTol, ipRecv );
    float target = 0;
    if( indiTargetUpdate( m_indiP_exptimeTol, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_exptimeTol = target;
    updateIfChanged( m_indiP_exptimeTol, "current", m_exptimeTol );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_dmDelay )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_dmDelay, ipRecv );
    float target = 0;
    if( indiTargetUpdate( m_indiP_dmDelay, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_dmDelay = target;
    updateIfChanged( m_indiP_dmDelay, "current", m_dmDelay );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_ampTol )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_ampTol, ipRecv );
    float target = 0;
    if( indiTargetUpdate( m_indiP_ampTol, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_ampTol = target;
    updateIfChanged( m_indiP_ampTol, "current", m_ampTol );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_ampTimeout )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_ampTimeout, ipRecv );
    float target = 0;
    if( indiTargetUpdate( m_indiP_ampTimeout, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_ampTimeout = target;
    updateIfChanged( m_indiP_ampTimeout, "current", m_ampTimeout );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_searchKind )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_searchKind, ipRecv );
    std::string target;
    if( indiTargetUpdate( m_indiP_searchKind, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    std::string kind;
    std::string gkind;
    std::string err;
    if( parseSearchKind( target, kind, gkind, &err ) < 0 )
    {
        log<text_log>( err, logPrio::LOG_ERROR );
        return -1;
    }
    if( target == "fit" || target == "mean" )
    {
        log<text_log>( "search_kind=" + target + " is magpyx grid_sweep skind, not eye_doctor search_kind; "
                       "using search_kind=grid with that quadratic/mean extractor",
                       logPrio::LOG_NOTICE );
    }
    m_searchKind = kind;
    m_gridKind = gkind;
    updateIfChanged( m_indiP_searchKind, "current", m_searchKind );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_resetToZero )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_resetToZero, ipRecv );
    if( !ipRecv.find( "request" ) )
    {
        return -1;
    }
    if( ipRecv["request"].getSwitchState() == pcf::IndiElement::On )
    {
        m_runRequested = false;
        m_resetRequested = true;
        setRunToggle( false, pcf::IndiProperty::Idle );
        updateSwitchIfChanged( m_indiP_resetToZero, "request", pcf::IndiElement::On, INDI_BUSY );
    }
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_baseline )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_baseline, ipRecv );
    if( !ipRecv.find( "toggle" ) )
    {
        return 0;
    }
    m_baseline = ( ipRecv["toggle"].getSwitchState() == pcf::IndiElement::On );
    updateSwitchIfChanged( m_indiP_baseline, "toggle", m_baseline ? pcf::IndiElement::On : pcf::IndiElement::Off,
                           m_baseline ? pcf::IndiProperty::Ok : pcf::IndiProperty::Idle );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_randomize )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_randomize, ipRecv );
    if( !ipRecv.find( "toggle" ) )
    {
        return 0;
    }
    m_randomize = ( ipRecv["toggle"].getSwitchState() == pcf::IndiElement::On );
    updateSwitchIfChanged( m_indiP_randomize, "toggle", m_randomize ? pcf::IndiElement::On : pcf::IndiElement::Off,
                           m_randomize ? pcf::IndiProperty::Ok : pcf::IndiProperty::Idle );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_ignoreFocus )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_ignoreFocus, ipRecv );
    if( !ipRecv.find( "toggle" ) )
    {
        return 0;
    }
    m_ignoreFocus = ( ipRecv["toggle"].getSwitchState() == pcf::IndiElement::On );
    updateSwitchIfChanged( m_indiP_ignoreFocus, "toggle",
                           m_ignoreFocus ? pcf::IndiElement::On : pcf::IndiElement::Off,
                           m_ignoreFocus ? pcf::IndiProperty::Ok : pcf::IndiProperty::Idle );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_run )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_run, ipRecv );
    if( !ipRecv.find( "toggle" ) )
    {
        return 0;
    }

    if( ipRecv["toggle"].getSwitchState() == pcf::IndiElement::On )
    {
        if( m_busy.load() || m_saveFlatRequested.load() || m_abortRequested.load() ||
            m_resetRequested.load() || m_darkLibLoadRequested.load() )
        {
            log<text_log>( "run: already busy", logPrio::LOG_WARNING );
            return 0;
        }
        m_runRequested = true;
        setRunToggle( true, pcf::IndiProperty::Busy );
        return 0;
    }

    m_runRequested = false;
    setRunToggle( false, pcf::IndiProperty::Idle );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_abort )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_abort, ipRecv );
    if( !ipRecv.find( "request" ) )
    {
        return -1;
    }
    if( ipRecv["request"].getSwitchState() == pcf::IndiElement::On )
    {
        m_runRequested = false;
        m_abortRequested = true;
        setRunToggle( false, pcf::IndiProperty::Idle );
        updateSwitchIfChanged( m_indiP_abort, "request", pcf::IndiElement::On, INDI_BUSY );
    }
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_saveFlat )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_saveFlat, ipRecv );
    if( !ipRecv.find( "request" ) )
    {
        return -1;
    }
    if( ipRecv["request"].getSwitchState() == pcf::IndiElement::On )
    {
        if( m_busy.load() || m_runRequested.load() )
        {
            log<text_log>( "save_flat: already busy", logPrio::LOG_WARNING );
            clearRequest( m_indiP_saveFlat );
            return 0;
        }
        m_saveFlatRequested = true;
        updateSwitchIfChanged( m_indiP_saveFlat, "request", pcf::IndiElement::On, INDI_BUSY );
    }
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_darkLibLoad )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_darkLibLoad, ipRecv );
    if( !ipRecv.find( "request" ) )
    {
        return -1;
    }
    if( ipRecv["request"].getSwitchState() == pcf::IndiElement::On )
    {
        if( m_busy.load() || m_runRequested.load() )
        {
            log<text_log>( "reload_dark_lib: already busy", logPrio::LOG_WARNING );
            clearRequest( m_indiP_darkLibLoad );
            return 0;
        }
        m_darkLibLoadRequested = true;
        updateSwitchIfChanged( m_indiP_darkLibLoad, "request", pcf::IndiElement::On, INDI_BUSY );
    }
    return 0;
}

namespace
{
bool parseIndiCurrentNumber( const pcf::IndiProperty &ip, double &out )
{
    try
    {
        if( !ip.find( "current" ) )
        {
            return false;
        }
        const std::string s = ip["current"].getValue();
        if( s.empty() )
        {
            return false;
        }
        char *end = nullptr;
        const double v = std::strtod( s.c_str(), &end );
        if( end == s.c_str() || !std::isfinite( v ) )
        {
            return false;
        }
        out = v;
        return true;
    }
    catch( ... )
    {
        return false;
    }
}
} // namespace

INDI_SETCALLBACK_DEFN( eyeDoctor, m_indiP_remoteExptime )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_remoteExptime, ipRecv );
    parseIndiCurrentNumber( ipRecv, m_remoteExp );
    return 0;
}

INDI_SETCALLBACK_DEFN( eyeDoctor, m_indiP_remoteFps )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_remoteFps, ipRecv );
    parseIndiCurrentNumber( ipRecv, m_remoteFps );
    return 0;
}

INDI_SETCALLBACK_DEFN( eyeDoctor, m_indiP_remoteEmgain )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_remoteEmgain, ipRecv );
    parseIndiCurrentNumber( ipRecv, m_remoteGain );
    return 0;
}

INDI_SETCALLBACK_DEFN( eyeDoctor, m_indiP_remoteBlacklevel )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_remoteBlacklevel, ipRecv );
    parseIndiCurrentNumber( ipRecv, m_remoteBlacklevel );
    return 0;
}

INDI_SETCALLBACK_DEFN( eyeDoctor, m_indiP_remoteAmps )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_remoteAmps, ipRecv );

    int maxIdx = -1;
    std::vector<std::pair<int, double>> vals;
    vals.reserve( ipRecv.getNumElements() );
    for( const auto &kv : ipRecv.getElements() )
    {
        char *end = nullptr;
        const long idx = std::strtol( kv.first.c_str(), &end, 10 );
        if( end == kv.first.c_str() || idx < 0 )
        {
            continue;
        }
        double v = 0.0;
        try
        {
            v = kv.second.get<double>();
        }
        catch( ... )
        {
            continue;
        }
        vals.emplace_back( static_cast<int>( idx ), v );
        if( static_cast<int>( idx ) > maxIdx )
        {
            maxIdx = static_cast<int>( idx );
        }
    }

    if( maxIdx < 0 )
    {
        return 0;
    }

    {
        std::lock_guard<std::mutex> lock( m_modesMutex );
        m_remoteAmps.assign( static_cast<size_t>( maxIdx + 1 ), 0.0 );
        for( const auto &p : vals )
        {
            m_remoteAmps[static_cast<size_t>( p.first )] = p.second;
        }
        m_modesMax = static_cast<int>( m_remoteAmps.size() );
    }
    updateIfChanged( m_indiP_modesMax, "current", static_cast<double>( m_modesMax ) );
    return 0;
}

} // namespace app
} // namespace MagAOX

#endif // eyeDoctor_hpp
