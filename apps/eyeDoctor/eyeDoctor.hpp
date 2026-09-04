/** \file eyeDoctor.hpp
  * \brief MagAO-X Eye Doctor: modal DM grid-search to maximize PSF core flux.
  *
  * C++ port of magpyx/scoobpy eye_doctor. Modes are generated in-process
  * (Noll Zernike via mxlib, Sylvester Hadamard on a circular actuator mask)
  * or optionally loaded from a FITS cube. Grid-search pokes go on a temporary
  * cacao channel; the winning command accumulates on an empty eye-doctor
  * channel. save_flat copies the summed DM command onto the flat channel,
  * zeros the eye-doctor channels, and writes a FITS file.
  *
  * \ingroup eyeDoctor_files
  */

#ifndef eyeDoctor_hpp
#define eyeDoctor_hpp

#include <atomic>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

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
/** INDI front-end for the eye_doctor grid-sweep algorithm.
  *
  * Hardware:
  *  - \c shm_dm_eyeDoc       : accumulated modal command (start empty)
  *  - \c shm_dm_eyeDoc_sweep : temporary grid-search pokes
  *  - \c shm_dm_flat         : cacao flat (typically disp00)
  *  - \c shm_dm_sum          : cacao total (dmXXdisp)
  *  - \c shm_cam             : WFS / science-camera image shmim
  *  - \c cam_name            : INDI device of that camera
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
    std::string m_shmDmEyeDoc{ "dm01disp07" };
    std::string m_shmDmSweep{ "dm01disp06" };
    std::string m_shmDmFlat{ "dm01disp00" };
    std::string m_shmDmSum{ "dm01disp" };
    std::string m_shmCam{ "camsci" };
    std::string m_camName{ "camsci" };
    std::string m_flatDir{ "/opt/MagAOX/calib/dm/bmc_1k/flats" };
    std::string m_lastFlatPath;
    std::string m_darkLibPath; ///< darkCtrl library (dark_metadata.txt + dark_NNN.fits)
    ///@}

    /** \name Modes
      *@{
      */
    std::string m_modeType{ "zernike" }; ///< zernike | hadamard | fits
    int m_nModes{ 36 };                  ///< Zernike planes to generate (Noll j=1 .. n)
    std::string m_modesetPath;           ///< Used only when mode_type=fits
    ///@}

    /** \name Algorithm parameters (Python eye_doctor / console_comprehensive)
      *@{
      */
    int m_modeStart{ 0 };
    int m_modeEnd{ 10 };
    int m_focusModeIndex{ 3 }; ///< 0-based Noll focus (Z4) when piston is index 0
    double m_coreRadius{ 10.0 };
    double m_searchRange{ 0.1 }; ///< Total span; sweep is [-range/2, +range/2]
    double m_searchStep{ 0.0 };  ///< Amplitude spacing; 0 = use n_steps
    int m_nSteps{ 20 };
    int m_nRepeats{ 3 };
    int m_nSeqRepeat{ 1 };
    int m_nImages{ 1 };
    int m_skipFrames{ 0 };
    double m_cenX{ -1.0 }; ///< <0 = auto centroid
    double m_cenY{ -1.0 };
    double m_satThresh{ 55000.0 }; ///< Warn if camera peak >= this (0 = off)
    double m_blankThresh{ 0.0 };   ///< Peak ADU treated as off-camera. 0 = 10% of sweep max.
    double m_exptimeTol{ 1e-4 };    ///< |live exptime - library exptime| allowed [s]
    double m_dmDelay{ 0.1 }; ///< Seconds after each DM write
    std::string m_searchKind{ "fit" };
    bool m_resetToZero{ false };
    bool m_ignoreFocus{ false };
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
    int m_nModesLoaded{ 0 };
    double m_lastAmp{ 0 };
    double m_lastMetric{ 0 };
    int m_satWarnedMode{ -2 };
    int m_nanWarnedMode{ -2 };
    ///@}

    dev::wavefrontHardware m_hw;
    dev::modeCube m_modes;

    /** \name INDI
      *@{
      */
    pcf::IndiProperty m_indiP_shmDmEyeDoc;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_shmDmEyeDoc );
    pcf::IndiProperty m_indiP_shmDmSweep;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_shmDmSweep );
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

    pcf::IndiProperty m_indiP_modeType;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_modeType );
    pcf::IndiProperty m_indiP_nModes;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_nModes );
    pcf::IndiProperty m_indiP_modeset;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_modeset );

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
    pcf::IndiProperty m_indiP_searchKind;
    INDI_NEWCALLBACK_DECL( eyeDoctor, m_indiP_searchKind );
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
    pcf::IndiProperty m_indiP_nModesLoaded;
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
    int saveFlat();
    int abortAndZero();
    int resetToZero();
    int zeroAlgoChannels();
    int reloadDarkLib();
    int refreshDark( bool required );
    lina::DarkMatchFilter darkFilter() const;
    std::string formatDarkEntry( const lina::DarkLibraryEntry &e ) const;
    std::string pickDark( double target_exptime, const lina::DarkMatchFilter &filter,
                           lina::DarkLibraryEntry *matched, double *match_err );
    void applyDark( mx::improc::eigenImage<float> &im );
    int prepareModes( int size0, int size1 );
    int measureMetric( mx::improc::eigenImage<float> &im, double &metric );
    void warnIfSaturated( const mx::improc::eigenImage<float> &im );
    void warnIfDmNonFinite( const std::string &channel, int nbad );
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
    config.add( "shmims.shm_dm_eyeDoc", "", "shmims.shm_dm_eyeDoc", argType::Required, "shmims", "shm_dm_eyeDoc", false,
                "string", "Accumulated eye-doctor command channel (start empty)." );
    config.add( "shmims.shm_dm_eyeDoc_sweep", "", "shmims.shm_dm_eyeDoc_sweep", argType::Required, "shmims",
                "shm_dm_eyeDoc_sweep", false, "string",
                "Temporary grid-search poke channel. Set equal to shm_dm_eyeDoc for single-channel." );
    config.add( "shmims.shm_dm_flat", "", "shmims.shm_dm_flat", argType::Required, "shmims", "shm_dm_flat", false,
                "string", "cacao flat channel (typically dmXXdisp00)." );
    config.add( "shmims.shm_dm_sum", "", "shmims.shm_dm_sum", argType::Required, "shmims", "shm_dm_sum", false, "string",
                "cacao summed / total DM command (dmXXdisp)." );
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
    config.add( "eyedoctor.mode_type", "", "eyedoctor.mode_type", argType::Required, "eyedoctor", "mode_type", false,
                "string", "zernike, hadamard, or fits." );
    config.add( "eyedoctor.n_modes", "", "eyedoctor.n_modes", argType::Required, "eyedoctor", "n_modes", false, "int",
                "Number of Zernike modes to generate (Noll j=1 is plane 0). Ignored for hadamard." );
    config.add( "eyedoctor.modeset", "", "eyedoctor.modeset", argType::Required, "eyedoctor", "modeset", false,
                "string", "Optional FITS cube when mode_type=fits." );
    config.add( "eyedoctor.mode_start", "", "eyedoctor.mode_start", argType::Required, "eyedoctor", "mode_start", false,
                "int", "First 0-based mode index to optimize." );
    config.add( "eyedoctor.mode_end", "", "eyedoctor.mode_end", argType::Required, "eyedoctor", "mode_end", false, "int",
                "Last 0-based mode index to optimize (inclusive)." );
    config.add( "eyedoctor.focus_mode_index", "", "eyedoctor.focus_mode_index", argType::Required, "eyedoctor",
                "focus_mode_index", false, "int", "0-based focus mode skipped when ignore_focus is on (default 3)." );
    config.add( "eyedoctor.core_radius", "", "eyedoctor.core_radius", argType::Required, "eyedoctor", "core_radius",
                false, "float", "PSF core radius [pixels] for coresum metric." );
    config.add( "eyedoctor.search_range", "", "eyedoctor.search_range", argType::Required, "eyedoctor", "search_range",
                false, "float", "Total amplitude span; sweep is +/- range/2." );
    config.add( "eyedoctor.search_step", "", "eyedoctor.search_step", argType::Required, "eyedoctor", "search_step",
                false, "float", "Amplitude step size. If >0, n_steps is derived as range/step + 1." );
    config.add( "eyedoctor.n_steps", "", "eyedoctor.n_steps", argType::Required, "eyedoctor", "n_steps", false, "int",
                "Grid samples per sweep." );
    config.add( "eyedoctor.n_repeats", "", "eyedoctor.n_repeats", argType::Required, "eyedoctor", "n_repeats", false,
                "int", "Number of sweep repeats averaged / jointly fit." );
    config.add( "eyedoctor.n_seq_repeat", "", "eyedoctor.n_seq_repeat", argType::Required, "eyedoctor", "n_seq_repeat",
                false, "int", "Repeat the full mode sequence this many times." );
    config.add( "eyedoctor.n_images", "", "eyedoctor.n_images", argType::Required, "eyedoctor", "n_images", false,
                "int", "Camera frames averaged per metric sample." );
    config.add( "eyedoctor.skip_frames", "", "eyedoctor.skip_frames", argType::Required, "eyedoctor", "skip_frames",
                false, "int", "Camera frames to discard after each DM write." );
    config.add( "eyedoctor.cen_x", "", "eyedoctor.cen_x", argType::Required, "eyedoctor", "cen_x", false, "float",
                "PSF x pixel in the camera image (size[0], 0-based). <0 = auto." );
    config.add( "eyedoctor.cen_y", "", "eyedoctor.cen_y", argType::Required, "eyedoctor", "cen_y", false, "float",
                "PSF y pixel in the camera image (size[1], 0-based). <0 = auto." );
    config.add( "eyedoctor.sat_thresh", "", "eyedoctor.sat_thresh", argType::Required, "eyedoctor", "sat_thresh",
                false, "float", "Warn if camera peak ADU >= this. 0 disables." );
    config.add( "eyedoctor.blank_thresh", "", "eyedoctor.blank_thresh", argType::Required, "eyedoctor", "blank_thresh",
                false, "float", "Peak ADU treated as PSF off-camera. 0 = 10% of the sweep's max peak." );
    config.add( "eyedoctor.dm_delay", "", "eyedoctor.dm_delay", argType::Required, "eyedoctor", "dm_delay", false,
                "float", "Settle time after DM write [s]." );
    config.add( "eyedoctor.search_kind", "", "eyedoctor.search_kind", argType::Required, "eyedoctor", "search_kind",
                false, "string", "fit (quadratic) or mean (argmin average)." );
    config.add( "eyedoctor.reset_to_zero", "", "eyedoctor.reset_to_zero", argType::Required, "eyedoctor",
                "reset_to_zero", false, "bool",
                "On run, ignore the current eyeDoc channel and start from zeros." );
    config.add( "eyedoctor.ignore_focus", "", "eyedoctor.ignore_focus", argType::Required, "eyedoctor", "ignore_focus",
                false, "bool", "Skip focus_mode_index." );
}

void eyeDoctor::loadConfig()
{
    config( m_shmDmEyeDoc, "shmims.shm_dm_eyeDoc" );
    config( m_shmDmSweep, "shmims.shm_dm_eyeDoc_sweep" );
    config( m_shmDmFlat, "shmims.shm_dm_flat" );
    config( m_shmDmSum, "shmims.shm_dm_sum" );
    config( m_shmCam, "shmims.shm_cam" );
    config( m_camName, "camera.cam_name" );
    config( m_flatDir, "eyedoctor.flat_dir" );
    config( m_darkLibPath, "eyedoctor.dark_lib_path" );
    config( m_exptimeTol, "eyedoctor.exptime_tol" );
    config( m_modeType, "eyedoctor.mode_type" );
    config( m_nModes, "eyedoctor.n_modes" );
    config( m_modesetPath, "eyedoctor.modeset" );
    config( m_modeStart, "eyedoctor.mode_start" );
    config( m_modeEnd, "eyedoctor.mode_end" );
    config( m_focusModeIndex, "eyedoctor.focus_mode_index" );
    config( m_coreRadius, "eyedoctor.core_radius" );
    config( m_searchRange, "eyedoctor.search_range" );
    config( m_searchStep, "eyedoctor.search_step" );
    config( m_nSteps, "eyedoctor.n_steps" );
    config( m_nRepeats, "eyedoctor.n_repeats" );
    config( m_nSeqRepeat, "eyedoctor.n_seq_repeat" );
    config( m_nImages, "eyedoctor.n_images" );
    config( m_skipFrames, "eyedoctor.skip_frames" );
    config( m_cenX, "eyedoctor.cen_x" );
    config( m_cenY, "eyedoctor.cen_y" );
    config( m_satThresh, "eyedoctor.sat_thresh" );
    config( m_blankThresh, "eyedoctor.blank_thresh" );
    config( m_dmDelay, "eyedoctor.dm_delay" );
    config( m_searchKind, "eyedoctor.search_kind" );
    config( m_resetToZero, "eyedoctor.reset_to_zero" );
    config( m_ignoreFocus, "eyedoctor.ignore_focus" );
}

int eyeDoctor::appStartup()
{
    CREATE_REG_INDI_NEW_TEXT( m_indiP_shmDmEyeDoc, "shm_dm_eyeDoc", "Accumulated eye-doctor DM channel", "shmims" );
    CREATE_REG_INDI_NEW_TEXT( m_indiP_shmDmSweep, "shm_dm_eyeDoc_sweep", "Grid-search poke DM channel", "shmims" );
    CREATE_REG_INDI_NEW_TEXT( m_indiP_shmDmFlat, "shm_dm_flat", "cacao flat DM channel", "shmims" );
    CREATE_REG_INDI_NEW_TEXT( m_indiP_shmDmSum, "shm_dm_sum", "cacao summed DM command", "shmims" );
    CREATE_REG_INDI_NEW_TEXT( m_indiP_shmCam, "shm_cam", "WFS camera image shmim", "shmims" );
    CREATE_REG_INDI_NEW_TEXT( m_indiP_camName, "cam_name", "INDI WFS camera device", "camera" );
    CREATE_REG_INDI_NEW_TEXT( m_indiP_flatDir, "flat_dir", "Directory for saved flat FITS", "flat" );
    CREATE_REG_INDI_NEW_TEXT( m_indiP_darkLibPath, "dark_lib_path", "darkCtrl library directory", "paths" );

    CREATE_REG_INDI_NEW_TEXT( m_indiP_modeType, "mode_type", "zernike, hadamard, or fits", "modes" );
    CREATE_REG_INDI_NEW_NUMBERI( m_indiP_nModes, "n_modes", 1, 10000, 1, "%d", "Zernike modes to generate", "modes" );
    CREATE_REG_INDI_NEW_TEXT( m_indiP_modeset, "modeset", "FITS cube when mode_type=fits", "modes" );

    CREATE_REG_INDI_NEW_NUMBERI( m_indiP_modeStart, "mode_start", 0, 10000, 1, "%d", "First mode index", "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERI( m_indiP_modeEnd, "mode_end", 0, 10000, 1, "%d", "Last mode index", "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERI( m_indiP_focusModeIndex, "focus_mode_index", 0, 10000, 1, "%d", "Focus mode index",
                                "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERF( m_indiP_coreRadius, "core_radius", 0.5, 500, 0.5, "%0.2f", "PSF core radius [pix]",
                                "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERF( m_indiP_searchRange, "search_range", 0, 10, 0.01, "%0.4f", "Sweep span (full)",
                                "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERF( m_indiP_searchStep, "search_step", 0, 10, 0.001, "%0.4f",
                                "Amplitude step (0 = use n_steps)", "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERI( m_indiP_nSteps, "n_steps", 3, 500, 1, "%d", "Grid samples", "algorithm" );
    CREATE_REG_INDI_NEW_NUMBERI( m_indiP_nRepeats, "n_repeats", 1, 50, 1, "%d", "Sweep repeats", "algorithm" );
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
    CREATE_REG_INDI_NEW_NUMBERF( m_indiP_dmDelay, "dm_delay", 0, 10, 0.01, "%0.3f", "DM settle [s]", "algorithm" );
    CREATE_REG_INDI_NEW_TEXT( m_indiP_searchKind, "search_kind", "fit or mean", "algorithm" );

    if( createStandardIndiToggleSw( m_indiP_ignoreFocus, "ignore_focus", "Skip focus mode", "algorithm" ) < 0 )
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

    REG_INDI_NEWPROP_NOCB( m_indiP_nModesLoaded, "n_modes_loaded", pcf::IndiProperty::Number );
    m_indiP_nModesLoaded.add( pcf::IndiElement( "current" ) );
    m_indiP_nModesLoaded["current"].set( 0.0 );

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

    m_indiP_shmDmEyeDoc["current"].setValue( m_shmDmEyeDoc );
    m_indiP_shmDmEyeDoc["target"].setValue( m_shmDmEyeDoc );
    m_indiP_shmDmSweep["current"].setValue( m_shmDmSweep );
    m_indiP_shmDmSweep["target"].setValue( m_shmDmSweep );
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
    m_indiP_modeType["current"].setValue( m_modeType );
    m_indiP_modeType["target"].setValue( m_modeType );
    m_indiP_nModes["current"].setValue( m_nModes );
    m_indiP_nModes["target"].setValue( m_nModes );
    m_indiP_modeset["current"].setValue( m_modesetPath );
    m_indiP_modeset["target"].setValue( m_modesetPath );
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
    m_indiP_searchKind["current"].setValue( m_searchKind );
    m_indiP_searchKind["target"].setValue( m_searchKind );

    updateSwitchIfChanged( m_indiP_ignoreFocus, "toggle", m_ignoreFocus ? pcf::IndiElement::On : pcf::IndiElement::Off,
                           INDI_IDLE );

    REG_INDI_SETPROP( m_indiP_remoteExptime, m_camName, "exptime" );
    REG_INDI_SETPROP( m_indiP_remoteFps, m_camName, "fps" );
    REG_INDI_SETPROP( m_indiP_remoteEmgain, m_camName, "emgain" );
    REG_INDI_SETPROP( m_indiP_remoteBlacklevel, m_camName, "blacklevel" );

    m_worker = std::thread( workerStart, this );
    state( stateCodes::READY );
    log<text_log>( "eyeDoctor ready (shm_dm_eyeDoc=" + m_shmDmEyeDoc + " shm_dm_sum=" + m_shmDmSum +
                   " cam_name=" + m_camName + " mode_type=" + m_modeType + ")" );
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

void eyeDoctor::warnIfDmNonFinite( const std::string &channel, int nbad )
{
    if( nbad <= 0 )
    {
        return;
    }
    if( m_nanWarnedMode == m_currentMode )
    {
        return;
    }
    m_nanWarnedMode = m_currentMode;
    log<text_log>( "replaced " + std::to_string( nbad ) + " NaN/Inf pixels with 0 on " + channel +
                       " (mode " + std::to_string( m_currentMode ) + ")",
                   logPrio::LOG_WARNING );
}

int eyeDoctor::prepareModes( int size0, int size1 )
{
    int rv = -1;
    if( m_modeType == "zernike" )
    {
        rv = m_modes.generateZernike( size0, size1, std::max( 1, m_nModes ), 1 );
        if( rv < 0 )
        {
            setStatus( "zernike generation failed" );
            return -1;
        }
    }
    else if( m_modeType == "hadamard" )
    {
        rv = m_modes.generateHadamard( size0, size1 );
        if( rv < 0 )
        {
            setStatus( "hadamard generation failed" );
            return -1;
        }
    }
    else if( m_modeType == "fits" )
    {
        if( m_modesetPath.empty() )
        {
            setStatus( "no modeset path" );
            return -1;
        }
        if( m_modes.load( m_modesetPath, "modes" ) < 0 )
        {
            setStatus( "failed to load modeset" );
            return -1;
        }
        if( m_modes.size0() != size0 || m_modes.size1() != size1 )
        {
            setStatus( "modeset size != DM shmim size" );
            log<software_error>( { __FILE__, __LINE__,
                                   "modes " + std::to_string( m_modes.size0() ) + "x" +
                                       std::to_string( m_modes.size1() ) + " vs dm " + std::to_string( size0 ) + "x" +
                                       std::to_string( size1 ) } );
            return -1;
        }
    }
    else
    {
        setStatus( "mode_type must be zernike, hadamard, or fits" );
        return -1;
    }

    m_nModesLoaded = m_modes.nModes();
    updateIfChanged( m_indiP_nModesLoaded, "current", static_cast<double>( m_nModesLoaded ) );
    const int nbad = m_modes.sanitize();
    if( nbad > 0 )
    {
        log<text_log>( "modes: replaced " + std::to_string( nbad ) + " non-finite pixels with 0",
                       logPrio::LOG_WARNING );
    }
    log<text_log>( "modes: " + m_modes.name + " n=" + std::to_string( m_nModesLoaded ) + " " +
                   std::to_string( size0 ) + "x" + std::to_string( size1 ) );
    return 0;
}

int eyeDoctor::zeroAlgoChannels()
{
    m_hw.dmEyeDocName = m_shmDmEyeDoc;
    m_hw.dmSweepName = m_shmDmSweep;
    m_hw.disconnect();
    if( m_hw.connectAlgoChannels() < 0 )
    {
        log<software_error>( { __FILE__, __LINE__, m_hw.error() } );
        setStatus( "DM connect failed: " + m_hw.error() );
        return -1;
    }
    if( m_hw.zeroEyeDoc() < 0 || m_hw.zeroSweep() < 0 )
    {
        setStatus( "failed to zero eye-doctor channels" );
        return -1;
    }
    m_currentMode = -1;
    updateIfChanged( m_indiP_currentMode, "current", -1.0 );
    return 0;
}

int eyeDoctor::abortAndZero()
{
    setStatus( "aborting" );
    if( zeroAlgoChannels() < 0 )
    {
        return -1;
    }
    log<text_log>( "aborted: zeroed " + m_shmDmEyeDoc + " and " + m_shmDmSweep );
    return 0;
}

int eyeDoctor::resetToZero()
{
    setStatus( "resetting to zero" );
    if( zeroAlgoChannels() < 0 )
    {
        return -1;
    }
    log<text_log>( "reset_to_zero: wrote 0 to " + m_shmDmEyeDoc + " and " + m_shmDmSweep );
    return 0;
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

    m_hw.dmEyeDocName = m_shmDmEyeDoc;
    m_hw.dmSweepName = m_shmDmSweep;
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
    if( m_hw.zeroEyeDoc() < 0 || m_hw.zeroSweep() < 0 )
    {
        setStatus( "failed to zero eye-doctor channels" );
        return -1;
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

int eyeDoctor::runOptimization()
{
    setStatus( "connecting" );

    m_hw.dmEyeDocName = m_shmDmEyeDoc;
    m_hw.dmSweepName = m_shmDmSweep;
    m_hw.camName = m_shmCam;
    m_hw.camDevice = m_camName;
    m_hw.disconnect();
    if( m_hw.connectLoop() < 0 )
    {
        log<software_error>( { __FILE__, __LINE__, m_hw.error() } );
        setStatus( "connect failed: " + m_hw.error() );
        return -1;
    }
    if( stopping() )
    {
        return -2;
    }

    setStatus( "generating modes" );
    if( prepareModes( static_cast<int>( m_hw.dmEyeDoc.size0() ), static_cast<int>( m_hw.dmEyeDoc.size1() ) ) < 0 )
    {
        return -1;
    }
    if( stopping() )
    {
        return -2;
    }

    const int start = std::max( 0, m_modeStart );
    const int end = std::min( m_modeEnd, m_nModesLoaded - 1 );
    if( end < start )
    {
        setStatus( "invalid mode range" );
        return -1;
    }

    mx::improc::eigenImage<float> eyeDocCmd;
    if( m_resetToZero )
    {
        eyeDocCmd.resize( m_modes.size0(), m_modes.size1() );
        eyeDocCmd.setZero();
    }
    else if( m_hw.grabEyeDoc( eyeDocCmd ) < 0 )
    {
        setStatus( "failed to read eyeDoc channel" );
        return -1;
    }
    else
    {
        const int nbad = dev::replaceNonFinite( eyeDocCmd );
        if( nbad > 0 )
        {
            log<text_log>( "eyeDoc channel had " + std::to_string( nbad ) +
                               " NaN/Inf pixels; replacing with 0 so dmcomb does not poison the sum",
                           logPrio::LOG_WARNING );
        }
    }

    if( m_hw.writeEyeDoc( eyeDocCmd ) < 0 || m_hw.zeroSweep() < 0 )
    {
        setStatus( "failed to initialize DM channels" );
        return -1;
    }

    if( !m_darkLibPath.empty() )
    {
        refreshDark( false );
    }

    const int seqRepeats = std::max( 1, m_nSeqRepeat );
    mx::improc::eigenImage<float> camIm;

    for( int seq = 0; seq < seqRepeats && !stopping(); ++seq )
    {
        for( int mi = start; mi <= end && !stopping(); ++mi )
        {
            if( m_ignoreFocus && mi == m_focusModeIndex )
            {
                continue;
            }

            m_currentMode = mi;
            m_satWarnedMode = -2;
            m_nanWarnedMode = -2;
            updateIfChanged( m_indiP_currentMode, "current", static_cast<double>( mi ) );
            setStatus( "mode " + std::to_string( mi ) + " seq " + std::to_string( seq + 1 ) + "/" +
                       std::to_string( seqRepeats ) );

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

            mx::improc::eigenImage<float> mode = m_modes.modes.image( mi );
            dev::gridSweep sweep;
            sweep.lo = -0.5 * m_searchRange;
            sweep.hi = 0.5 * m_searchRange;
            if( m_searchStep > 0.0 && m_searchRange > 0.0 )
            {
                sweep.nSteps = std::max( 3, static_cast<int>( std::lround( m_searchRange / m_searchStep ) ) + 1 );
            }
            else
            {
                sweep.nSteps = std::max( 3, m_nSteps );
            }
            sweep.nRepeats = std::max( 1, m_nRepeats );
            sweep.kind = m_searchKind;
            sweep.blankThresh = m_blankThresh;

            const auto sw = sweep.run(
                [&]( double a ) -> int {
                    if( m_hw.applySweep( mode, a, &eyeDocCmd ) < 0 )
                    {
                        return -1;
                    }
                    const int nbad = m_hw.singleChannel() ? m_hw.dmEyeDoc.lastNonFinite()
                                                          : m_hw.dmSweep.lastNonFinite();
                    warnIfDmNonFinite( m_hw.singleChannel() ? m_shmDmEyeDoc : m_shmDmSweep, nbad );
                    if( m_dmDelay > 0 )
                    {
                        mx::sys::milliSleep( static_cast<unsigned>( m_dmDelay * 1000.0 ) );
                    }
                    return 0;
                },
                [&]() -> dev::metricSample {
                    double m = 0;
                    if( measureMetric( camIm, m ) < 0 )
                    {
                        return { 1e6, 0.0 };
                    }
                    const double peak = camIm.size() > 0 ? static_cast<double>( camIm.maxCoeff() ) : 0.0;
                    return { m, peak };
                },
                [this]() { return stopping(); } );

            if( stopping() || sw.stopped )
            {
                m_hw.zeroSweep();
                return -2;
            }

            const double useAmp = dev::finiteOrZero( sw.amp );
            if( m_searchKind == "fit" && !sw.usedFit )
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
                if( useAmp == 0.0 )
                {
                    why += ", leaving amp=0";
                }
                else
                {
                    why += ", using best-sample amp=" + std::to_string( useAmp );
                }
                log<text_log>( why, logPrio::LOG_WARNING );
            }
            else if( m_searchKind == "fit" && ( sw.truncated || sw.refined ) )
            {
                log<text_log>( "mode " + std::to_string( mi ) + ": quadratic on " +
                                   std::to_string( sw.nGood ) + "/" + std::to_string( sw.nTotal ) +
                                   " on-camera samples" + ( sw.refined ? " after refine" : "" ),
                               logPrio::LOG_INFO );
            }

            eyeDocCmd += mode * static_cast<float>( useAmp );
            dev::replaceNonFinite( eyeDocCmd );
            if( m_hw.writeEyeDoc( eyeDocCmd ) < 0 || m_hw.zeroSweep() < 0 )
            {
                setStatus( "DM write failed" );
                return -1;
            }
            warnIfDmNonFinite( m_shmDmEyeDoc, m_hw.dmEyeDoc.lastNonFinite() );
            if( m_dmDelay > 0 )
            {
                mx::sys::milliSleep( static_cast<unsigned>( m_dmDelay * 1000.0 ) );
            }

            double metric1 = 0;
            measureMetric( camIm, metric1 );
            m_lastAmp = useAmp;
            m_lastMetric = dev::finiteOrZero( metric1 );
            updateIfChanged( m_indiP_optAmp, "current", m_lastAmp );
            updateIfChanged( m_indiP_metric, "current", m_lastMetric );
            log<text_log>( "mode " + std::to_string( mi ) + " amp=" + std::to_string( useAmp ) + " metric " +
                           std::to_string( metric0 ) + " -> " + std::to_string( metric1 ) );
        }
    }

    m_hw.zeroSweep();
    m_currentMode = -1;
    updateIfChanged( m_indiP_currentMode, "current", -1.0 );
    return stopping() ? -2 : 0;
}

// ---------- INDI NEW ----------

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_shmDmEyeDoc )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_shmDmEyeDoc, ipRecv );
    std::string target;
    if( indiTargetUpdate( m_indiP_shmDmEyeDoc, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_shmDmEyeDoc = target;
    updateIfChanged( m_indiP_shmDmEyeDoc, "current", m_shmDmEyeDoc );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_shmDmSweep )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_shmDmSweep, ipRecv );
    std::string target;
    if( indiTargetUpdate( m_indiP_shmDmSweep, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_shmDmSweep = target;
    updateIfChanged( m_indiP_shmDmSweep, "current", m_shmDmSweep );
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

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_modeType )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_modeType, ipRecv );
    std::string target;
    if( indiTargetUpdate( m_indiP_modeType, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    if( target != "zernike" && target != "hadamard" && target != "fits" )
    {
        log<text_log>( "mode_type must be zernike, hadamard, or fits", logPrio::LOG_ERROR );
        return -1;
    }
    m_modeType = target;
    updateIfChanged( m_indiP_modeType, "current", m_modeType );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_nModes )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_nModes, ipRecv );
    int target = 0;
    if( indiTargetUpdate( m_indiP_nModes, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_nModes = target;
    updateIfChanged( m_indiP_nModes, "current", m_nModes );
    return 0;
}

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_modeset )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_modeset, ipRecv );
    std::string target;
    if( indiTargetUpdate( m_indiP_modeset, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    m_modesetPath = target;
    updateIfChanged( m_indiP_modeset, "current", m_modesetPath );
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

INDI_NEWCALLBACK_DEFN( eyeDoctor, m_indiP_searchKind )( const pcf::IndiProperty &ipRecv )
{
    INDI_VALIDATE_CALLBACK_PROPS( m_indiP_searchKind, ipRecv );
    std::string target;
    if( indiTargetUpdate( m_indiP_searchKind, target, ipRecv, false ) < 0 )
    {
        return log<software_error, -1>( { __FILE__, __LINE__ } );
    }
    if( target != "fit" && target != "mean" )
    {
        log<text_log>( "search_kind must be fit or mean", logPrio::LOG_ERROR );
        return -1;
    }
    m_searchKind = target;
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

} // namespace app
} // namespace MagAOX

#endif // eyeDoctor_hpp
