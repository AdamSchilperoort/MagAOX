#pragma once

/// Shared dark-library schema used by darkCtrl, psfRefCtrl, and iefcCtrl.
///
/// Lives in apps/common/include (not under any one app) so the writer and
/// readers cannot drift. Include with -I../common/include and
/// `#include <lina/dark_library.h>`.
///
/// Directory layout (dark_lib_path):
///   dark_000.fits
///   dark_001.fits
///   ...
///   dark_metadata.txt   (CSV; canonical)
///
/// Match key is shm_cam_input (ImageStreamIO name), not the INDI cam_name device.
/// Legacy: dark_library.txt (space-separated) and darks/dark_NNN.fits are still read.
/// Old CSVs that only have a cam_name column treat that value as shm_cam_input.

#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <vector>

namespace lina {

inline constexpr const char *kDarkMetadataFile = "dark_metadata.txt";
inline constexpr double kDarkExptimeMatchTol = 1e-4;

struct DarkLibraryEntry {
    double exptime = 0.0;
    std::string relpath;
    unsigned ndark = 0;
    std::string shm_cam_input; ///< ImageStreamIO name these frames came from
    std::string cam_name;      ///< INDI camera device used when the dark was taken
    unsigned width = 0;
    unsigned height = 0;
    unsigned bitdepth = 0;
    int roi_x = 0;
    int roi_y = 0;
    unsigned roi_width = 0;
    unsigned roi_height = 0;
    double gain = std::numeric_limits<double>::quiet_NaN();
    double blacklevel = std::numeric_limits<double>::quiet_NaN();
};

struct DarkMatchFilter {
    std::string shm_cam_input; ///< Match ImageStreamIO name (required for iefc/psfRef)
    unsigned width = 0;
    unsigned height = 0;
    double gain = std::numeric_limits<double>::quiet_NaN();
    double blacklevel = std::numeric_limits<double>::quiet_NaN();
    double gain_tol = 1e-3;
    double blacklevel_tol = 1.0;
};

namespace darklib_detail {

inline std::string join_path( const std::string &dir, const std::string &name )
{
    if( dir.empty() )
        return name;
    if( dir.back() == '/' )
        return dir + name;
    return dir + "/" + name;
}

inline void ensure_dir( const std::string &dir )
{
    if( dir.empty() )
        return;
    if( mkdir( dir.c_str(), 0755 ) != 0 && errno != EEXIST )
        throw std::runtime_error( "mkdir failed: " + dir );
}

inline std::string trim( std::string s )
{
    while( !s.empty() && std::isspace( static_cast<unsigned char>( s.front() ) ) )
        s.erase( s.begin() );
    while( !s.empty() && std::isspace( static_cast<unsigned char>( s.back() ) ) )
        s.pop_back();
    return s;
}

inline double parse_dbl( const std::string &s )
{
    const std::string t = trim( s );
    if( t.empty() )
        return std::numeric_limits<double>::quiet_NaN();
    char *end = nullptr;
    const double v = std::strtod( t.c_str(), &end );
    if( end == t.c_str() )
        return std::numeric_limits<double>::quiet_NaN();
    return v;
}

inline unsigned parse_u( const std::string &s )
{
    const std::string t = trim( s );
    if( t.empty() )
        return 0;
    char *end = nullptr;
    const unsigned long v = std::strtoul( t.c_str(), &end, 10 );
    if( end == t.c_str() )
        return 0;
    return static_cast<unsigned>( v );
}

inline int parse_i( const std::string &s )
{
    const std::string t = trim( s );
    if( t.empty() )
        return 0;
    char *end = nullptr;
    const long v = std::strtol( t.c_str(), &end, 10 );
    if( end == t.c_str() )
        return 0;
    return static_cast<int>( v );
}

inline std::vector<std::string> split_csv( const std::string &line )
{
    std::vector<std::string> out;
    std::string cur;
    for( char c : line )
    {
        if( c == ',' )
        {
            out.push_back( trim( cur ) );
            cur.clear();
        }
        else
            cur.push_back( c );
    }
    out.push_back( trim( cur ) );
    return out;
}

inline std::string lower( std::string s )
{
    for( char &c : s )
        c = static_cast<char>( std::tolower( static_cast<unsigned char>( c ) ) );
    return s;
}

inline DarkLibraryEntry entry_from_fields( const std::map<std::string, std::string> &f )
{
    DarkLibraryEntry e;
    auto get = [&]( const char *k ) -> std::string {
        auto it = f.find( k );
        return it == f.end() ? std::string() : it->second;
    };
    e.relpath = get( "filename" );
    if( e.relpath.empty() )
        e.relpath = get( "relative_path" );
    e.exptime = parse_dbl( get( "exptime" ) );
    e.ndark = parse_u( get( "ndark" ) );
    e.shm_cam_input = get( "shm_cam_input" );
    e.cam_name = get( "cam_name" );
    if( e.shm_cam_input == "-" )
        e.shm_cam_input.clear();
    if( e.cam_name == "-" )
        e.cam_name.clear();
    // Legacy CSV: cam_name column was the stream/match key.
    if( e.shm_cam_input.empty() )
        e.shm_cam_input = e.cam_name;
    e.width = parse_u( get( "width" ) );
    e.height = parse_u( get( "height" ) );
    e.bitdepth = parse_u( get( "bitdepth" ) );
    e.roi_x = parse_i( get( "roi_x" ) );
    e.roi_y = parse_i( get( "roi_y" ) );
    e.roi_width = parse_u( get( "roi_width" ) );
    e.roi_height = parse_u( get( "roi_height" ) );
    const std::string em = get( "emgain" );
    e.gain = parse_dbl( em.empty() ? get( "gain" ) : em );
    e.blacklevel = parse_dbl( get( "blacklevel" ) );
    return e;
}

inline DarkLibraryEntry entry_from_legacy_tokens( const std::vector<std::string> &toks )
{
    DarkLibraryEntry e;
    if( toks.size() < 2 )
        return e;
    e.exptime = parse_dbl( toks[0] );
    e.relpath = toks[1];
    if( toks.size() > 2 )
        e.ndark = parse_u( toks[2] );
    if( toks.size() > 3 )
    {
        e.shm_cam_input = toks[3];
        if( e.shm_cam_input == "-" )
            e.shm_cam_input.clear();
        e.cam_name = e.shm_cam_input;
    }
    if( toks.size() > 5 )
    {
        e.width = parse_u( toks[4] );
        e.height = parse_u( toks[5] );
    }
    if( toks.size() > 6 )
        e.bitdepth = parse_u( toks[6] );
    if( toks.size() > 8 )
    {
        e.roi_x = parse_i( toks[7] );
        e.roi_y = parse_i( toks[8] );
    }
    if( toks.size() >= 13 )
    {
        e.roi_width = parse_u( toks[9] );
        e.roi_height = parse_u( toks[10] );
        e.gain = parse_dbl( toks[11] );
        e.blacklevel = parse_dbl( toks[12] );
    }
    else if( toks.size() >= 11 )
    {
        e.gain = parse_dbl( toks[9] );
        e.blacklevel = parse_dbl( toks[10] );
    }
    return e;
}

inline std::vector<DarkLibraryEntry> load_manifest_file( const std::string &path )
{
    std::ifstream in( path );
    if( !in )
        return {};
    std::vector<DarkLibraryEntry> entries;
    std::string line;
    std::vector<std::string> header;
    while( std::getline( in, line ) )
    {
        if( line.empty() || line[0] == '#' )
            continue;
        const bool csv = line.find( ',' ) != std::string::npos;
        if( csv )
        {
            auto cols = split_csv( line );
            if( cols.empty() )
                continue;
            if( header.empty() )
            {
                const std::string c0 = lower( cols[0] );
                if( c0 == "filename" || c0 == "exptime" || c0 == "relative_path" )
                {
                    for( auto &c : cols )
                        header.push_back( lower( c ) );
                    continue;
                }
                // Positional CSV without header.
                if( cols.size() >= 2 )
                {
                    std::map<std::string, std::string> f;
                    const char *keys[] = { "filename",    "exptime",    "ndark",      "shm_cam_input",
                                           "cam_name",    "width",      "height",     "bitdepth",
                                           "roi_x",       "roi_y",       "roi_width",  "roi_height",
                                           "emgain",      "blacklevel" };
                    for( std::size_t i = 0; i < cols.size() && i < 14; ++i )
                        f[keys[i]] = cols[i];
                    auto e = entry_from_fields( f );
                    if( !e.relpath.empty() )
                        entries.push_back( std::move( e ) );
                }
                continue;
            }
            std::map<std::string, std::string> f;
            for( std::size_t i = 0; i < cols.size() && i < header.size(); ++i )
                f[header[i]] = cols[i];
            auto e = entry_from_fields( f );
            if( !e.relpath.empty() )
                entries.push_back( std::move( e ) );
        }
        else
        {
            std::stringstream ss( line );
            std::vector<std::string> toks;
            std::string tok;
            while( ss >> tok )
                toks.push_back( tok );
            if( toks.size() < 2 )
                continue;
            auto e = entry_from_legacy_tokens( toks );
            if( !e.relpath.empty() )
                entries.push_back( std::move( e ) );
        }
    }
    return entries;
}

} // namespace darklib_detail

inline std::vector<DarkLibraryEntry> load_dark_library_manifest( const std::string &lib_dir )
{
    const char *names[] = { kDarkMetadataFile, "dark_metadata.csv", "dark_library.txt" };
    for( const char *name : names )
    {
        auto entries = darklib_detail::load_manifest_file( darklib_detail::join_path( lib_dir, name ) );
        if( !entries.empty() )
            return entries;
    }
    return {};
}

inline std::vector<DarkLibraryEntry>
filter_dark_library_entries( const std::vector<DarkLibraryEntry> &entries,
                             const DarkMatchFilter &filter )
{
    std::vector<DarkLibraryEntry> out;
    out.reserve( entries.size() );
    for( const auto &e : entries )
    {
        if( !filter.shm_cam_input.empty() && !e.shm_cam_input.empty() &&
            e.shm_cam_input != filter.shm_cam_input )
            continue;
        if( filter.width > 0 && e.width > 0 && e.width != filter.width )
            continue;
        if( filter.height > 0 && e.height > 0 && e.height != filter.height )
            continue;
        if( std::isfinite( filter.gain ) && std::isfinite( e.gain ) &&
            std::fabs( e.gain - filter.gain ) > filter.gain_tol )
            continue;
        if( std::isfinite( filter.blacklevel ) )
        {
            const double ebl = std::isfinite( e.blacklevel ) ? e.blacklevel : 0.0;
            if( std::fabs( ebl - filter.blacklevel ) > filter.blacklevel_tol )
                continue;
        }
        out.push_back( e );
    }
    return out;
}

inline std::string pick_dark_from_library( const std::string &lib_dir, double target_exptime,
                                           const DarkMatchFilter &filter = {},
                                           DarkLibraryEntry *matched = nullptr,
                                           double *match_err = nullptr )
{
    auto entries = filter_dark_library_entries( load_dark_library_manifest( lib_dir ), filter );
    if( entries.empty() )
        return {};
    std::size_t best = 0;
    double best_err = std::numeric_limits<double>::infinity();
    bool found = false;
    for( std::size_t i = 0; i < entries.size(); ++i )
    {
        if( !std::isfinite( entries[i].exptime ) || !std::isfinite( target_exptime ) )
            continue;
        const double err = std::fabs( entries[i].exptime - target_exptime );
        if( err < best_err )
        {
            best_err = err;
            best = i;
            found = true;
        }
    }
    if( !found )
        return {};
    if( matched )
        *matched = entries[best];
    if( match_err )
        *match_err = best_err;
    std::cout << "dark library: " << entries.size() << " matching entries; target exptime="
              << target_exptime << " -> closest " << entries[best].exptime
              << " (err=" << best_err << " s, " << entries[best].relpath << ")\n";
    if( best_err > kDarkExptimeMatchTol )
    {
        std::cerr << "warning: nearest dark exptime " << entries[best].exptime << " differs by "
                  << best_err << " s from requested " << target_exptime
                  << " (tol=" << kDarkExptimeMatchTol << " s)\n";
    }
    const std::string &rel = entries[best].relpath;
    if( !rel.empty() && rel[0] == '/' )
        return rel;
    return darklib_detail::join_path( lib_dir, rel );
}

inline void write_dark_library_manifest( const std::string &lib_dir,
                                         const std::vector<DarkLibraryEntry> &entries )
{
    darklib_detail::ensure_dir( lib_dir );
    const std::string path = darklib_detail::join_path( lib_dir, kDarkMetadataFile );
    std::ofstream out( path );
    if( !out )
        throw std::runtime_error( "failed to write " + path );
    out << "# dark_metadata_format=2\n";
    out << "filename,exptime,ndark,shm_cam_input,cam_name,width,height,bitdepth,roi_x,roi_y,"
           "roi_width,roi_height,emgain,blacklevel\n";
    out << std::setprecision( 17 );
    for( const auto &e : entries )
    {
        out << e.relpath << "," << e.exptime << "," << e.ndark << ","
            << ( e.shm_cam_input.empty() ? "-" : e.shm_cam_input ) << ","
            << ( e.cam_name.empty() ? "-" : e.cam_name ) << "," << e.width << "," << e.height << ","
            << e.bitdepth << "," << e.roi_x << "," << e.roi_y << "," << e.roi_width << ","
            << e.roi_height << ",";
        if( std::isfinite( e.gain ) )
            out << e.gain;
        out << ",";
        if( std::isfinite( e.blacklevel ) )
            out << e.blacklevel;
        out << "\n";
    }
}

} // namespace lina
