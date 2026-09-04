/** \file dmWavefrontControl.hpp
  * \brief Shared DM + wavefront-sensor I/O and PSF optimization helpers.
  *
  * Connects cacao/MILK DM channel shmims, a camera (WFS) image shmim, and
  * optional camera INDI device name. Provides in-process Zernike / Hadamard
  * mode generation (plus optional FITS load), PSF core-sum metric, and the
  * grid-sweep / quadratic-fit loop used by eyeDoctor.
  *
  * \ingroup dmWavefrontControl_files
  */

#ifndef dmWavefrontControl_hpp
#define dmWavefrontControl_hpp

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <time.h>

#include <ImageStreamIO/ImageStreamIO.h>
#include <fitsio.h>

#include <Eigen/Dense>
#include <mx/improc/eigenCube.hpp>
#include <mx/improc/eigenImage.hpp>
#include <mx/ioutils/fits/fitsFile.hpp>
#include <mx/sigproc/zernike.hpp>

#include "../../ImageStreamIO/pixaccess.hpp"

/** \defgroup dmWavefrontControl
  * \brief Wavefront hardware I/O and modal PSF optimization utilities
  *
  * \ingroup appdev
  */

namespace MagAOX
{
namespace app
{
namespace dev
{

/// IEEE NaN/Inf test that still works when compiled with -ffast-math.
inline bool floatBitsNonFinite( float v )
{
    uint32_t u = 0;
    std::memcpy( &u, &v, sizeof( u ) );
    return ( u & 0x7fffffffu ) >= 0x7f800000u;
}

inline bool doubleBitsNonFinite( double v )
{
    uint64_t u = 0;
    std::memcpy( &u, &v, sizeof( u ) );
    return ( u & 0x7fffffffffffffffull ) >= 0x7ff0000000000000ull;
}

/// Force a finite scalar for INDI / DM use. Survives -ffast-math.
inline double finiteOrZero( double v )
{
    volatile uint64_t u = 0;
    std::memcpy( const_cast<uint64_t *>( &u ), &v, sizeof( uint64_t ) );
    if( ( u & 0x7fffffffffffffffull ) >= 0x7ff0000000000000ull )
    {
        return 0.0;
    }
    return v;
}

/// RAII wrapper around one ImageStreamIO shared-memory image.
/** Used for both DM command channels (write) and camera/WFS streams (read).
  * Writes use ImageStreamIO_writeBuffer + ImageStreamIO_UpdateIm so cacao
  * dmcomb sees cnt0 increment. Reads wait on a semaphore, then copy the
  * last-written slice into an eigenImage<float> with milkImage layout
  * (rows = size[0], cols = size[1]).
  */
class shmimStream
{
  public:
    shmimStream() = default;
    ~shmimStream()
    {
        close();
    }

    shmimStream( const shmimStream & ) = delete;
    shmimStream &operator=( const shmimStream & ) = delete;

    /// Open an existing stream. Closes any previous connection.
    /** \returns 0 on success, -1 on error.
      */
    int open( const std::string &name )
    {
        close();
        if( name.empty() )
        {
            m_error = "empty shmim name";
            return -1;
        }

        std::memset( &m_image, 0, sizeof( m_image ) );
        if( ImageStreamIO_openIm( &m_image, name.c_str() ) != IMAGESTREAMIO_SUCCESS )
        {
            m_error = "ImageStreamIO_openIm failed for " + name;
            return -1;
        }

        m_open = true;
        m_name = name;
        m_size0 = m_image.md->size[0];
        m_size1 = ( m_image.md->naxis >= 2 ) ? m_image.md->size[1] : 1;
        m_datatype = m_image.md->datatype;
        m_sem = ImageStreamIO_getsemwaitindex( &m_image, 0 );
        if( m_sem < 0 && m_image.md->sem > 0 )
        {
            close();
            m_error = "no free semaphore on " + name;
            return -1;
        }
        return 0;
    }

    void close()
    {
        if( m_open )
        {
            ImageStreamIO_closeIm( &m_image );
            m_open = false;
        }
        m_name.clear();
        m_sem = -1;
        m_size0 = 0;
        m_size1 = 0;
    }

    bool isOpen() const
    {
        return m_open;
    }
    const std::string &name() const
    {
        return m_name;
    }
    uint32_t size0() const
    {
        return m_size0;
    }
    uint32_t size1() const
    {
        return m_size1;
    }
    uint8_t datatype() const
    {
        return m_datatype;
    }
    const std::string &error() const
    {
        return m_error;
    }
    int lastNonFinite() const
    {
        return m_lastNonFinite;
    }

    /// Write a float image (milkImage layout) and publish (cnt0 + semaphores).
    int write( const mx::improc::eigenImage<float> &im )
    {
        if( !m_open )
        {
            m_error = "write: stream not open";
            return -1;
        }
        if( im.rows() != static_cast<int>( m_size0 ) || im.cols() != static_cast<int>( m_size1 ) )
        {
            m_error = "write size mismatch on " + m_name;
            return -1;
        }

        m_image.md->write = 1;
        void *buffer = nullptr;
        if( ImageStreamIO_writeBuffer( &m_image, &buffer ) != IMAGESTREAMIO_SUCCESS || buffer == nullptr )
        {
            m_image.md->write = 0;
            m_error = "ImageStreamIO_writeBuffer failed for " + m_name;
            return -1;
        }

        const size_t n = static_cast<size_t>( m_size0 ) * static_cast<size_t>( m_size1 );
        const int nbad = copyToBuffer( buffer, im.data(), n );
        if( nbad < 0 )
        {
            m_image.md->write = 0;
            return -1;
        }
        m_lastNonFinite = nbad;

        if( ImageStreamIO_UpdateIm( &m_image ) != IMAGESTREAMIO_SUCCESS )
        {
            m_error = "ImageStreamIO_UpdateIm failed for " + m_name;
            return -1;
        }
        return 0;
    }

    int writeZero()
    {
        mx::improc::eigenImage<float> z;
        z.resize( static_cast<int>( m_size0 ), static_cast<int>( m_size1 ) );
        z.setZero();
        return write( z );
    }

    /// Copy the last-written frame into \p out (converted to float).
    int grabLatest( mx::improc::eigenImage<float> &out )
    {
        if( !m_open )
        {
            m_error = "grabLatest: stream not open";
            return -1;
        }

        void *buffer = nullptr;
        if( ImageStreamIO_readLastWroteBuffer( &m_image, &buffer ) != IMAGESTREAMIO_SUCCESS || buffer == nullptr )
        {
            m_error = "readLastWroteBuffer failed for " + m_name;
            return -1;
        }

        out.resize( static_cast<int>( m_size0 ), static_cast<int>( m_size1 ) );
        const size_t n = static_cast<size_t>( m_size0 ) * static_cast<size_t>( m_size1 );
        return copyFromBuffer( out.data(), buffer, n );
    }

    /// Average \p nframes new camera frames. Optionally skip \p waitFrames first.
    /** \p stop is polled between frames (return true to abort).
      * \returns 0 on success, -1 on error, -2 if stopped.
      */
    int grabMean( unsigned nframes,
                  unsigned waitFrames,
                  const std::function<bool()> &stop,
                  mx::improc::eigenImage<float> &out,
                  double timeout_s = 30.0 )
    {
        if( !m_open )
        {
            m_error = "grabMean: stream not open";
            return -1;
        }
        if( nframes == 0 )
        {
            m_error = "grabMean: nframes == 0";
            return -1;
        }

        mx::improc::eigenImage<float> frame;
        mx::improc::eigenImage<float> acc;
        acc.resize( static_cast<int>( m_size0 ), static_cast<int>( m_size1 ) );
        acc.setZero();

        if( m_sem >= 0 && m_image.md->sem > 0 )
        {
            ImageStreamIO_semflush( &m_image, m_sem );
        }

        unsigned collected = 0;
        unsigned skipped = 0;
        const auto t0 = std::chrono::steady_clock::now();

        while( collected < nframes )
        {
            if( stop && stop() )
            {
                return -2;
            }
            const double elapsed = std::chrono::duration<double>( std::chrono::steady_clock::now() - t0 ).count();
            if( elapsed > timeout_s )
            {
                m_error = "grabMean timed out waiting for " + m_name;
                return -1;
            }

            if( waitNextFrame( 0.2 ) < 0 )
            {
                continue;
            }

            if( grabLatest( frame ) < 0 )
            {
                return -1;
            }

            if( skipped + collected < waitFrames )
            {
                ++skipped;
                continue;
            }

            acc += frame;
            ++collected;
        }

        out = acc / static_cast<float>( nframes );
        return 0;
    }

  private:
    IMAGE m_image{};
    bool m_open{ false };
    std::string m_name;
    std::string m_error;
    uint32_t m_size0{ 0 };
    uint32_t m_size1{ 0 };
    uint8_t m_datatype{ 0 };
    int m_sem{ -1 };
    int m_lastNonFinite{ 0 };

    int copyToBuffer( void *dst, const float *src, size_t n )
    {
        int nbad = 0;
        switch( m_datatype )
        {
        case _DATATYPE_FLOAT:
            for( size_t i = 0; i < n; ++i )
            {
                const float v = src[i];
                if( floatBitsNonFinite( v ) )
                {
                    static_cast<float *>( dst )[i] = 0.0f;
                    ++nbad;
                }
                else
                {
                    static_cast<float *>( dst )[i] = v;
                }
            }
            return nbad;
        case _DATATYPE_DOUBLE:
            for( size_t i = 0; i < n; ++i )
            {
                const float v = src[i];
                if( floatBitsNonFinite( v ) )
                {
                    static_cast<double *>( dst )[i] = 0.0;
                    ++nbad;
                }
                else
                {
                    static_cast<double *>( dst )[i] = static_cast<double>( v );
                }
            }
            return nbad;
        default:
            m_error = "unsupported write datatype on " + m_name;
            return -1;
        }
    }

    int waitNextFrame( double timeout_s )
    {
        if( m_sem < 0 || m_image.md->sem <= 0 )
        {
            timespec req{};
            req.tv_nsec = 10 * 1000 * 1000;
            nanosleep( &req, nullptr );
            return 0;
        }

        timespec ts{};
        if( clock_gettime( CLOCK_REALTIME, &ts ) != 0 )
        {
            return ImageStreamIO_semwait( &m_image, m_sem ) == 0 ? 0 : -1;
        }
        const time_t whole = static_cast<time_t>( timeout_s );
        const long nsec = static_cast<long>( ( timeout_s - static_cast<double>( whole ) ) * 1e9 );
        ts.tv_sec += whole;
        ts.tv_nsec += nsec;
        if( ts.tv_nsec >= 1000000000L )
        {
            ts.tv_sec += 1;
            ts.tv_nsec -= 1000000000L;
        }
        return ImageStreamIO_semtimedwait( &m_image, m_sem, &ts ) == 0 ? 0 : -1;
    }

    int copyFromBuffer( float *dst, const void *src, size_t n )
    {
        auto pix = getPixPointer<float>( static_cast<int>( m_datatype ) );
        if( pix == nullptr )
        {
            m_error = "unsupported read datatype on " + m_name;
            return -1;
        }
        for( size_t i = 0; i < n; ++i )
        {
            dst[i] = pix( const_cast<void *>( src ), i );
        }
        return 0;
    }
};

/// FITS cube of DM modes (zernike, hadamard, SVD, ...).
/** Layout matches milkImage / ImageStreamIO: plane (x,y) with x = size[0].
  */
class modeCube
{
  public:
    std::string name;
    std::string filename;
    mx::improc::eigenCube<float> modes; ///< [size0, size1, nmodes]

    int nModes() const
    {
        return modes.planes();
    }
    int size0() const
    {
        return static_cast<int>( modes.rows() );
    }
    int size1() const
    {
        return static_cast<int>( modes.cols() );
    }

    mx::improc::eigenCube<float>::imageRef image( int i )
    {
        return modes.image( i );
    }

    /// Load a 3D FITS cube. NAXIS1=width (size[0]), NAXIS2=height, NAXIS3=nmodes.
    /** \returns 0 on success, <0 on error.
      */
    int load( const std::string &path, const std::string &setName = "modes" )
    {
        filename = path;
        name = setName;

        fitsfile *fptr = nullptr;
        int status = 0;
        if( fits_open_image( &fptr, path.c_str(), READONLY, &status ) )
        {
            return -1;
        }

        int naxis = 0;
        long naxes[3] = { 0, 0, 0 };
        fits_get_img_dim( fptr, &naxis, &status );
        fits_get_img_size( fptr, 3, naxes, &status );
        if( status != 0 || naxis < 2 )
        {
            fits_close_file( fptr, &status );
            return -2;
        }

        const int width = static_cast<int>( naxes[0] );
        const int height = naxis >= 2 ? static_cast<int>( naxes[1] ) : 1;
        const int nplanes = naxis >= 3 ? static_cast<int>( naxes[2] ) : 1;
        modes.resize( width, height, nplanes );

        std::vector<float> buffer( static_cast<size_t>( width ) * static_cast<size_t>( height ) );
        for( int p = 0; p < nplanes; ++p )
        {
            long fpixel[3] = { 1, 1, p + 1 };
            if( fits_read_pix( fptr, TFLOAT, fpixel, width * height, nullptr, buffer.data(), nullptr, &status ) )
            {
                fits_close_file( fptr, &status );
                return -3;
            }
            // FITS / ImageStreamIO: x varies fastest. Eigen col-major (rows=width) matches.
            std::memcpy( modes.image( p ).data(), buffer.data(), buffer.size() * sizeof( float ) );
        }

        fits_close_file( fptr, &status );
        return 0;
    }

    /// Noll Zernike cube sized to the DM channel. Plane 0 is Noll j=\p minNoll (default 1 = piston).
    int generateZernike( int size0, int size1, int nModes, int minNoll = 1 )
    {
        if( size0 < 2 || size1 < 2 || nModes < 1 )
        {
            return -1;
        }
        name = "zernike";
        filename.clear();
        modes.resize( size0, size1, nModes );
        return mx::sigproc::zernikeBasis<mx::improc::eigenCube<float>, double>( modes, -1, minNoll );
    }

    /// Sylvester Hadamard modes on a circular actuator mask (lina / magpyx convention).
    /** Valid actuators: hypot(x,y) < min(size)/2 + 0.5. Number of planes is the next power of two
      * of the valid-actuator count. Each plane is a 2D map in milkImage layout.
      */
    int generateHadamard( int size0, int size1 )
    {
        if( size0 < 2 || size1 < 2 )
        {
            return -1;
        }

        const double half0 = 0.5 * static_cast<double>( size0 );
        const double half1 = 0.5 * static_cast<double>( size1 );
        const double rad = 0.5 * static_cast<double>( std::min( size0, size1 ) ) + 0.5;
        std::vector<std::pair<int, int>> valid;
        valid.reserve( static_cast<size_t>( size0 * size1 ) );
        for( int y = 0; y < size1; ++y )
        {
            for( int x = 0; x < size0; ++x )
            {
                const double dx = static_cast<double>( x ) - half0 + 0.5;
                const double dy = static_cast<double>( y ) - half1 + 0.5;
                if( std::hypot( dx, dy ) < rad )
                {
                    valid.emplace_back( x, y );
                }
            }
        }
        if( valid.empty() )
        {
            return -2;
        }

        size_t np2 = 1;
        while( np2 < valid.size() )
        {
            np2 <<= 1;
        }

        Eigen::MatrixXd h( 1, 1 );
        h( 0, 0 ) = 1.0;
        while( static_cast<size_t>( h.rows() ) < np2 )
        {
            const int n = static_cast<int>( h.rows() );
            Eigen::MatrixXd next( 2 * n, 2 * n );
            next.topLeftCorner( n, n ) = h;
            next.topRightCorner( n, n ) = h;
            next.bottomLeftCorner( n, n ) = h;
            next.bottomRightCorner( n, n ) = -h;
            h.swap( next );
        }

        name = "hadamard";
        filename.clear();
        modes.resize( size0, size1, static_cast<int>( np2 ) );
        for( int p = 0; p < static_cast<int>( np2 ); ++p )
        {
            modes.image( p ).setZero();
            for( size_t i = 0; i < valid.size(); ++i )
            {
                modes.image( p )( valid[i].first, valid[i].second ) = static_cast<float>( h( p, static_cast<int>( i ) ) );
            }
        }
        return 0;
    }

    /// Replace NaN/Inf with 0. Returns the number of pixels changed.
    int sanitize()
    {
        int n = 0;
        const int npix = static_cast<int>( modes.rows() ) * static_cast<int>( modes.cols() );
        for( int p = 0; p < modes.planes(); ++p )
        {
            float *d = modes.image( p ).data();
            for( int i = 0; i < npix; ++i )
            {
                if( floatBitsNonFinite( d[i] ) )
                {
                    d[i] = 0.0f;
                    ++n;
                }
            }
        }
        return n;
    }
};

/// Replace NaN/Inf with 0. Returns the number of pixels changed.
inline int replaceNonFinite( mx::improc::eigenImage<float> &im )
{
    int n = 0;
    float *d = im.data();
    const int npix = static_cast<int>( im.size() );
    for( int i = 0; i < npix; ++i )
    {
        if( floatBitsNonFinite( d[i] ) )
        {
            d[i] = 0.0f;
            ++n;
        }
    }
    return n;
}

/// Write a float image to FITS (mxlib).
inline int writeFitsImage( const std::string &path, const mx::improc::eigenImage<float> &im )
{
    try
    {
        mx::fits::fitsFile<float> ff;
        return ff.write( path, im );
    }
    catch( ... )
    {
        return -1;
    }
}

/// Read a 2D FITS image into a float eigenImage (mxlib).
inline int readFitsImage( const std::string &path, mx::improc::eigenImage<float> &im )
{
    try
    {
        mx::fits::fitsFile<float> ff;
        return ff.read( im, path );
    }
    catch( ... )
    {
        return -1;
    }
}

/// PSF / image metrics. Core-sum matches magpyx get_image_coresum (negative for minimization).
struct psfMetrics
{
    static mx::improc::eigenImage<float> subtractEdgeMedian( const mx::improc::eigenImage<float> &image )
    {
        mx::improc::eigenImage<float> result = image;
        const int edge = std::min( 5, std::min( static_cast<int>( image.rows() ), static_cast<int>( image.cols() ) ) );
        std::vector<float> edgeVals;
        edgeVals.reserve( static_cast<size_t>( 2 * edge * ( image.rows() + image.cols() ) ) );

        for( int y = 0; y < edge && y < image.cols(); ++y )
        {
            for( int x = 0; x < image.rows(); ++x )
            {
                edgeVals.push_back( image( x, y ) );
                edgeVals.push_back( image( x, image.cols() - 1 - y ) );
            }
        }
        for( int y = edge; y < image.cols() - edge; ++y )
        {
            for( int x = 0; x < edge && x < image.rows(); ++x )
            {
                edgeVals.push_back( image( x, y ) );
                edgeVals.push_back( image( image.rows() - 1 - x, y ) );
            }
        }

        if( !edgeVals.empty() )
        {
            const size_t mid = edgeVals.size() / 2;
            std::nth_element( edgeVals.begin(), edgeVals.begin() + static_cast<std::ptrdiff_t>( mid ), edgeVals.end() );
            result.array() -= edgeVals[mid];
        }
        return result;
    }

    static std::pair<double, double> peakIndex( const mx::improc::eigenImage<float> &image )
    {
        int ix = 0;
        int iy = 0;
        image.maxCoeff( &ix, &iy );
        return { static_cast<double>( ix ), static_cast<double>( iy ) };
    }

    static std::pair<double, double> centerOfMass( const mx::improc::eigenImage<float> &image,
                                                   double cx,
                                                   double cy,
                                                   double radius )
    {
        double sum = 0;
        double sx = 0;
        double sy = 0;
        const double r2 = radius * radius;
        for( int y = 0; y < image.cols(); ++y )
        {
            for( int x = 0; x < image.rows(); ++x )
            {
                const double dx = static_cast<double>( x ) - cx;
                const double dy = static_cast<double>( y ) - cy;
                if( dx * dx + dy * dy > r2 )
                {
                    continue;
                }
                const double v = image( x, y );
                if( v <= 0 )
                {
                    continue;
                }
                sx += static_cast<double>( x ) * v;
                sy += static_cast<double>( y ) * v;
                sum += v;
            }
        }
        if( sum <= 0 )
        {
            return { cx, cy };
        }
        return { sx / sum, sy / sum };
    }

    /// Negative core sum (minimize this to maximize PSF core flux).
    /** If \p cenx / \p ceny are negative, peak + COM refinement is used (magpyx).
      */
    static double coreSum( const mx::improc::eigenImage<float> &image,
                           double radius,
                           double cenx = -1.0,
                           double ceny = -1.0 )
    {
        mx::improc::eigenImage<float> bg = subtractEdgeMedian( image );

        double cx = cenx;
        double cy = ceny;
        if( cx < 0 || cy < 0 )
        {
            auto pk = peakIndex( bg );
            auto com = centerOfMass( bg, pk.first, pk.second, 2.0 * radius );
            cx = com.first;
            cy = com.second;
        }

        double sum = 0;
        const double r2 = radius * radius;
        for( int y = 0; y < bg.cols(); ++y )
        {
            for( int x = 0; x < bg.rows(); ++x )
            {
                const double dx = static_cast<double>( x ) - cx;
                const double dy = static_cast<double>( y ) - cy;
                if( dx * dx + dy * dy <= r2 )
                {
                    sum += bg( x, y );
                }
            }
        }
        return -sum;
    }
};

/// Least-squares quadratic vertex. Returns false if not a minimum inside \p [lo, hi].
inline bool quadraticMinimaOk( const std::vector<double> &x,
                                 const std::vector<double> &y,
                                 double lo,
                                 double hi,
                                 double &vertexOut )
{
    if( x.size() < 3 || x.size() != y.size() )
    {
        return false;
    }

    const int n = static_cast<int>( x.size() );
    Eigen::MatrixXd A( n, 3 );
    Eigen::VectorXd b( n );
    for( int i = 0; i < n; ++i )
    {
        A( i, 0 ) = x[static_cast<size_t>( i )] * x[static_cast<size_t>( i )];
        A( i, 1 ) = x[static_cast<size_t>( i )];
        A( i, 2 ) = 1.0;
        const double yi = y[static_cast<size_t>( i )];
        b( i ) = doubleBitsNonFinite( yi ) ? 0.0 : yi;
    }

    const Eigen::Vector3d c = A.colPivHouseholderQr().solve( b );
    if( doubleBitsNonFinite( c( 0 ) ) || doubleBitsNonFinite( c( 1 ) ) || !( c( 0 ) > 0 ) )
    {
        return false;
    }

    const double vertex = -c( 1 ) / ( 2.0 * c( 0 ) );
    if( doubleBitsNonFinite( vertex ) || vertex < lo || vertex > hi )
    {
        return false;
    }
    vertexOut = vertex;
    return true;
}

/// Least-squares quadratic vertex. Returns 0 if the fit is not a minimum inside \p [lo, hi].
inline double quadraticMinima( const std::vector<double> &x,
                                 const std::vector<double> &y,
                                 double lo,
                                 double hi )
{
    double v = 0;
    return quadraticMinimaOk( x, y, lo, hi, v ) ? v : 0.0;
}

/// Mean of per-repeat argmin amplitudes.
inline double meanArgmin( const std::vector<double> &steps,
                          const std::vector<std::vector<double>> &curves )
{
    if( steps.empty() || curves.empty() )
    {
        return 0.0;
    }
    double acc = 0;
    int n = 0;
    for( const auto &row : curves )
    {
        if( row.size() != steps.size() )
        {
            continue;
        }
        const auto it = std::min_element( row.begin(), row.end() );
        acc += steps[static_cast<size_t>( std::distance( row.begin(), it ) )];
        ++n;
    }
    return n > 0 ? acc / n : 0.0;
}

/// One metric sample: coresum (minimize) plus image peak, used to drop blank frames.
struct metricSample
{
    double metric{ 0 };
    double peak{ 0 };
};

inline std::vector<double> amplitudeGrid( double lo, double hi, int nSteps )
{
    nSteps = std::max( 2, nSteps );
    std::vector<double> steps( static_cast<size_t>( nSteps ) );
    for( int i = 0; i < nSteps; ++i )
    {
        const double t = static_cast<double>( i ) / static_cast<double>( nSteps - 1 );
        steps[static_cast<size_t>( i )] = lo + t * ( hi - lo );
    }
    return steps;
}

/// Contiguous on-camera island around the best (lowest-metric) amplitude.
struct onCameraWindow
{
    int i0{ 0 };
    int i1{ -1 };
    int best{ 0 };
    int nGood{ 0 };
    int nTotal{ 0 };
    double bestMetric{ 0 };
    double bestAmp{ 0 };
    bool truncated{ false };
};

/// Drop amplitudes whose mean peak is below \p blankThresh (or 10% of the sweep max if 0).
inline onCameraWindow findOnCameraWindow( const std::vector<double> &steps,
                                          const std::vector<std::vector<double>> &metrics,
                                          const std::vector<std::vector<double>> &peaks,
                                          double blankThresh )
{
    onCameraWindow w;
    const int nS = static_cast<int>( steps.size() );
    const int nR = static_cast<int>( peaks.size() );
    w.nTotal = nS;
    if( nS < 1 || nR < 1 || static_cast<int>( metrics.size() ) != nR )
    {
        return w;
    }

    std::vector<double> meanM( static_cast<size_t>( nS ), 0.0 );
    std::vector<double> meanP( static_cast<size_t>( nS ), 0.0 );
    double maxPeak = 0;
    for( int s = 0; s < nS; ++s )
    {
        int n = 0;
        for( int r = 0; r < nR; ++r )
        {
            if( static_cast<int>( peaks[static_cast<size_t>( r )].size() ) != nS ||
                static_cast<int>( metrics[static_cast<size_t>( r )].size() ) != nS )
            {
                continue;
            }
            meanM[static_cast<size_t>( s )] += metrics[static_cast<size_t>( r )][static_cast<size_t>( s )];
            meanP[static_cast<size_t>( s )] += peaks[static_cast<size_t>( r )][static_cast<size_t>( s )];
            if( peaks[static_cast<size_t>( r )][static_cast<size_t>( s )] > maxPeak )
            {
                maxPeak = peaks[static_cast<size_t>( r )][static_cast<size_t>( s )];
            }
            ++n;
        }
        if( n > 0 )
        {
            meanM[static_cast<size_t>( s )] /= static_cast<double>( n );
            meanP[static_cast<size_t>( s )] /= static_cast<double>( n );
        }
    }

    const double thresh = blankThresh > 0.0 ? blankThresh : 0.1 * maxPeak;
    if( !( thresh > 0.0 ) )
    {
        return w;
    }

    std::vector<char> good( static_cast<size_t>( nS ), 0 );
    int nMarked = 0;
    for( int s = 0; s < nS; ++s )
    {
        if( meanP[static_cast<size_t>( s )] >= thresh )
        {
            good[static_cast<size_t>( s )] = 1;
            ++nMarked;
        }
    }
    if( nMarked < 1 )
    {
        return w;
    }

    int best = -1;
    for( int s = 0; s < nS; ++s )
    {
        if( !good[static_cast<size_t>( s )] )
        {
            continue;
        }
        if( best < 0 || meanM[static_cast<size_t>( s )] < meanM[static_cast<size_t>( best )] ||
            ( meanM[static_cast<size_t>( s )] == meanM[static_cast<size_t>( best )] &&
              meanP[static_cast<size_t>( s )] > meanP[static_cast<size_t>( best )] ) )
        {
            best = s;
        }
    }
    if( best < 0 )
    {
        return w;
    }

    int i0 = best;
    int i1 = best;
    while( i0 > 0 && good[static_cast<size_t>( i0 - 1 )] )
    {
        --i0;
    }
    while( i1 + 1 < nS && good[static_cast<size_t>( i1 + 1 )] )
    {
        ++i1;
    }

    w.i0 = i0;
    w.i1 = i1;
    w.best = best;
    w.nGood = i1 - i0 + 1;
    w.bestMetric = meanM[static_cast<size_t>( best )];
    w.bestAmp = steps[static_cast<size_t>( best )];
    w.truncated = ( i0 > 0 || i1 < nS - 1 );
    return w;
}

inline void flattenWindow( const std::vector<double> &steps,
                            const std::vector<std::vector<double>> &curves,
                            int i0,
                            int i1,
                            std::vector<double> &x,
                            std::vector<double> &y )
{
    x.clear();
    y.clear();
    if( i1 < i0 || steps.empty() )
    {
        return;
    }
    x.reserve( static_cast<size_t>( ( i1 - i0 + 1 ) * static_cast<int>( curves.size() ) ) );
    y.reserve( x.capacity() );
    for( int s = i0; s <= i1; ++s )
    {
        for( const auto &row : curves )
        {
            if( static_cast<int>( row.size() ) != static_cast<int>( steps.size() ) )
            {
                continue;
            }
            x.push_back( steps[static_cast<size_t>( s )] );
            y.push_back( row[static_cast<size_t>( s )] );
        }
    }
}

inline void sliceWindow( const std::vector<double> &steps,
                           const std::vector<std::vector<double>> &curves,
                           int i0,
                           int i1,
                           std::vector<double> &st,
                           std::vector<std::vector<double>> &cu )
{
    st.clear();
    cu.clear();
    if( i1 < i0 )
    {
        return;
    }
    const int n = i1 - i0 + 1;
    st.resize( static_cast<size_t>( n ) );
    for( int i = 0; i < n; ++i )
    {
        st[static_cast<size_t>( i )] = steps[static_cast<size_t>( i0 + i )];
    }
    cu.resize( curves.size() );
    for( size_t r = 0; r < curves.size(); ++r )
    {
        cu[r].resize( static_cast<size_t>( n ), 0.0 );
        if( static_cast<int>( curves[r].size() ) != static_cast<int>( steps.size() ) )
        {
            continue;
        }
        for( int i = 0; i < n; ++i )
        {
            cu[r][static_cast<size_t>( i )] = curves[r][static_cast<size_t>( i0 + i )];
        }
    }
}

/// Sweep amplitude of one mode, measure a scalar metric, return the minimizing amplitude.
/** If the PSF walks off the camera at large amplitudes, blank samples are dropped
  * and the quadratic is fit on the remaining contiguous island. If that fit is
  * still rejected, a finer sweep is taken around the best on-camera sample.
  */
struct gridSweep
{
    double lo{ -0.05 };
    double hi{ 0.05 };
    int nSteps{ 20 };
    int nRepeats{ 3 };
    std::string kind{ "fit" }; ///< "fit" or "mean"
    double blankThresh{ 0 };  ///< Peak ADU treated as blank. 0 = 10% of the sweep's max peak.
    int nRefine{ 1 };         ///< Extra finer sweeps around the best sample if the quadratic fails.

    struct result
    {
        double amp{ 0 };
        bool usedFit{ false };
        bool truncated{ false };
        bool refined{ false };
        bool stopped{ false };
        int nGood{ 0 };
        int nTotal{ 0 };
    };

    /// \p apply(amp) writes the DM; \p measure() returns \ref metricSample (minimize metric).
    template <typename Apply, typename Measure>
    result run( Apply &&apply, Measure &&measure, const std::function<bool()> &stop = {} ) const
    {
        result out;
        if( nSteps < 2 || nRepeats < 1 )
        {
            return out;
        }

        const double span = hi - lo;
        double clo = lo;
        double chi = hi;
        double fallbackAmp = 0;
        double fallbackMetric = 0;

        auto collect = [&]( double a0, double a1, std::vector<double> &steps,
                             std::vector<std::vector<double>> &metrics,
                             std::vector<std::vector<double>> &peaks ) -> int {
            steps = amplitudeGrid( a0, a1, nSteps );
            const int nS = static_cast<int>( steps.size() );
            metrics.assign( static_cast<size_t>( nRepeats ),
                            std::vector<double>( static_cast<size_t>( nS ), 0.0 ) );
            peaks.assign( static_cast<size_t>( nRepeats ),
                          std::vector<double>( static_cast<size_t>( nS ), 0.0 ) );
            for( int r = 0; r < nRepeats; ++r )
            {
                for( int s = 0; s < nS; ++s )
                {
                    if( stop && stop() )
                    {
                        return -2;
                    }
                    if( apply( steps[static_cast<size_t>( s )] ) < 0 )
                    {
                        return -1;
                    }
                    metricSample sm{};
                    sm = measure();
                    metrics[static_cast<size_t>( r )][static_cast<size_t>( s )] = finiteOrZero( sm.metric );
                    peaks[static_cast<size_t>( r )][static_cast<size_t>( s )] =
                        finiteOrZero( sm.peak ) < 0.0 ? 0.0 : finiteOrZero( sm.peak );
                }
            }
            return 0;
        };

        for( int pass = 0; pass <= std::max( 0, nRefine ); ++pass )
        {
            std::vector<double> steps;
            std::vector<std::vector<double>> metrics;
            std::vector<std::vector<double>> peaks;
            const int crv = collect( clo, chi, steps, metrics, peaks );
            if( crv < 0 )
            {
                apply( 0.0 );
                out.stopped = ( crv == -2 );
                return out;
            }

            const onCameraWindow win = findOnCameraWindow( steps, metrics, peaks, blankThresh );
            if( pass == 0 )
            {
                out.nGood = win.nGood;
                out.nTotal = win.nTotal;
                out.truncated = win.truncated;
            }
            else
            {
                out.refined = true;
                if( win.truncated )
                {
                    out.truncated = true;
                }
            }

            if( win.nGood < 1 )
            {
                apply( 0.0 );
                out.amp = ( fallbackMetric < 0.0 ) ? finiteOrZero( fallbackAmp ) : 0.0;
                return out;
            }
            fallbackAmp = win.bestAmp;
            fallbackMetric = win.bestMetric;

            if( kind == "mean" )
            {
                std::vector<double> st;
                std::vector<std::vector<double>> cu;
                sliceWindow( steps, metrics, win.i0, win.i1, st, cu );
                apply( 0.0 );
                out.amp = finiteOrZero( meanArgmin( st, cu ) );
                return out;
            }

            if( win.nGood >= 3 )
            {
                std::vector<double> x;
                std::vector<double> y;
                flattenWindow( steps, metrics, win.i0, win.i1, x, y );
                double vertex = 0;
                if( quadraticMinimaOk( x, y, steps[static_cast<size_t>( win.i0 )],
                                          steps[static_cast<size_t>( win.i1 )], vertex ) )
                {
                    apply( 0.0 );
                    out.amp = finiteOrZero( vertex );
                    out.usedFit = true;
                    return out;
                }
            }

            // Fit rejected: either refine around the best on-camera sample, or take it.
            if( pass < nRefine )
            {
                const double half = 0.25 * span;
                clo = std::max( lo, win.bestAmp - half );
                chi = std::min( hi, win.bestAmp + half );
                if( !( chi > clo ) )
                {
                    apply( 0.0 );
                    out.amp = ( win.bestMetric < 0.0 ) ? finiteOrZero( win.bestAmp ) : 0.0;
                    return out;
                }
                continue;
            }

            apply( 0.0 );
            out.amp = ( win.bestMetric < 0.0 ) ? finiteOrZero( win.bestAmp ) : 0.0;
            return out;
        }

        apply( 0.0 );
        return out;
    }
};

/// Hook-up for cacao DM channels + WFS camera.
/** Two working channels so dmcomb can sum a running eye-doctor command with
  * temporary grid-search pokes:
  *  - \c dmEyeDoc  : accumulated modal command (starts empty)
  *  - \c dmSweep   : temporary pokes (zeroed after each mode)
  *  - \c dmFlat    : cacao flat channel (typically disp00)
  *  - \c dmSum     : cacao total (dmXXdisp)
  *
  * If sweep name is empty or equal to eyeDoc, a single working channel is used.
  */
class wavefrontHardware
{
  public:
    std::string dmEyeDocName; ///< Accumulated eye-doctor command
    std::string dmSweepName;  ///< Grid-search poke channel
    std::string dmFlatName;   ///< Flat channel
    std::string dmSumName;    ///< Summed / total DM command
    std::string camName;      ///< Camera / WFS image shmim
    std::string camDevice;    ///< INDI device of the camera

    shmimStream dmEyeDoc;
    shmimStream dmSweep;
    shmimStream dmFlat;
    shmimStream dmSum;
    shmimStream cam;

    bool singleChannel() const
    {
        return dmSweepName.empty() || dmSweepName == dmEyeDocName;
    }

    int connectAlgoChannels()
    {
        if( dmEyeDoc.open( dmEyeDocName ) < 0 )
        {
            return -1;
        }
        if( !singleChannel() )
        {
            if( dmSweep.open( dmSweepName ) < 0 )
            {
                return -1;
            }
        }
        return 0;
    }

    int connectLoop()
    {
        if( connectAlgoChannels() < 0 )
        {
            return -1;
        }
        if( cam.open( camName ) < 0 )
        {
            return -1;
        }
        return 0;
    }

    int connectFlatSave()
    {
        if( dmSum.open( dmSumName ) < 0 )
        {
            return -1;
        }
        if( dmFlat.open( dmFlatName ) < 0 )
        {
            return -1;
        }
        if( !dmEyeDoc.isOpen() && dmEyeDoc.open( dmEyeDocName ) < 0 )
        {
            return -1;
        }
        if( !singleChannel() && !dmSweep.isOpen() && dmSweep.open( dmSweepName ) < 0 )
        {
            return -1;
        }
        return 0;
    }

    void disconnect()
    {
        dmEyeDoc.close();
        dmSweep.close();
        dmFlat.close();
        dmSum.close();
        cam.close();
    }

    const std::string &error() const
    {
        if( !dmEyeDoc.error().empty() )
        {
            return dmEyeDoc.error();
        }
        if( !dmSweep.error().empty() )
        {
            return dmSweep.error();
        }
        if( !dmFlat.error().empty() )
        {
            return dmFlat.error();
        }
        if( !dmSum.error().empty() )
        {
            return dmSum.error();
        }
        return cam.error();
    }

    int grabEyeDoc( mx::improc::eigenImage<float> &cmd )
    {
        return dmEyeDoc.grabLatest( cmd );
    }

    int writeEyeDoc( const mx::improc::eigenImage<float> &cmd )
    {
        return dmEyeDoc.write( cmd );
    }

    int zeroEyeDoc()
    {
        return dmEyeDoc.writeZero();
    }

    int zeroSweep()
    {
        if( singleChannel() )
        {
            return 0;
        }
        return dmSweep.writeZero();
    }

    int applySweep( const mx::improc::eigenImage<float> &mode,
                    double amp,
                    const mx::improc::eigenImage<float> *eyeDocCmd )
    {
        if( doubleBitsNonFinite( amp ) )
        {
            amp = 0.0;
        }
        mx::improc::eigenImage<float> cmd = mode * static_cast<float>( amp );
        replaceNonFinite( cmd );
        if( singleChannel() )
        {
            if( eyeDocCmd != nullptr )
            {
                cmd += *eyeDocCmd;
                replaceNonFinite( cmd );
            }
            return dmEyeDoc.write( cmd );
        }
        return dmSweep.write( cmd );
    }
};

} // namespace dev
} // namespace app
} // namespace MagAOX

#endif // dmWavefrontControl_hpp
