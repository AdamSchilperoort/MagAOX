/** \file eyeDoctor.cpp
  * \brief The MagAO-X Eye Doctor implementation file
  *
  * \ingroup eyeDoctor_files
  */

#include "eyeDoctor.hpp"

// Explicit template instantiation for dmWavefrontControl
template class MagAOX::app::dev::dmWavefrontControl<MagAOX::app::eyeDoctor>;

int main(int argc, char **argv)
{
    MagAOX::app::eyeDoctor app;

    return app.main(argc, argv);
}
