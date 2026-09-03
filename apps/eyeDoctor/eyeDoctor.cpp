/** \file eyeDoctor.cpp
  * \brief The MagAO-X Eye Doctor implementation file
  *
  * \ingroup eyeDoctor_files
  */

#include "eyeDoctor.hpp"

int main(int argc, char **argv)
{
    MagAOX::app::eyeDoctor app;

    return app.main(argc, argv);
}
