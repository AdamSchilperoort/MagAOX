# Makefile for eyeDoctor

allall: all 

COMMON_INC := ../common/include
CXXFLAGS += -I$(COMMON_INC)

OTHER_HEADERS=$(COMMON_INC)/lina/dark_library.h
TARGET=eyeDoctor

# Disable user check to allow running as adamschilperoort
CXXFLAGS += -DXWC_DISABLE_USER_CHECK

include ../../Make/magAOXApp.mk
