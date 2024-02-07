# Source and include directories
INCDIR =
INCDIROUT := include
ALLINC = 

SRCDIRBASE := Base/src
INCDIRBASE := Base/inc
INCDIR += $(INCDIRBASE)
ALLINC += $(wildcard $(INCDIRBASE)/*.hh)

SRCDIRCOMPO := Component/src
INCDIRCOMPO := Component/inc
INCDIR += $(INCDIRCOMPO)
ALLINC += $(wildcard $(INCDIRCOMPO)/*.hh)

SRCDIRGEOM := Geometry/src
INCDIRGEOM := Geometry/inc
INCDIR += $(INCDIRGEOM)
ALLINC += $(wildcard $(INCDIRGEOM)/*.hh)


# Source files per package
# _WP_ means "with path"
SRCFILES_WP_BASE := $(wildcard $(SRCDIRBASE)/*.cpp)
SRCFILESBASE := $(subst $(SRCDIRBASE)/,,$(SRCFILES_WP_BASE))

SRCFILES_WP_COMPO := $(wildcard $(SRCDIRCOMPO)/*.cpp)
SRCFILESCOMPO := $(subst $(SRCDIRCOMPO)/,,$(SRCFILES_WP_COMPO))

SRCFILES_WP_GEOM := $(wildcard $(SRCDIRGEOM)/*.cpp)
SRCFILESGEOM := $(subst $(SRCDIRGEOM)/,,$(SRCFILES_WP_GEOM))


# Target directories
OBJDIR := obj


# Substitutions for objects
# Here we use the base file names combined to VPATH
OBJS_BASE := $(SRCFILESBASE:%.cpp=$(OBJDIR)/%.o)
OBJS_COMPO := $(SRCFILESCOMPO:%.cpp=$(OBJDIR)/%.o)
OBJS_GEOM := $(SRCFILESGEOM:%.cpp=$(OBJDIR)/%.o)

# Defining VPATH for automatic search of source files in the 
# different source directories 
# This enables to have a single rule/target for all source files in 
# various directories and all objects in a single directory
VPATH := $(SRCDIRBASE):$(SRCDIRCOMPO):$(SRCDIRGEOM)


# Compiler
COMP  := nvcc
LINK  := nvcc
DEBUG  := -g


# Compiler flags
INCFLAGS  := $(INCDIR:%=-I%)
COMPFLAGS := $(DEBUG) -x cu -O3 -arch=sm_75 $(INCFLAGS)


#------------------------------------------------------------------------------
# Production rules
.SILENT:

all : grains

grains : MESS CPINC BASE COMPO GEOM
	$(LINK) -arch=sm_75 $(OBJS_GEOM) $(OBJS_COMPO) $(OBJS_BASE) -lcudart -o grains
	@echo 'Compilation is complete'

MESS   :
	@echo '***********************************************************'
	@echo 'GRAINS compiling with nvcc'
	@echo '***********************************************************'

CPINC  : 
	@echo 'Copy include files from subdirectories to' $(INCDIROUT)
	rm -f $(INCDIROUT)/*
	cp $(ALLINC) $(INCDIROUT)/ 	

BASE   : $(OBJS_BASE)
	@echo '*** Package Base compiled'
	@echo

COMPO  : $(OBJS_COMPO)
	@echo '*** Package Component compiled'
	@echo

GEOM   : $(OBJS_GEOM)
	@echo '*** Package Geometry compiled'
	@echo

$(OBJDIR)/%.o : %.cpp
	@echo 'Compiling '$<
	$(COMP) $(COMPFLAGS) $(MACRO) -dc $< -o $@	

# CLEAN
clean : 
	@echo 'Grains : Deleting object and library files'
	rm -f $(INCDIROUT)/*
	rm -f $(OBJDIR)/*.o
