# Source and include directories
INCDIR =
INCDIROUT := include
ALLINC = 

SRCDIRBASE := Base/src
INCDIRBASE := Base/include
INCDIR += $(INCDIRBASE)
ALLINC += $(wildcard $(INCDIRBASE)/*.hh)

SRCDIRCOLL := CollisionDetection/src
INCDIRCOLL := CollisionDetection/include
INCDIR += $(INCDIRCOLL)
ALLINC += $(wildcard $(INCDIRCOLL)/*.hh)

SRCDIRCOMP := Component/src
INCDIRCOMP := Component/include
INCDIR += $(INCDIRCOMP)
ALLINC += $(wildcard $(INCDIRCOMP)/*.hh)

SRCDIRCONT := ContactForce/src
INCDIRCONT := ContactForce/include
INCDIR += $(INCDIRCONT)
ALLINC += $(wildcard $(INCDIRCONT)/*.hh)

SRCDIRGEOM := Geometry/src
INCDIRGEOM := Geometry/include
INCDIR += $(INCDIRGEOM)
ALLINC += $(wildcard $(INCDIRGEOM)/*.hh)

SRCDIRPP := PostProcessing/src
INCDIRPP := PostProcessing/include
INCDIR += $(INCDIRPP)
ALLINC += $(wildcard $(INCDIRPP)/*.hh)

SRCDIRTIME := TimeIntegration/src
INCDIRTIME := TimeIntegration/include
INCDIR += $(INCDIRTIME)
ALLINC += $(wildcard $(INCDIRTIME)/*.hh)

SRCDIRXML := XML/src
INCDIRXML := XML/include
INCDIR += $(INCDIRXML)
ALLINC += $(wildcard $(INCDIRXML)/*.hh)



# Source files per package
# _WP_ means "with path"
SRCFILES_WP_BASE := $(wildcard $(SRCDIRBASE)/*.cpp)
SRCFILESBASE := $(subst $(SRCDIRBASE)/,,$(SRCFILES_WP_BASE))

SRCFILES_WP_COLL := $(wildcard $(SRCDIRCOLL)/*.cpp)
SRCFILESCOLL := $(subst $(SRCDIRCOLL)/,,$(SRCFILES_WP_COLL))

SRCFILES_WP_COMP := $(wildcard $(SRCDIRCOMP)/*.cpp)
SRCFILESCOMP := $(subst $(SRCDIRCOMP)/,,$(SRCFILES_WP_COMP))

SRCFILES_WP_CONT := $(wildcard $(SRCDIRCONT)/*.cpp)
SRCFILESCONT := $(subst $(SRCDIRCONT)/,,$(SRCFILES_WP_CONT))

SRCFILES_WP_GEOM := $(wildcard $(SRCDIRGEOM)/*.cpp)
SRCFILESGEOM := $(subst $(SRCDIRGEOM)/,,$(SRCFILES_WP_GEOM))

SRCFILES_WP_PP := $(wildcard $(SRCDIRPP)/*.cpp)
SRCFILESPP := $(subst $(SRCDIRPP)/,,$(SRCFILES_WP_PP))

SRCFILES_WP_TIME := $(wildcard $(SRCDIRTIME)/*.cpp)
SRCFILESTIME := $(subst $(SRCDIRTIME)/,,$(SRCFILES_WP_TIME))

SRCFILES_WP_XML := $(wildcard $(SRCDIRXML)/*.cpp)
SRCFILESXML := $(subst $(SRCDIRXML)/,,$(SRCFILES_WP_XML))


# Target directories
OBJDIR := obj$(GRAINS_FULL_EXT)
LIBDIR := lib$(GRAINS_FULL_EXT)


# Substitutions for objects
# Here we use the base file names combined to VPATH
OBJS_BASE := $(SRCFILESBASE:%.cpp=$(OBJDIR)/%.o)
OBJS_COLL := $(SRCFILESCOLL:%.cpp=$(OBJDIR)/%.o)
OBJS_COMP := $(SRCFILESCOMP:%.cpp=$(OBJDIR)/%.o)
OBJS_CONT := $(SRCFILESCONT:%.cpp=$(OBJDIR)/%.o)
OBJS_GEOM := $(SRCFILESGEOM:%.cpp=$(OBJDIR)/%.o)
OBJS_PP   := $(SRCFILESPP:%.cpp=$(OBJDIR)/%.o)
OBJS_TIME := $(SRCFILESTIME:%.cpp=$(OBJDIR)/%.o)
OBJS_XML  := $(SRCFILESXML:%.cpp=$(OBJDIR)/%.o)


# Defining VPATH for automatic search of source files in the 
# different source directories 
# This enables to have a single rule/target for all source files in 
# various directories and all objects in a single directory
VPATH := $(SRCDIRBASE):$(SRCDIRCOLL):$(SRCDIRCOMP):$(SRCDIRCONT):$(SRCDIRGEOM):$(SRCDIRPP):$(SRCDIRTIME):$(SRCDIRXML)


# Compiler
INCFLAGS  := $(INCDIR:%=-I%) $(GRAINS_XERCES_INCDIR:%=-I%)
COMPFLAGS := $(GRAINS_GPU_COMPILER_FLAGS) $(INCFLAGS)
LINKFLAGS := $(GRAINS_GPU_LINKER_FLAGS) $(GRAINS_Z_FLAGS) $(GRAINS_XERCES_FLAGS)


#------------------------------------------------------------------------------
# Production rules
.SILENT:

all : grains

grains : MESS CPINC BASE COLL COMP CONT GEOM PP TIME XML
		$(GRAINS_GPU_COMPILER) -shared -Xcompiler \"-Wl,-soname,libgrains.so\" \
		$(LINKFLAGS) \
		-o $(LIBDIR)/libgrains.so \
		$(OBJS_BASE) $(OBJS_COLL) $(OBJS_COMP) $(OBJS_CONT) $(OBJS_GEOM) \
		$(OBJS_PP) $(OBJS_TIME) $(OBJS_XML) 
	@echo 'Compilation is complete'

MESS   :
	@echo '***********************************************************'
	@echo 'GRAINS compiling with'\
	$(GRAINS_CPP_COMPILER_DIST)-$(GRAINS_CPP_COMPILER_VERSION)\
	', and' \
	$(GRAINS_GPU_COMPILER_DIST)-$(GRAINS_GPU_COMPILER_VERSION)
	@echo '***********************************************************'

CPINC  : 
	@echo 'Copy include files from subdirectories to' $(INCDIROUT)
	rm -f $(INCDIROUT)/*
	cp $(ALLINC) $(INCDIROUT)/ 	

BASE  : $(OBJS_BASE)
	@echo '*** Package Base compiled'
	@echo

COLL  : $(OBJS_COLL)
	@echo '*** Package Collision compiled'
	@echo

COMP  : $(OBJS_COMP)
	@echo '*** Package Component compiled'
	@echo

CONT  : $(OBJS_CONT)
	@echo '*** Package Contact compiled'
	@echo
	
GEOM  : $(OBJS_GEOM)
	@echo '*** Package Geometry compiled'
	@echo

PP    : $(OBJS_PP)
	@echo '*** Package PostProcessing compiled'
	@echo

TIME  : $(OBJS_TIME)
	@echo '*** Package TimeIntegration compiled'
	@echo

XML   : $(OBJS_XML)
	@echo '*** Package XML compiled'
	@echo


$(OBJDIR)/%.o : %.cpp
	@echo 'Compiling '$<
	$(GRAINS_GPU_COMPILER) $(COMPFLAGS) -D_XML $< -o $@


# DEPENDENCIES
DEPEND  := $(INCDIR:%=-I%) $(GRAINS_SYSINC:%=-I%)

depend:
	@echo 'Generating dependencies'
	touch ./Maketmp
	makedepend -f ./Maketmp $(DEPEND) \
	$(SRCFILES_WP_BASE) $(SRCFILES_WP_COLL) $(SRCFILES_WP_COMP) \
	$(SRCFILES_WP_CONT) $(SRCFILES_WP_GEOM) $(SRCFILES_WP_PP) \ 
	$(SRCFILES_WP_TIME) $(SRCFILES_WP_XML) 
	rm -f ./Maketmp ./Maketmp.bak

# CLEAN
clean : 
	@echo 'Grains : Deleting object and library files ...'
	rm -f $(INCDIROUT)/*
	rm -f $(OBJDIR)/*.o
	rm -f $(LIBDIR)/*
	@echo 'Grains : Deleted.'