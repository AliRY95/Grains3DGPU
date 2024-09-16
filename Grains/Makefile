# Source and include directories
INCDIR =
INCDIROUT := include
ALLINC = 

SRCDIRBASE := Base/src
INCDIRBASE := Base/include
INCDIR += $(INCDIRBASE)
ALLINC += $(wildcard $(INCDIRBASE)/*.hh)

SRCDIRCOMPONENT := Component/src
INCDIRCOMPONENT := Component/include
INCDIR += $(INCDIRCOMPONENT)
ALLINC += $(wildcard $(INCDIRCOMPONENT)/*.hh)

SRCDIRCOLLISION := CollisionDetection/src
INCDIRCOLLISION := CollisionDetection/include
INCDIR += $(INCDIRCOLLISION)
ALLINC += $(wildcard $(INCDIRCOLLISION)/*.hh)

SRCDIRGEOMETRY := Geometry/src
INCDIRGEOMETRY := Geometry/include
INCDIR += $(INCDIRGEOMETRY)
ALLINC += $(wildcard $(INCDIRGEOMETRY)/*.hh)

SRCDIRTIME := TimeIntegration/src
INCDIRTIME := TimeIntegration/include
INCDIR += $(INCDIRTIME)
ALLINC += $(wildcard $(INCDIRTIME)/*.hh)

SRCDIRXML := XML/src
INCDIRXML := XML/include
INCDIR += $(INCDIRXML)
ALLINC += $(wildcard $(INCDIRXML)/*.hh)

SRCDIRPP := PostProcessing/src
INCDIRPP := PostProcessing/include
INCDIR += $(INCDIRPP)
ALLINC += $(wildcard $(INCDIRPP)/*.hh)


# Source files per package
# _WP_ means "with path"
SRCFILES_WP_BASE := $(wildcard $(SRCDIRBASE)/*.cpp)
SRCFILESBASE := $(subst $(SRCDIRBASE)/,,$(SRCFILES_WP_BASE))

SRCFILES_WP_COMPONENT := $(wildcard $(SRCDIRCOMPONENT)/*.cpp)
SRCFILESCOMPONENT := $(subst $(SRCDIRCOMPONENT)/,,$(SRCFILES_WP_COMPONENT))

SRCFILES_WP_COLLISION := $(wildcard $(SRCDIRCOLLISION)/*.cpp)
SRCFILESCOLLISION := $(subst $(SRCDIRCOLLISION)/,,$(SRCFILES_WP_COLLISION))

SRCFILES_WP_GEOMETRY := $(wildcard $(SRCDIRGEOMETRY)/*.cpp)
SRCFILESGEOMETRY := $(subst $(SRCDIRGEOMETRY)/,,$(SRCFILES_WP_GEOMETRY))

SRCFILES_WP_TIME := $(wildcard $(SRCDIRTIME)/*.cpp)
SRCFILESTIME := $(subst $(SRCDIRTIME)/,,$(SRCFILES_WP_TIME))

SRCFILES_WP_XML := $(wildcard $(SRCDIRXML)/*.cpp)
SRCFILESXML := $(subst $(SRCDIRXML)/,,$(SRCFILES_WP_XML))

SRCFILES_WP_PP := $(wildcard $(SRCDIRPP)/*.cpp)
SRCFILESPP := $(subst $(SRCDIRPP)/,,$(SRCFILES_WP_PP))


# Target directories
OBJDIR := obj
LIBDIR := lib


# Substitutions for objects
# Here we use the base file names combined to VPATH
OBJS_BASE := $(SRCFILESBASE:%.cpp=$(OBJDIR)/%.o)
OBJS_COMPONENT := $(SRCFILESCOMPONENT:%.cpp=$(OBJDIR)/%.o)
OBJS_COLLISION := $(SRCFILESCOLLISION:%.cpp=$(OBJDIR)/%.o)
OBJS_GEOMETRY := $(SRCFILESGEOMETRY:%.cpp=$(OBJDIR)/%.o)
OBJS_TIME := $(SRCFILESTIME:%.cpp=$(OBJDIR)/%.o)
OBJS_XML := $(SRCFILESXML:%.cpp=$(OBJDIR)/%.o)
OBJS_PP := $(SRCFILESPP:%.cpp=$(OBJDIR)/%.o)


# Defining VPATH for automatic search of source files in the 
# different source directories 
# This enables to have a single rule/target for all source files in 
# various directories and all objects in a single directory
VPATH := $(SRCDIRBASE):$(SRCDIRCOMPONENT):$(SRCDIRCOLLISION):$(SRCDIRGEOMETRY):$(SRCDIRTIME):$(SRCDIRXML):$(SRCDIRPP)


# Compiler
INCFLAGS  := $(INCDIR:%=-I%) $(GRAINS_XERCES_INCDIR:%=-I%)
COMPFLAGS := $(GRAINS_GPU_COMPILER_FLAGS) $(INCFLAGS)
LINKFLAGS := $(GRAINS_GPU_LINKER_FLAGS) $(GRAINS_Z_FLAGS) $(GRAINS_XERCES_FLAGS)


#------------------------------------------------------------------------------
# Production rules
.SILENT:

all : grains

grains : MESS CPINC BASE COMPONENT COLLISION GEOMETRY TIME XML PP
		$(GRAINS_GPU_COMPILER) $(LINKFLAGS) -o $(LIBDIR)/grains \
		$(OBJS_XML) $(OBJS_TIME) $(OBJS_GEOMETRY) $(OBJS_COMPONENT) \
		$(OBJS_COLLISION) $(OBJS_BASE) $(OBJS_PP)
	@echo 'Compilation is complete'

MESS   :
	@echo '***********************************************************'
	@echo 'GRAINS compiling with' $(GRAINS_CPP_COMPILER_DIST)-$(GRAINS_CPP_COMPILER_VERSION)', and' $(GRAINS_GPU_COMPILER_DIST)-$(GRAINS_GPU_COMPILER_VERSION)
	@echo '***********************************************************'

CPINC  : 
	@echo 'Copy include files from subdirectories to' $(INCDIROUT)
	rm -f $(INCDIROUT)/*
	cp $(ALLINC) $(INCDIROUT)/ 	

BASE   : $(OBJS_BASE)
	@echo '*** Package Base compiled'
	@echo

COMPONENT  : $(OBJS_COMPONENT)
	@echo '*** Package Component compiled'
	@echo

COLLISION  : $(OBJS_COLLISION)
	@echo '*** Package Collision compiled'
	@echo
	
GEOMETRY   : $(OBJS_GEOMETRY)
	@echo '*** Package Geometry compiled'
	@echo

TIME   : $(OBJS_TIME)
	@echo '*** Package TimeIntegration compiled'
	@echo

XML    : $(OBJS_XML)
	@echo '*** Package XML compiled'
	@echo

PP     : $(OBJS_PP)
	@echo '*** Package PostProcessing compiled'
	@echo

$(OBJDIR)/%.o : %.cpp
	@echo 'Compiling '$<
	$(GRAINS_GPU_COMPILER) $(COMPFLAGS) -D_XML $< -o $@


# DEPENDENCIES
DEPEND  := $(INCDIR:%=-I%) $(GRAINS_SYSINC:%=-I%)

depend:
	@echo 'Generating dependencies'
	touch ./Maketmp
	makedepend -f ./Maketmp $(DEPEND) $(SRCFILES_WP_BASE) \
	$(SRCFILES_WP_COMPONENTS) $(SRCFILES_WP_COLLISION) $(SRCFILES_WP_GEOMETRY) \
	$(SRCFILES_WP_TIME) $(SRCFILES_WP_XML) $(SRCFILES_WP_PP)
	rm -f ./Maketmp ./Maketmp.bak

# CLEAN
clean : 
	@echo 'Grains : Deleting object and library files'
	rm -f $(INCDIROUT)/*
	rm -f $(OBJDIR)/*.o
	rm -f $(LIBDIR)/*