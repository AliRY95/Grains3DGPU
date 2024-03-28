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


# Target directories
OBJDIR := obj


# Substitutions for objects
# Here we use the base file names combined to VPATH
OBJS_BASE := $(SRCFILESBASE:%.cpp=$(OBJDIR)/%.o)
OBJS_COMPONENT := $(SRCFILESCOMPONENT:%.cpp=$(OBJDIR)/%.o)
OBJS_COLLISION := $(SRCFILESCOLLISION:%.cpp=$(OBJDIR)/%.o)
OBJS_GEOMETRY := $(SRCFILESGEOMETRY:%.cpp=$(OBJDIR)/%.o)

# Defining VPATH for automatic search of source files in the 
# different source directories 
# This enables to have a single rule/target for all source files in 
# various directories and all objects in a single directory
VPATH := $(SRCDIRBASE):$(SRCDIRCOMPONENT):$(SRCDIRCOLLISION):$(SRCDIRGEOMETRY)


# Compiler
COMP  := nvcc
LINK  := nvcc
INCFLAGS  := $(INCDIR:%=-I%)
# DEBUG := -g -G -diag-suppress 554
# COMPFLAGS := $(DEBUG) -x cu -arch=sm_75 -dc $(INCFLAGS)
# LINKFLAGS := -Xnvlink -dump-callgraph-no-demangle -arch=sm_75 -rdc=true $(INCFLAGS)
DEBUG  := -g -Xcompiler -rdynamic -diag-suppress 554
COMPFLAGS := $(DEBUG) -O3 -arch=sm_75 -dc -dlto -x cu $(INCFLAGS)
LINKFLAGS := -O3 -arch=sm_75 -dlto $(INCFLAGS)


#------------------------------------------------------------------------------
# Production rules
.SILENT:

all : grains

grains : MESS CPINC BASE COMPONENT COLLISION GEOMETRY
	$(LINK) $(LINKFLAGS) $(OBJS_GEOM) $(OBJS_COMPO) $(OBJS_BASE) -o grains
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

COMPONENT  : $(OBJS_COMPONENTS)
	@echo '*** Package Component compiled'
	@echo

COLLISION  : $(OBJS_COLLISION)
	@echo '*** Package Component compiled'
	@echo
	
GEOMETRY   : $(OBJS_GEOMETRY)
	@echo '*** Package Geometry compiled'
	@echo

$(OBJDIR)/%.o : %.cpp
	@echo 'Compiling '$<
	$(COMP) $(COMPFLAGS) $< -o $@


# DEPENDENCIES
DEPEND  := $(INCDIR:%=-I%) $(GRAINS_SYSINC:%=-I%)

depend:
	@echo 'Generating dependencies'
	touch ./Maketmp
	makedepend -f ./Maketmp $(DEPEND) $(SRCFILES_WP_BASE) \
	$(SRCFILES_WP_COMPONENTS) $(SRCFILES_WP_COLLISION) $(SRCFILES_WP_GEOMETRY) \
	rm -f ./Maketmp ./Maketmp.bak

# CLEAN
clean : 
	@echo 'Grains : Deleting object and library files'
	rm -f $(INCDIROUT)/*
	rm -f $(OBJDIR)/*.o
