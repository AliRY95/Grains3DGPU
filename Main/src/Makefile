# Target directories
EXEDIR := ../bin$(GRAINS_FULL_EXT)
OBJDIR := ../obj$(GRAINS_FULL_EXT)


# Source and object files
MAINSRC := main.cpp
MAINOBJ := $(MAINSRC:%.cpp=$(OBJDIR)/%.o)


# Include and library from Grains
GRAINSINC   := $(GRAINS_ROOT)/include
GRAINSLIB   := $(GRAINS_ROOT)/lib$(GRAINS_FULL_EXT)
GRAINSFLAGS := -L$(GRAINSLIB) -lgrains


# Compiler
INCFLAGS  := $(INCDIR:%=-I%) $(GRAINSINC:%=-I%) $(GRAINS_XERCES_INCDIR:%=-I%)
COMPFLAGS := $(GRAINS_GPU_COMPILER_FLAGS) $(INCFLAGS)
LINKFLAGS := $(GRAINS_GPU_LINKER_FLAGS) $(GRAINS_Z_FLAGS) $(GRAINS_XERCES_FLAGS) $(GRAINSFLAGS)


# -----------------------------------------------------------------------------
# Production rules
.SILENT:

all : grains

# EXECUTABLE
grains : MESS $(EXEDIR)/grains
	@echo '----------------------'
	@echo 'Software grains built!'
	@echo '----------------------'

$(EXEDIR)/grains : app
	@echo 'Building software Grains'
	$(GRAINS_GPU_COMPILER) $(MAINOBJ) -o $@ $(LINKFLAGS)

MESS   :
	@echo '***********************************************************'
	@echo 'Binary compiling with' $(GRAINS_CPP_COMPILER_DIST)-$(GRAINS_CPP_COMPILER_VERSION)', and' $(GRAINS_GPU_COMPILER_DIST)-$(GRAINS_GPU_COMPILER_VERSION)
	@echo '***********************************************************'

app    :  Grains $(MAINOBJ)

Grains :
	( cd ../../Grains; make; cd ../Main/src )

$(OBJDIR)/%.o : %.cpp
	@echo 'Compiling '$<
	$(GRAINS_GPU_COMPILER) $(COMPFLAGS) -c $< -o $@


# DEPENDENCIES
UTLDIR  := $(GRAINS_HOME)/Tools/Perl
DEPEND  := -I$(GRAINSINC) $(GRAINS_SYSINC:%=-I%)

depend :
	@echo 'Generating dependencies'
	touch ./Maketmp
	makedepend -f ./Maketmp $(DEPEND) $(MAINSRC)
	$(UTLDIR)/makeadd.pl $(OBJDIR)/ ./Maketmp ./Makedepend
	rm -f ./Maketmp ./Maketmp.bak

include ./Makedepend      # DO NOT DELETE

# CLEAN
clean :
	( cd ../../Grains; make clean; cd ../Main/src )
	@echo 'Main   : deleting binary and object files ...'
	rm -f $(MAINOBJ)
	rm -f $(EXEDIR)/*
	@echo 'Main   : Deleted.'
