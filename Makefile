# ----------------
# Standard targets
# ----------------
update: githook
	cd Main/src; \
	make; \
	cd ../..;
	@echo 'Grains is updated!'

install: xerces createarch createdepend depend update dtd
	@echo 'Full Grains platform built!'

updatedev: clean cleandepend createdepend update 

depend:
	cd Grains; \
	make depend; \
	cd ..;
	cd Main/src; \
	make depend; \
	cd ../..;
	
clean:
	cd Main/src; \
	make clean; \
	cd ../..;
	@echo 'Grains platform cleaned'
	
cleanall: cleanxerces clean cleandepend deletearch cleandtd
	@echo 'Full Grains platform cleaned!'		


# -----------------
# Low level targets
# -----------------
apply-clang-format:
	@echo "Formatting all source files according to .clang-format ..."
	find ./Grains/ -name "*.cpp" -o -name "*.hh" | \
	xargs clang-format -i --style=file:./.clang-format --verbose; \
	echo "Formatting complete.";

# Rule to install the pre-commit hook
install-githook:
	@echo "Installing pre-commit hook..."
	cp .githooks/pre-commit .git/hooks/pre-commit && \
	chmod +x .git/hooks/pre-commit && \
	echo "Pre-commit hook installed successfully."; \

githook:
	@echo '----------------------'
	@echo "Running githooks..."
	@echo '----------------------'
	bash .git/hooks/pre-commit
	@echo '----------------------'
	@echo "Running githooks finished."
	@echo '----------------------'

xerces:
	cd $(XERCES_DIR); \
	$(INSTALL_XERCES); \
	cd ..;

dtd:
	cd Main/dtd; \
	$(INSTALL_DTD); \
	cd ../..;

createarch:
	./makeARCH create
	
createdepend:
	cd Grains; \
	$(TOUCH) Makedepend; \
	cd ..;
	cd Main/src; \
	$(TOUCH) Makedepend; \
	cd ../..;
	

# --------------------------
# Low level cleaning targets
# --------------------------
cleanxerces:
	cd $(XERCES_SOURCE); \
	make clean; \
	cd ../../..;
	cd $(XERCES_DIR); \
	$(RM) ${GRAINS_XERCES_LIBDIR}; \
	cd ..;
	@echo 'XERCES cleaned'

cleandtd:
	cd Main/dtd; \
	$(RM) Grains*.dtd; \
	cd ../..
	@echo 'dtd cleaned!'

deletearch:
	./makeARCH delete

cleandepend:
	cd Grains; \
	$(RM) Makedepend; \
	cd ..
	cd Main/src; \
	$(RM) Makedepend; \
	cd ../..
	

# ----	
# Help
# ----		
help:
	@echo 'Below are the various targets:'
	@echo '   STANDARD TARGETS:'
	@echo '      update (default) $(BANG) compile Grains3D source files, create library, main exe file and pre/post exe files'
	@echo '      install          $(BANG) perform the following sequence of targets: xerces createarch createdepend depend update dtd'
	@echo '      depend           $(BANG) generate all dependency files'	
	@echo '      clean            $(BANG) delete all Grains library and exe files'		
	@echo '      cleanall         $(BANG) perform the following sequence of targets: cleanxerces clean cleandepend deletearch cleandtd'			
	@echo
	@echo '   LOW-LEVEL TARGETS:'
	@echo '      xerces           $(BANG) compile the XERCES library'
	@echo '      dtd              $(BANG) install the DTD files'
	@echo '      createarch       $(BANG) create all directories for obj, lib and exe files using the arch name defined in the env file '	
	@echo '      createdepend     $(BANG) create all Makedepend files (empty if they do not yet exist)'
	@echo		
	@echo '   LOW-LEVEL CLEANING TARGETS:'
	@echo '      cleanxerces      $(BANG) delete all XERCES lib and obj files/directories (undoes target xerces)'
	@echo '      cleandtd         $(BANG) delete the path specific DTD files (undoes target dtd)'
	@echo '      deletearch       $(BANG) delete all architecture specific obj, lib and exe files/directories (undoes target createarch)'	
	@echo '      cleandepend      $(BANG) delete all Makedepend files (undoes target createdepend)'	
	@echo
	@echo '   DEVELOPER TARGETS:'	
	@echo '      dev              $(BANG) compile Grains3D source files and create library'
	@echo '      updatedev        $(BANG) perform the following sequence of targets: clean cleandepend createdepend update'	

	
##################################################################
# internal commands                                              #
##################################################################
TOUCH := touch
RM := rm -rf
INSTALL_XERCES := ./install.sh
XERCES_DIR := XERCES-2.8.0
XERCES_SOURCE := XERCES-2.8.0/src/xercesc
INSTALL_DTD := ./installdtd.sh
BANG := \#
