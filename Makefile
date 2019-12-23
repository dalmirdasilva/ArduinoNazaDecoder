ARDUINO_LIB_PATH=~/Arduino/libraries
THIS_PATH=`pwd`
SOURCE_PATH=`$(THIS_PATH)/libs/arduino/`

all: 
	@echo "Use [install], [unistall] or [doc]"

install:
	@echo "Instaling librarie..."
	ln -s  $(SOURCE_PATH) $(ARDUINO_LIB_PATH)/NazaDecoder
	@echo "done."
	
uninstall:
	@echo "Uninstaling all libraries..."
	rm -r $(ARDUINO_LIB_PATH)/NazaDecoder
	@echo "done."
		
doc:
	@echo "Running doxygen..."
	@rm -rf doc
	@mkdir doc
	doxygen doxygen.conf
	@cd doc/latex; make pdf; cp refman.pdf ../../Documentation.pdf 
	@cd ../..
	@rm -rf doc
	@echo "done."
	
