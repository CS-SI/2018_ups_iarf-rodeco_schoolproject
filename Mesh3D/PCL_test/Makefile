MAKEFLAGS =--no-print-directory
SHELL := /bin/bash
CMAKE_BUILD_DIRECTORY := project-build

.PHONY: compile build clearall clean clean-build

all: compile

compile: |build
	cd $(CMAKE_BUILD_DIRECTORY) && $(MAKE) $(MAKEFLAGS)

$(CMAKE_BUILD_DIRECTORY):
	mkdir $(CMAKE_BUILD_DIRECTORY)

build: |$(CMAKE_BUILD_DIRECTORY)
	cd ./$(CMAKE_BUILD_DIRECTORY) &&	cmake ../sources

clearall: clean clean-build

clean-exec:
	cd $(CMAKE_BUILD_DIRECTORY) && $(MAKE) clean $(MAKEFLAGS)

clean-pyc:
	cd meshpylib && rm -f *.pyc

clean: clean-exec clean-pyc
	rm -rf reconstructed

clean-build:
	rm -rf $(CMAKE_BUILD_DIRECTORY)
