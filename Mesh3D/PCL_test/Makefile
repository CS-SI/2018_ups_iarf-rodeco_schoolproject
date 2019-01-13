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

clean:
	rm -rf reconstructed  && cd $(CMAKE_BUILD_DIRECTORY) && $(MAKE) clean $(MAKEFLAGS)

clean-build:
	rm -rf $(CMAKE_BUILD_DIRECTORY)
