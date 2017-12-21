# Webots Makefile system 
#
# You may add some variable definitions hereafter to customize the build process
# See documentation in $(WEBOTS_HOME_PATH)/resources/Makefile.include


# Do not modify the following: this includes Webots global Makefile.include
space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))

RESOURCES_PATH="/home/wxr302124/Assignments/Robo_Bar/controllers/pr2_demo"

INCLUDE += -I"$(RESOURCES_PATH)/include"
LIBRARIES += -L"$(RESOURCES_PATH)/libs/x64" -lmsc -lrt -ldl -lpthread -lasound
C_SOURCES = $(wildcard *.c)


include $(WEBOTS_HOME_PATH)/resources/Makefile.include
