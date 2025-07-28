null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))

FRAMEWORK_PATH = ./Framework
MANAGERS_SOURCES_PATH = ./managers/src
MANAGERS_INCLUDE_PATH = ./managers

CXX_SOURCES = \
	enum.cpp \
  main.cpp \
  $(FRAMEWORK_PATH)/src/math/Matrix.cpp \
  $(FRAMEWORK_PATH)/src/math/Vector.cpp \
  $(FRAMEWORK_PATH)/src/math/Point.cpp \
  $(FRAMEWORK_PATH)/src/MX28.cpp \
  $(FRAMEWORK_PATH)/src/CM730.cpp \
  $(FRAMEWORK_PATH)/src/LinuxMotionTimer.cpp \
  $(FRAMEWORK_PATH)/src/motion/JointData.cpp \
  $(FRAMEWORK_PATH)/src/vision/ColorFinder.cpp \
  $(FRAMEWORK_PATH)/src/vision/Camera.cpp \
  $(FRAMEWORK_PATH)/src/vision/Image.cpp \
  $(FRAMEWORK_PATH)/src/vision/ImgProcess.cpp \
  $(FRAMEWORK_PATH)/src/motion/MotionStatus.cpp \
  $(FRAMEWORK_PATH)/src/motion/MotionManager.cpp \
  $(FRAMEWORK_PATH)/src/motion/Kinematics.cpp \
  $(FRAMEWORK_PATH)/src/motion/modules/Action.cpp \
  $(FRAMEWORK_PATH)/src/motion/modules/Walking.cpp \
  $(MANAGERS_SOURCES_PATH)/DirectoryManager.cpp \
  $(MANAGERS_SOURCES_PATH)/MotionPlayManager.cpp \
  $(MANAGERS_SOURCES_PATH)/MotionTimerManager.cpp \
  $(MANAGERS_SOURCES_PATH)/VisionManager.cpp \
  $(MANAGERS_SOURCES_PATH)/GaitManager.cpp 

C_SOURCES = \
  $(FRAMEWORK_PATH)/src/minIni/minIni.c

INCLUDE = -I"$(FRAMEWORK_PATH)/include" -I"$(MANAGERS_INCLUDE_PATH)/include"

### Do not modify: this includes Webots global Makefile.include
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
