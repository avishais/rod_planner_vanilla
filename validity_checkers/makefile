#compiler
OMPL_DIR = /usr
WRK_SPC = /home/avishai/Documents/workspace
INC_CLASSES = ${WRK_SPC}/rod_planner_vanilla/proj_classes/
INC_PLANNERS = ${WRK_SPC}/rod_planner_vanilla/planners/
INC_VALIDITY = ${WRK_SPC}/rod_planner_vanilla/validity_checkers/
INC_RUN = ${WRK_SPC}/rod_planner_vanilla/run/

GL_INCPATH = -I/usr/include/
GL_LIBPATH = -L/usr/lib/ -L/usr/X11R6/lib/
GL_LIBS = -lGLU -lGL -lXext -lX11 -lglut

PQP_DIR= /home/avishai/Documents/PQP_v1.3/

EIGEN_DIR = /home/avishai/Documents/eigen
KDL_DIR = /usr/local

CXX= g++
CXXFLAGS= -I${OMPL_DIR}/local/include -I${OMPL_DIR}/lib/x86_64-linux-gnu -I${INC_CLASSES} -I${INC_PLANNERS} -I${PQP_DIR}/include $(GL_INCPATH) -I${KDL_DIR}/include  -I$(EIGEN_DIR) 
LDFLAGS= -L${OMPL_DIR}/local/lib -L${OMPL_DIR}/lib -L${OMPL_DIR}/lib/x86_64-linux-gnu -llapack -lompl -lboost_filesystem -lboost_system -lboost_serialization -lboost_program_options -Wl,-rpath ${OM zPL_DIR}/lib/x86_64-linux-gnu -L${PQP_DIR}/lib -L${KDL_DIR}/lib -lPQP -lm $(GL_LIBS) -lblas  -larmadillo -lorocos-kdl 
LIBS += -L/usr/lib/x86_64-linux-gnu -lboost_system

CPPPQP = ${INC_VALIDITY}collisionDetection.cpp ${INC_VALIDITY}model.cpp

CPPROD = ${INC_CLASSES}Rod_ODE_class.cpp
CPPROB = ${INC_CLASSES}abb_apc_class.cpp ${INC_CLASSES}kdl_class.cpp ${INC_VALIDITY}StateValidityChecker.cpp

CPPPLN = gen_rand.cpp


all:
	$(CXX) ${CPPPLN} ${CPPROD} ${CPPPQP} ${CPPROB} -o g $(CXXFLAGS) $(LDFLAGS) -std=c++11
