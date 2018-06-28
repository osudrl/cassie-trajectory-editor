
FLAGS = -O2 -I mjpro150/include -Lmjpro150/bin -Wall -mavx

MAIN = traj jointtest

MJ = \
mjpro150/bin/libglewegl.so \
mjpro150/bin/libglewosmesa.so \
mjpro150/bin/libglew.so \
mjpro150/bin/libglfw.so.3 \
mjpro150/bin/libmujoco150nogl.so \
mjpro150/bin/libmujoco150.so \
mjpro150/include/glfw3.h \
mjpro150/include/mjdata.h \
mjpro150/include/mjmodel.h \
mjpro150/include/mjrender.h \
mjpro150/include/mjvisualize.h \
mjpro150/include/mjxmacro.h \
mjpro150/include/mujoco.h \

CASSIE = \
cassie-stl-meshes/achilles-rod.stl \
cassie-stl-meshes/foot-crank.stl \
cassie-stl-meshes/foot.stl \
cassie-stl-meshes/heel-spring.stl \
cassie-stl-meshes/hip-pitch.stl \
cassie-stl-meshes/hip-roll.stl \
cassie-stl-meshes/hip-yaw.stl \
cassie-stl-meshes/knee-spring.stl \
cassie-stl-meshes/knee.stl \
cassie-stl-meshes/pelvis.stl \
cassie-stl-meshes/plantar-rod.stl \
cassie-stl-meshes/shin.stl \
cassie-stl-meshes/tarsus.stl \
cassie.xml

all : $(MAIN) | mjkey.txt package

mjkey.txt : 
	$(error MuJoCo will need a product key to run the tool. \
		Please provide a product key for MuJoCo and name it mjkey.txt)

$(CASSIE) : | mjkey.txt package
	-rm -rf cassie-stl-meshes
	mkdir -p cassie-stl-meshes
	wget -O cassie.xml "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie.xml"
	wget -O cassie-stl-meshes/achilles-rod.stl "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie-stl-meshes/achilles-rod.stl"
	wget -O cassie-stl-meshes/foot-crank.stl "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie-stl-meshes/foot-crank.stl"
	wget -O cassie-stl-meshes/foot.stl "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie-stl-meshes/foot.stl"
	wget -O cassie-stl-meshes/heel-spring.stl "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie-stl-meshes/heel-spring.stl"
	wget -O cassie-stl-meshes/hip-pitch.stl "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie-stl-meshes/hip-pitch.stl"
	wget -O cassie-stl-meshes/hip-roll.stl "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie-stl-meshes/hip-roll.stl"
	wget -O cassie-stl-meshes/hip-yaw.stl "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie-stl-meshes/hip-yaw.stl"
	wget -O cassie-stl-meshes/knee-spring.stl "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie-stl-meshes/knee-spring.stl"
	wget -O cassie-stl-meshes/knee.stl "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie-stl-meshes/knee.stl"
	wget -O cassie-stl-meshes/pelvis.stl "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie-stl-meshes/pelvis.stl"
	wget -O cassie-stl-meshes/plantar-rod.stl "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie-stl-meshes/plantar-rod.stl"
	wget -O cassie-stl-meshes/shin.stl "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie-stl-meshes/shin.stl"
	wget -O cassie-stl-meshes/tarsus.stl "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie-stl-meshes/tarsus.stl"

$(MJ) : | mjkey.txt package
	mkdir -p temp
	wget -O temp/mjpro150_linux.zip "https://www.roboti.us/download/mjpro150_linux.zip"
	-rm -rf mjpro150 $(MAIN)
	unzip temp/mjpro150_linux.zip
	-rm -rf temp

traj : simulate.o main-traj.o main.h | mjkey.txt package $(CASSIE) $(MJ) 
	g++ $(FLAGS) \
	simulate.o main-traj.o \
	-lmujoco150 -lGL -lglew \
	mjpro150/bin/libglfw.so.3 \
	-o traj

jointtest : simulate.o main-joint.o main.h | mjkey.txt package $(CASSIE) $(MJ)
	g++ $(FLAGS) \
	simulate.o main-joint.o \
	-lmujoco150 -lGL -lglew \
	mjpro150/bin/libglfw.so.3 \
	-o jointtest

simulate.o : main.h simulate.c | mjkey.txt package $(MJ)
	gcc -c $(FLAGS) simulate.c

main-traj.o : main.h main-traj.c | mjkey.txt package $(MJ)
	gcc -c $(FLAGS) -std=gnu99 -pedantic -Wdeclaration-after-statement main-traj.c

main-joint.o : main.h main-joint.c | mjkey.txt package $(MJ)
	gcc -c $(FLAGS) -std=gnu99 -pedantic -Wdeclaration-after-statement main-joint.c	

clean :
	-rm -f *.o $(MAIN)

lpurge :
	-rm -f *.o $(MAIN)
	-rm -rf cassie-stl-meshes
	-rm -rf cassie-mujoco-sim
	-rm -f cassie.xml
	-rm -rf mjpro150

purge :
	-rm -f *.o $(MAIN)
	-rm -rf cassie-stl-meshes
	-rm -rf cassie-mujoco-sim
	-rm -f cassie.xml
	-rm -rf mjpro150
	-rm -f mjkey.txt

.PHONY: package

package:
	@dpkg -l | grep libglfw3-dev > /dev/null
	@echo $(LD_LIBRARY_PATH) | grep "mjpro150/bin" > /dev/null
	
