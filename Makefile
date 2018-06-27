
FLAGS = -O2 -I mjpro150/include -Lmjpro150/bin -Wall -mavx

MAIN = traj
OBJS = simulate.o trajmain.o

MJ = mjpro150/bin/libglewegl.so \
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

CASSIE = cassie-stl-meshes/achilles-rod.stl \
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

all : $(MAIN) | mjkey.txt	

$(CASSIE) : | mjkey.txt
	-rm -rf cassie-mujoco-sim
	git clone "git@github.com:osudrl/cassie-mujoco-sim.git"
	cp cassie-mujoco-sim/model/cassie.xml .
	cp -rf cassie-mujoco-sim/model/cassie-stl-meshes .
	rm -rf cassie-mujoco-sim

$(MJ) : | mjkey.txt
	mkdir -p temp
	wget -O temp/mjpro150_linux.zip "https://www.roboti.us/download/mjpro150_linux.zip"
	-rm -rf mjpro150 $(MAIN)
	unzip temp/mjpro150_linux.zip
	-rm -rf temp

$(MAIN) :  $(OBJS) trajmain.h | mjkey.txt $(CASSIE) $(MJ) 
	g++ $(FLAGS) $(OBJS) -lmujoco150 -lGL -lglew mjpro150/bin/libglfw.so.3 -o $(MAIN)

mjkey.txt : 
	$(error MuJoCo will need a product key to run the tool. \
		Please provide a product key for MuJoCo and name it mjkey.txt)

simulate.o : trajmain.h simulate.c | mjkey.txt $(MJ)
	gcc -c $(FLAGS) simulate.c

trajmain.o : trajmain.h trajmain.c | mjkey.txt $(MJ)
	gcc -c $(FLAGS) trajmain.c

clean :
	-rm -f *.o $(MAIN)

purge :
	-rm -f *.o $(MAIN)
	-rm -rf cassie-stl-meshes
	-rm -rf cassie-mujoco-sim
	-rm -f cassie.xml
	-rm -rf mjpro150
	-rm -f mjkey.txt
