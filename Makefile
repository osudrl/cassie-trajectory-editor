
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
model/cassie-stl-meshes/achilles-rod.stl \
model/cassie-stl-meshes/foot-crank.stl \
model/cassie-stl-meshes/foot.stl \
model/cassie-stl-meshes/heel-spring.stl \
model/cassie-stl-meshes/hip-pitch.stl \
model/cassie-stl-meshes/hip-roll.stl \
model/cassie-stl-meshes/hip-yaw.stl \
model/cassie-stl-meshes/knee-spring.stl \
model/cassie-stl-meshes/knee.stl \
model/cassie-stl-meshes/pelvis.stl \
model/cassie-stl-meshes/plantar-rod.stl \
model/cassie-stl-meshes/shin.stl \
model/cassie-stl-meshes/tarsus.stl \
model/cassie.xml

LCOMMON = -lmujoco150 -lGL -lglew \
-Wl,-rpath,"\$$ORIGIN/mjpro150/bin" \
mjpro150/bin/libglfw.so.3 

CCOMMON = -std=gnu99 -pedantic -Wdeclaration-after-statement

all : $(MAIN) | mjkey.txt

mjkey.txt : 
	$(error MuJoCo will need a product key to run the tool. \
		Please provide a product key for MuJoCo and name it mjkey.txt)

$(CASSIE) : | mjkey.txt
	-rm -rf model
	mkdir -p model
	mkdir -p model/cassie-stl-meshes
	wget -O model/cassie.xml "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie.xml" 
	wget -O model/cassie-stl-meshes/achilles-rod.stl "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie-stl-meshes/achilles-rod.stl" 
	wget -O model/cassie-stl-meshes/foot-crank.stl "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie-stl-meshes/foot-crank.stl" 
	wget -O model/cassie-stl-meshes/foot.stl "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie-stl-meshes/foot.stl" 
	wget -O model/cassie-stl-meshes/heel-spring.stl "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie-stl-meshes/heel-spring.stl" 
	wget -O model/cassie-stl-meshes/hip-pitch.stl "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie-stl-meshes/hip-pitch.stl" 
	wget -O model/cassie-stl-meshes/hip-roll.stl "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie-stl-meshes/hip-roll.stl" 
	wget -O model/cassie-stl-meshes/hip-yaw.stl "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie-stl-meshes/hip-yaw.stl" 
	wget -O model/cassie-stl-meshes/knee-spring.stl "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie-stl-meshes/knee-spring.stl" 
	wget -O model/cassie-stl-meshes/knee.stl "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie-stl-meshes/knee.stl" 
	wget -O model/cassie-stl-meshes/pelvis.stl "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie-stl-meshes/pelvis.stl" 
	wget -O model/cassie-stl-meshes/plantar-rod.stl "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie-stl-meshes/plantar-rod.stl" 
	wget -O model/cassie-stl-meshes/shin.stl "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie-stl-meshes/shin.stl" 
	wget -O model/cassie-stl-meshes/tarsus.stl "https://raw.githubusercontent.com/osudrl/cassie-mujoco-sim/master/model/cassie-stl-meshes/tarsus.stl" 

$(MJ) : | mjkey.txt
	mkdir -p temp
	wget -O temp/mjpro150_linux.zip "https://www.roboti.us/download/mjpro150_linux.zip"
	-rm -rf mjpro150 $(MAIN)
	unzip temp/mjpro150_linux.zip
	-rm -rf temp

traj : bin/simulate.o bin/main-traj.o src/main.h | mjkey.txt $(CASSIE) $(MJ) 
	g++ \
		$(FLAGS) \
	    bin/simulate.o bin/main-traj.o \
	    $(LCOMMON) \
	    -o traj

jointtest : bin/simulate.o bin/main-joint.o src/main.h | mjkey.txt $(CASSIE) $(MJ)
	g++ \
		$(FLAGS) \
		bin/simulate.o bin/main-joint.o \
		$(LCOMMON) \
		-o jointtest

bin/simulate.o : src/main.h src/simulate.c | mjkey.txt $(MJ) $(CASSIE)
	-@mkdir -p bin
	gcc -c \
		$(FLAGS) \
		src/simulate.c \
		-o bin/simulate.o

bin/main-traj.o : src/main.h src/main-traj.c | mjkey.txt $(MJ) $(CASSIE)
	-@mkdir -p bin
	gcc -c \
		$(FLAGS) \
		$(CCOMMON) \
		src/main-traj.c \
		-o bin/main-traj.o

bin/main-joint.o : src/main.h src/main-joint.c | mjkey.txt $(MJ) $(CASSIE)
	-@mkdir -p bin
	gcc -c \
		$(FLAGS) \
		$(CCOMMON) \
		src/main-joint.c \
		-o bin/main-joint.o

clean :
	-rm -rf bin 
	-rm -rf $(MAIN)

lpurge :
	-rm -rf bin 
	-rm -rf $(MAIN)
	-rm -rf model
	-rm -rf mjpro150

purge :
	-rm -rf bin 
	-rm -rf $(MAIN)
	-rm -rf model
	-rm -rf mjpro150
	-rm -f mjkey.txt	
