
FLAGS = -O2 -I mjpro150/include -Lmjpro150/bin -Wall -mavx -g # -Wconversion -Wextra -Wpedantic

MAIN = simik traj

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

OBJS =\
	bin/simulate.o \
	bin/out.o \
	bin/timeline.o \
	bin/vectors.o \
	bin/ik.o \
	bin/node.o

HEADS =  \
	src/main.h \
	src/out.h \
	src/timeline.h \
	src/vectors.h \
	src/node.h \
	src/ik.h

all : $(MAIN) | mjkey.txt

mjkey.txt : 
	$(error MuJoCo will need a product key to run the tool. \
		Please provide a product key for MuJoCo and name it mjkey.txt)

$(CASSIE) : | mjkey.txt
	mkdir -p model/cassie-stl-meshes
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

traj : bin/main-traj.o $(OBJS) $(HEADS) | mjkey.txt $(CASSIE) $(MJ) 
	g++ \
		$(FLAGS) \
	    bin/main-traj.o $(OBJS) \
	    $(LCOMMON) \
	    -o traj

simik : bin/phys.o bin/pdik.o | mjkey.txt $(CASSIE) $(MJ) 
	g++ \
		$(FLAGS) \
	    bin/phys.o bin/pdik.o\
	    $(LCOMMON) \
	    -o simik

bin/phys.o : src/phys.c | mjkey.txt $(MJ) $(CASSIE)
	-@mkdir -p bin
	gcc -c \
		$(FLAGS) \
		src/phys.c \
		-o bin/phys.o

bin/pdik.o : src/pdik.h src/main.h src/pdik.c | mjkey.txt $(MJ) $(CASSIE)
	-@mkdir -p bin
	gcc -c \
		$(FLAGS) \
		$(CCOMMON) \
		src/pdik.c \
		-o bin/pdik.o


bin/simulate.o : $(HEADS) src/simulate.c | mjkey.txt $(MJ) $(CASSIE)
	-@mkdir -p bin
	gcc -c \
		$(FLAGS) \
		src/simulate.c \
		-o bin/simulate.o

bin/main-traj.o : $(HEADS) src/main-traj.c | mjkey.txt $(MJ) $(CASSIE)
	-@mkdir -p bin
	gcc -c \
		$(FLAGS) \
		$(CCOMMON) \
		src/main-traj.c \
		-o bin/main-traj.o

bin/out.o : $(HEADS) src/out.c | mjkey.txt $(MJ) $(CASSIE)
	-@mkdir -p bin
	gcc -c \
		$(FLAGS) \
		$(CCOMMON) \
		src/out.c \
		-o bin/out.o

bin/timeline.o : $(HEADS) src/timeline.c | mjkey.txt $(MJ) $(CASSIE)
	-@mkdir -p bin
	gcc -c \
		$(FLAGS) \
		$(CCOMMON) \
		src/timeline.c \
		-o bin/timeline.o

bin/vectors.o : $(HEADS) src/vectors.c | mjkey.txt $(MJ) $(CASSIE)
	-@mkdir -p bin
	gcc -c \
		$(FLAGS) \
		$(CCOMMON) \
		src/vectors.c \
		-o bin/vectors.o

bin/ik.o : $(HEADS) src/ik.c | mjkey.txt $(MJ) $(CASSIE)
	-@mkdir -p bin
	gcc -c \
		$(FLAGS) \
		$(CCOMMON) \
		src/ik.c \
		-o bin/ik.o

bin/node.o : $(HEADS) src/node.c | mjkey.txt $(MJ) $(CASSIE)
	-@mkdir -p bin
	gcc -c \
		$(FLAGS) \
		$(CCOMMON) \
		src/node.c \
		-o bin/node.o

clean :
	-rm -rf bin 
	-rm -rf $(MAIN)

lpurge :
	-rm -rf bin 
	-rm -rf $(MAIN)
	-rm -rf mjpro150
	-rm -rf model/cassie-stl-meshes

purge :
	-rm -rf bin 
	-rm -rf $(MAIN)
	-rm -rf mjpro150
	-rm -f mjkey.txt
	-rm -rf model/cassie-stl-meshes
