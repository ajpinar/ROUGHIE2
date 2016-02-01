
all: gimmepitch.o gimmeyaw.o gimmeroll.o vn200.o vncp_services.o vndevice.o
	gcc -pthread -o bin/gimmepitch gimmepitch.o vn200.o vndevice.o vncp_services.o -lrt
	gcc -pthread -o bin/gimmeyaw gimmeyaw.o vn200.o vndevice.o vncp_services.o -lrt
	gcc -pthread -o bin/gimmeroll gimmeroll.o vn200.o vndevice.o vncp_services.o -lrt

gimmepitch.o: src/gimmeangle.c
	gcc -c -Wall -Iinclude -DEULERANGLE=pitch src/gimmeangle.c -o gimmepitch.o

gimmeyaw.o: src/gimmeangle.c
	gcc -c -Wall -Iinclude -DEULERANGLE=yaw src/gimmeangle.c -o gimmeyaw.o

gimmeroll.o: src/gimmeangle.c
	gcc -c -Wall -Iinclude -DEULERANGLE=roll src/gimmeangle.c -o gimmeroll.o

vn200.o: src/vn200.c
	gcc -c -Wall -Iinclude src/vn200.c

vndevice.o: src/vndevice.c
	gcc -c -Wall -Iinclude src/vndevice.c

vncp_services.o: src/arch/linux/vncp_services.c
	gcc -c -Wall -Iinclude src/arch/linux/vncp_services.c

clean:
	rm -f *.o gimmepitch