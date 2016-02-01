saveIMUdata: saveIMUdata.o vn200.o vncp_services.o vndevice.o
	gcc -pthread -o saveIMUdata saveIMUdata.o vn200.o vndevice.o vncp_services.o -lrt

saveIMUdata.o: saveIMUdata.c
	gcc -c -Wall -I../../include saveIMUdata.c

vn200.o: ../../src/vn200.c
	gcc -c -Wall -I../../include ../../src/vn200.c

vndevice.o: ../../src/vndevice.c
	gcc -c -Wall -I../../include ../../src/vndevice.c

vncp_services.o: ../../src/arch/linux/vncp_services.c
	gcc -c -Wall -I../../include ../../src/arch/linux/vncp_services.c

clean:
	rm -f *.o saveIMUdata