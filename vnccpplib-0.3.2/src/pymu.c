#include <Python.h>

#include <stdio.h>
#include <unistd.h>
#include "vectornav.h"

/* Change the connection settings to your configuration. */
const char* const COM_PORT = "//dev//ttyUSB0"; // tonys
const int BAUD_RATE = 115200;
Vn200 vn200;

void
startup(PyObject* self, PyObject* args)
{
	VN_ERROR_CODE errorCode;

	errorCode = vn200_connect(
        &vn200,
        COM_PORT,
        BAUD_RATE);
	
	/* Make sure the user has permission to use the COM port. */
	if (errorCode == VNERR_PERMISSION_DENIED) {

		printf("Current user does not have permission to open the COM port.\n");
		printf("Try running again using 'sudo'.\n");

		Py_RETURN_NONE;
	}
	else if (errorCode != VNERR_NO_ERROR)
	{
		printf("Error encountered when trying to connect to the sensor.\n");

		Py_RETURN_NONE;
	}
	
	/* Configure the VN-200 to output asynchronous data. */
	errorCode = vn200_setAsynchronousDataOutputType(
        &vn200,
        VNASYNC_VNINS,
        true);

	/* Pause to ensure we have received the first asynchronous data record
	   from the sensor. */
	sleep(1);
	Py_RETURN_NONE;
}

void
shutdown(PyObject* self, PyObject* args)
{
	VN_ERROR_CODE errorCode;
	errorCode = vn200_disconnect(&vn200);
	
	if (errorCode != VNERR_NO_ERROR)
	{
		printf("Error encountered when trying to disconnect from the sensor.\n");
		
		Py_RETURN_NONE;
	}

	Py_RETURN_NONE;
}

double
get_pitch(PyObject* self, PyObject* args)
{
	VnDeviceCompositeData data;
	vn200_getCurrentAsyncData(&vn200, &data);
	Py_BuildValue("d", data.ypr.pitch);
}

double
get_yaw(PyObject* self, PyObject* args)
{
	VnDeviceCompositeData data;
	vn200_getCurrentAsyncData(&vn200, &data);
	Py_BuildValue("d", data.ypr.yaw);
}

double
get_roll(PyObject* self, PyObject* args)
{
	VnDeviceCompositeData data;
	vn200_getCurrentAsyncData(&vn200, &data);
	Py_BuildValue("d", data.ypr.roll);
}

static PyMethodDef TonyMethods[] =
{
     {"startup", startup, METH_VARARGS, "Greet somebody."},
     {"shutdown", shutdown, METH_VARARGS, "Greet somebody."},
     {"get_pitch", get_pitch, METH_VARARGS, "Greet somebody."},
     {"get_yaw", get_yaw, METH_VARARGS, "Greet somebody."},
     {"get_roll", get_roll, METH_VARARGS, "Greet somebody."},
     {NULL, NULL, 0, NULL}
};

PyMODINIT_FUNC
initpymu(void)
{
     (void) Py_InitModule("pymu", TonyMethods);
}