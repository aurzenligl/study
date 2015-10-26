#include <python2.7/Python.h>
#include "hello.h"

static PyObject* hello_fn(PyObject* self, PyObject* args);
static PyObject* waste_time_fn(PyObject* self, PyObject* args);

static PyMethodDef HelloMethods[] = {
    { "hello", hello_fn, METH_VARARGS, "Say hello" },
    { "waste_time", waste_time_fn, METH_VARARGS, "Waste time" },
    { NULL, NULL, 0, NULL }
};

DL_EXPORT(void) inithello(void)
{
    Py_InitModule("hello", HelloMethods);
}

static PyObject* hello_fn(PyObject* self, PyObject* args)
{
    char* input;
    char* result;
    PyObject* ret;

    // parse arguments
    if (!PyArg_ParseTuple(args, "s", &input)) {
        return NULL;
    }

    // run the actual function
    result = hello(input);

    // build the resulting string into a Python object.
    ret = PyString_FromString(result);
    //free(result);

    return ret;
}

static PyObject* waste_time_fn(PyObject* self, PyObject* args) {
    if (!PyArg_ParseTuple(args, "")) {
        return NULL;
    }

    Py_BEGIN_ALLOW_THREADS
    waste_time();
    Py_END_ALLOW_THREADS

    Py_INCREF(Py_None);
    return Py_None;
}
