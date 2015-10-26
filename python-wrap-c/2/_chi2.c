#include <Python.h>
#include <numpy/arrayobject.h>
#include "chi2.h"

static char module_docstring[] =
    "This module provides an interface for calculating chi-squared using C.";
static char chi2_docstring[] =
    "Calculate the chi-squared of some data given a model";

static PyObject* chi2_chi2(PyObject* self, PyObject* args);

static PyMethodDef module_methods[] = {
    { "chi2", chi2_chi2, METH_VARARGS, chi2_docstring },
    { NULL, NULL, 0, NULL }
};

PyMODINIT_FUNC init_chi2(void)
{
    PyObject* m = Py_InitModule3("_chi2", module_methods, module_docstring);
    if (m == NULL)
    {
        return;
    }

    /* Load 'numpy' functionality */
    import_array();
}

static PyObject* chi2_chi2(PyObject* self, PyObject* args)
{
    double m, b;
    PyObject* x, *y, *yerr;

    if (!PyArg_ParseTuple(args, "ddOOO", &m, &b, &x, &y, &yerr))
    {
        return NULL;
    }

    x = PyArray_FROM_OTF(x, NPY_DOUBLE, NPY_IN_ARRAY);
    y = PyArray_FROM_OTF(y, NPY_DOUBLE, NPY_IN_ARRAY);
    yerr = PyArray_FROM_OTF(yerr, NPY_DOUBLE, NPY_IN_ARRAY);
    if (x == NULL || y == NULL || yerr == NULL)
    {
        Py_XDECREF(x);
        Py_XDECREF(y);
        Py_XDECREF(yerr);
        return NULL;
    }

    double value = chi2(m, b,
        (double*)PyArray_DATA(x),
        (double*)PyArray_DATA(y),
        (double*)PyArray_DATA(yerr),
        (int)PyArray_DIM(x, 0));

    Py_DECREF(x);
    Py_DECREF(y);
    Py_DECREF(yerr);

    if (value < 0.0)
    {
        PyErr_SetString(PyExc_RuntimeError,
            "Chi-squared returned an impossible value.");
        return NULL;
    }

    PyObject* ret = Py_BuildValue("d", value);
    return ret;
}
