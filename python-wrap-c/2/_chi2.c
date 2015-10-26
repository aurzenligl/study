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
    PyObject* x_obj, *y_obj, *yerr_obj;

    if (!PyArg_ParseTuple(args, "ddOOO", &m, &b, &x_obj, &y_obj, &yerr_obj))
    {
        return NULL;
    }

    PyObject* x_array = PyArray_FROM_OTF(x_obj, NPY_DOUBLE, NPY_IN_ARRAY);
    PyObject* y_array = PyArray_FROM_OTF(y_obj, NPY_DOUBLE, NPY_IN_ARRAY);
    PyObject* yerr_array = PyArray_FROM_OTF(yerr_obj, NPY_DOUBLE, NPY_IN_ARRAY);
    if (x_array == NULL || y_array == NULL || yerr_array == NULL)
    {
        Py_XDECREF(x_array);
        Py_XDECREF(y_array);
        Py_XDECREF(yerr_array);
        return NULL;
    }

    int N = (int)PyArray_DIM(x_array, 0);

    double* x = (double*)PyArray_DATA(x_array);
    double* y = (double*)PyArray_DATA(y_array);
    double* yerr = (double*)PyArray_DATA(yerr_array);

    double value = chi2(m, b, x, y, yerr, N);

    Py_DECREF(x_array);
    Py_DECREF(y_array);
    Py_DECREF(yerr_array);

    if (value < 0.0)
    {
        PyErr_SetString(PyExc_RuntimeError,
            "Chi-squared returned an impossible value.");
        return NULL;
    }

    PyObject* ret = Py_BuildValue("d", value);
    return ret;
}
