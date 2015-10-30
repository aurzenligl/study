#ifndef NPYWRAP_HPP_
#define NPYWRAP_HPP_

template <typename T>
struct npyarray_traits;

template <>
struct npyarray_traits<double>
{
    enum { npy_type = NPY_DOUBLE };
};

template <typename T>
struct npyarray
{
    npyarray(PyObject* obj)
    {
        array_ = PyArray_FROM_OTF(obj, npyarray_traits<T>::npy_type, NPY_IN_ARRAY);
    }
    npyarray(const npyarray&) = delete;
    npyarray& operator=(const npyarray&) = delete;
    ~npyarray()
    {
        Py_XDECREF(array_);
    }

    double* data()
    {
        return static_cast<T*>(PyArray_DATA(array_));
    }

    size_t size() const
    {
        return static_cast<int>(PyArray_DIM(array_, 0));
    }

    explicit operator bool() const
    {
        return array_;
    }

private:
    PyObject* array_;
};

#endif /* NPYWRAP_HPP_ */
