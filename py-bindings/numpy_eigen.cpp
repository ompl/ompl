#include <Eigen/Eigen>
#include <boost/python/numpy.hpp>

namespace bp = boost::python;
namespace np = boost::python::numpy;

#define EIGEN_ARRAY_CONVERTER(Type, N)                                                                                 \
    EigenFromPython<Type, N>();                                                                                        \
    bp::to_python_converter<Eigen::Ref<Type>, EigenToPython<Type>>();                                                  \
    EigenFromPython<const Type, N>();                                                                                  \
    bp::to_python_converter<Eigen::Ref<const Type>, EigenToPython<const Type>>();

template <typename T>
struct EigenToPython
{
    static PyObject *convert(const Eigen::Ref<T> &m)
    {
        double *data = const_cast<double *>(m.data());
        bp::object capsule(
            bp::handle<>(PyCapsule_New(new Eigen::Map<T>(data, m.rows(), m.cols()), nullptr, [](PyObject *ptr) {
                delete (Eigen::Map<T> *)PyCapsule_GetPointer(ptr, nullptr);
            })));
        return boost::python::incref(
            np::from_data(data, np::dtype::get_builtin<double>(), bp::make_tuple(m.rows(), m.cols()),
                          bp::make_tuple(m.rowStride() * sizeof(double), m.colStride() * sizeof(double)), capsule)
                .ptr());
    }
};
template <>
PyObject *EigenToPython<Eigen::VectorXd>::convert(const Eigen::Ref<Eigen::VectorXd> &v)
{
    double *data = const_cast<double *>(v.data());
    bp::object capsule(
        bp::handle<>(PyCapsule_New(new Eigen::Map<Eigen::VectorXd>(data, v.rows()), nullptr, [](PyObject *ptr) {
            delete (Eigen::Map<Eigen::VectorXd> *)PyCapsule_GetPointer(ptr, nullptr);
        })));
    return boost::python::incref(np::from_data(data, np::dtype::get_builtin<double>(), bp::make_tuple(v.rows()),
                                               bp::make_tuple(v.innerStride() * sizeof(double)), capsule)
                                     .ptr());
}
template <>
PyObject *EigenToPython<const Eigen::VectorXd>::convert(const Eigen::Ref<const Eigen::VectorXd> &v)
{
    double *data = const_cast<double *>(v.data());
    bp::object capsule(
        bp::handle<>(PyCapsule_New(new Eigen::Map<Eigen::VectorXd>(data, v.rows()), nullptr, [](PyObject *ptr) {
            delete (Eigen::Map<Eigen::VectorXd> *)PyCapsule_GetPointer(ptr, nullptr);
        })));
    return boost::python::incref(np::from_data(data, np::dtype::get_builtin<double>(), bp::make_tuple(v.rows()),
                                               bp::make_tuple(v.innerStride() * sizeof(double)), capsule)
                                     .ptr());
}

template <typename T>
void copy_ndarray(const np::ndarray &array, void *storage)
{
    new (storage) Eigen::Ref<T>(Eigen::Map<T, 0, Eigen::OuterStride<>>(reinterpret_cast<double *>(array.get_data()),
                                                                       array.shape(0), array.shape(1),
                                                                       Eigen::OuterStride<>(array.strides(1))));
}
template <>
void copy_ndarray<Eigen::VectorXd>(const np::ndarray &array, void *storage)
{
    new (storage) Eigen::Ref<Eigen::VectorXd>(
        Eigen::Map<Eigen::VectorXd>(reinterpret_cast<double *>(array.get_data()), array.shape(0)));
}
template <>
void copy_ndarray<const Eigen::VectorXd>(const np::ndarray &array, void *storage)
{
    new (storage) Eigen::Ref<const Eigen::VectorXd>(
        Eigen::Map<const Eigen::VectorXd>(reinterpret_cast<double *>(array.get_data()), array.shape(0)));
}

template <typename T, int N>
struct EigenFromPython
{
    EigenFromPython()
    {
        bp::converter::registry::push_back(&convertible, &construct, bp::type_id<Eigen::Ref<T>>());
    }

    static void *convertible(PyObject *p)
    {
        try
        {
            bp::object obj(bp::handle<>(bp::borrowed(p)));
            std::unique_ptr<np::ndarray> array(new np::ndarray(
                np::from_object(obj, np::dtype::get_builtin<double>(), N, N, np::ndarray::C_CONTIGUOUS)));
            return array.release();
        }
        catch (bp::error_already_set &err)
        {
            bp::handle_exception();
            return nullptr;
        }
    }

    static void construct(PyObject *objPtr, bp::converter::rvalue_from_python_stage1_data *data)
    {
        std::unique_ptr<np::ndarray> array(reinterpret_cast<np::ndarray *>(data->convertible));
        void *storage =
            reinterpret_cast<bp::converter::rvalue_from_python_storage<Eigen::Ref<T>> *>(data)->storage.bytes;
        copy_ndarray<T>(*array, storage);
        data->convertible = storage;
    }
};
