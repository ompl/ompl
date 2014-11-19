# Boost Structures Used in OMPL

[Boost](http://www.boost.org) provides an extension to the C++ standard template library, supplying the user with many generic, cross-platform concepts that are easily included in the developer's program. For users that are unfamiliar with Boost, this page will briefly describe the constructs from the Boost library used in OMPL. This tutorial makes no attempt to fully educate the user on the usage and API of the Boost library, only to give a high level overview of these components and why they were chosen for OMPL. For further detail on Boost, please consult the extensive [Boost documentation](http://www.boost.org/doc/libs/) directly.

\attention
OMPL requires Boost __version 1.48__ or greater.


# Shared Pointer (boost/shared_ptr.hpp)

The [shared pointer from Boost](http://www.boost.org/libs/smart_ptr/shared_ptr.htm) provides intelligent memory
management for dynamically allocated objects created with the _new_ command. In short, the templated shared_ptr object will
delete (free) the allocated memory when the pointer goes out of scope. Since these pointers utilize [reference counting](http://en.wikipedia.org/wiki/Reference_counting), it is safe to pass them into functions or copy them;
the object referenced by the pointer will only be destroyed when the reference count goes to zero.

~~~{.cpp}
class MyClass
{
   /* ... */
   void myClassMethod (void) { /* ... */ }
   /* ... */
};

void performComputation ()
{
    // Create an instance of MyClass using a shared_ptr
    boost::shared_ptr <MyClass> myClassPtr (new MyClass ());

    // Invoke methods on the object as normal
    myClassPtr->myClassMethod ();

} // myClassPtr goes out of scope here.  The instance of MyClass will be deallocated at this point.
~~~

In the example above, the function performComputation creates a new instance of MyClass but instantiates it using a shared_ptr. Once the object is created, the pointer can be used like normal (using -> or the * operators). When the function has finished execution, the object does not have to be specifically deleted; the reference count on myClassPtr will decrement to zero when the shared_ptr goes out of scope, triggering the automatic destruction the MyClass object.

The shared_ptr is used in OMPL in nearly all instances where an object is created from the heap in order to mitigate memory leaks. Most classes in OMPL already have a typedef for a shared_ptr object using the _OMPL_CLASS_FORWARD_ macro. For example, ompl::base::Planner utilizes the macro just before the class declaration:

~~~{.cpp}
OMPL_CLASS_FORWARD(Planner);
~~~

which when expanded creates a forward declaration of the class _Planner_, and defines the type _PlannerPtr_:

~~~{.cpp}
class Planner;
typedef boost::shared_ptr<Planner> PlannerPtr;
~~~


# Function and Bind (boost/function.hpp and boost/bind.hpp)

[Boost's Function library](http://www.boost.org/libs/function) provides a templated wrapper for function pointers and a generalized framework for [callbacks](http://en.wikipedia.org/wiki/Callback_%28computer_programming%29). It is very easy to encapsulate a function pointer into a single object using Boost. Function objects are templated with the first argument being the return type for the function, and subsequent arguments corresponding to the function's arguments. For example, assume we have a function called _computeMin_ that returns the minimum of two integers:

~~~{.cpp}
int computeMin (int x, int y)
{
    return x < y ? x : y;
}
~~~

We could create a function pointer called _minFuncPtr_ to the _computeMin_ function using C++ syntax like this:

~~~{.cpp}
int (*minFuncPtr)(int, int) = computeMin;
~~~

The same object type using Boost looks like this:

~~~{.cpp}
boost::function<int(int,int)> minFuncPtr(computeMin);
~~~

In addition to improved readability and flexibility, we can also take advantage of the [Bind library](http://www.boost.org/libs/bind) from Boost to create function pointers to class members (not nearly as easy with standard C++ syntax), and fix one or more variables of our function pointer.  In a more complex example, assume we have the class Math:

~~~{.cpp}
class Math
{
    /* ... */

    // Return the minimum of x and y.  If x or y is below a lower bound,
    // return the lower bound instead.
    int boundedMin (int x, int y, int lowerBound = 0)
    {
        if (x > lowerBound && y > lowerBound)
            return (x < y) ? x : y;
        else
            return lowerBound;
    }

    /* ... */
};
~~~

If we wanted to create standard C++ function pointer to _Math::boundedMin_, the code gets a little crazy:

~~~{.cpp}
// Create a function pointer called minFuncPtr
int (Math::*minFuncPtr)(int, int, int);

// Assign the function pointer to the boundedMin method in the Math class.
minFuncPtr = &Math::boundedMin;
~~~

Calling the boundedMin function through the function pointer we just created is equally as obfuscated:

~~~{.cpp}
// Create the Math object so that we have a method to invoke
Math math;

// Wait, what?
int theMin = (math.*minFuncPtr)(1, 2, 0);
~~~

Note that the function pointer and the corresponding invocation had to include ALL arguments to the boundedMin method, even the default arguments. Since the function pointer is a type, all of the arguments supplied to the function are part of the type and we cannot use the default value of 0 for the lowerBound argument.  We can get around all of this by using Boost Functions in conjunction with Boost's Bind library:

~~~{.cpp}
// Create the Math object so that we have a method to invoke
Math math;

// Setup the Boost function pointer as a function that returns an int, and takes two integer arguments
boost::function<int(int,int)> minFuncPtr;

// Initializing the function pointer to the boundedMin method of the math
// instance, and binding the lowerBound argument to zero.
minFuncPtr = boost::bind (&Math::boundedMin, math, _1, _2, 0);

// Invoking the function.  Much better.
int theMin = minFuncPtr (1, 2);
~~~

In a nutshell, boost::bind returns a function pointer to the method given in the first argument, using the instance (“math”) supplied in the second argument. _1 and _2 indicate that the first and second arguments of the boundedMin function will be supplied when the function is called. The zero is permanently bound to the function pointer as the third argument (the default value for “lowerBound”).

[boost::function](http://www.boost.org/libs/function) and [boost::bind](http://www.boost.org/libs/bind) are used in OMPL whenever a callback function is used to invoke methods on a global function or specific instance. These are used to improve readability of the code, as well as to simplify the use of callback functions across object barriers. Also, if an uninitialized Boost function pointer is accessed, a _bad_function_call_ exception is thrown, which is far more desirable than attempting to debug the segmentation fault that would occur with a standard uninitialized C++ function pointer.


# NonCopyable (boost/noncopyable.hpp)

Boost provides a base class called [noncopyable](http://www.boost.org/libs/utility/utility.htm) that classes can derive from in order to prevent them from being copied. noncopyable has a private copy constructor and assignment operator. Therefore, classes deriving from noncopyable are prohibited from invoking these copy operations. Fully utilizing noncopyable is as simple as:

~~~{.cpp}
class MyClass : boost::noncopyable
{
   ...
};
~~~

OMPL derives several classes from boost::noncopyable.  The rationale behind
this is that these classes should never be copied anyway, either because
the copy mechanism is highly non-trivial, copying such an object would be
prohibitive in terms of memory, or both.


# Thread (boost/thread.hpp)

OMPL is designed to be thread-safe, meaning that most of the common API is reentrant (in general, any function marked as _const_ is thread-safe). However, since OMPL is not limited to a single operating system, the creation and management of threads poses a complication since operating systems are free to choose how a developer creates and manages threads. Thankfully, Boost provides a cross-platform [Threading library](http://www.boost.org/libs/thread) to take care of the operating system specific code behind the scenes. boost::thread provides classes for threads and mutexes that allow for multi-threaded computing and synchronization.

It is very easy to create Threads in boost.  Simply create an object of type boost::thread, and supply it the handle to a "callable" object, or function:

~~~{.cpp}
void threadFunction (void) { ... }

// Threads start automatically on creation
boost::thread myThread (threadFunction);
// Wait until the thread has finished execution
myThread.join ();
~~~

~~~{.cpp}
void threadFunction (int someNumber, const char *aString) { ... }

// Create a thread for threadFunction, and pass in some parameters
boost::thread myThread (threadFunction, 42, "A good number");
~~~

~~~{.cpp}
// Create an object for the Thread to operate in.  Thread calls operator () to start.
struct ThreadStruct
{
    void operator () (int param1, const char *param2, const std::vector<double> &param3)
    {
        ...
    }
};

// Create threaded object.
ThreadStruct tStruct;
// Create Thread.  Pass in three parameters.
boost::thread myThread (tStruct, 1, "Two", std::vector <double> ());
~~~

By using [Boost thread](http://www.boost.org/libs/thread), OMPL is able to remain operating system independent in multi-threaded applications, as long as the operating system employed by the user is supported by Boost.
