# OMPL Style Guide

This document provides a brief overview of OMPL coding standards. It is meant to assist developers who are contributing code to OMPL.


# Spacing

- Each source block should be indented by 4 space characters. No tab characters should appear in source code.
- Each enclosing brace should reside on its own line, but braces can be omitted if the enclosed block consists of exactly one line.

The following is an example of desirable code spacing.

~~~{.cpp}
for (unsigned int i = 0 ; i < 50 ; ++i)
{
    if (i % 2 == 0)
        std::cout << i << " ";
}
~~~


# Filenames & Macros

- Source filenames should begin each word with a capital letter, and there should be no underscores between words.
- Header files should use the .h extension, and implementation files should use the .cpp extension.
- Each implementation file should reside in a src/ directory, immediately below the directory containing the corresponding header file.
- Include guards in header files should be of the form OMPL_PATH_FILENAME_, where the words in the filename are separated by underscores.

For example, the header and implementation files in which ompl::RNG is defined are ompl/util/RandomNumbers.h and ompl/util/src/RandomNumbers.cpp, respectively. The header file begins with the include guard

~~~{.cpp}
#ifndef OMPL_UTIL_RANDOM_NUMBERS_
~~~


# Functions & Classes

- Function names should begin with a lower-case letter and should begin each subsequent word with a capital letter, and there should be no underscores between words (e.g., ompl::base::StateSpace::setName()).
- Functions accepting no parameters should have void in place of a parameter list.
- Class names should begin each word with a capital letter, again with no underscores (e.g., ompl::base::StateSpace).
- Scoping directives (_public_, _protected_, _private_) should be at the same level of indentation as the class declaration.
- Class member variable names should begin with a lower-case letter and should begin each subsequent word with a capital letter. They should end with a single underscore (e.g., ompl::base::StateSpace::longestValidSegment_).
- Names of constants and static variables should be in all capital letters, with an underscore between each word (e.g., ompl::base::StateSpace::DEFAULT_PROJECTION_NAME).

For example, consider the following source code, which follows the above guidelines.

~~~{.cpp}
class SampleObject
{
public:
    SampleObject(void) : objectTag_(NUM_INSTANCES++)
    {
    }

    int getObjectTag(void) const
    {
        return objectTag_;
    }

    static int NUM_INSTANCES;

private:
    const int objectTag_;
};
int SampleObject::NUM_INSTANCES = 0;
~~~


# Other Coding Guidelines

- Compiler specific features should never be used (avoid use of \#ifdef).
- Code must compile without warnings.
- If a class member function can be marked _const_, then it should be marked _const_.
- Member functions marked as _const_ must be thread safe. If a function is intended to be thread safe, it should be marked as _const_ (one can use the _mutable_ keyword if needed).
- All classes, methods and member variables must be documented (Doxygen style). High-level documentation can be written either in [MarkDown format](http://www.stack.nl/~dimitri/doxygen/markdown.html) or in Doxygen format.
- Forward declaration of types should be done with the OMPL_CLASS_FORWARD() macro when shared pointers to the type are also needed: OMPL_CLASS_FORWARD() will define a shared pointer to the type using the _Ptr_ suffix (e.g., ompl::base::StateSpacePtr).
