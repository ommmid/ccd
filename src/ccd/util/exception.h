#pragma once

#include <stdexcept>
#include <string>

// copied from ompl
namespace ccd
{
    /** \brief The exception type for ompl */
    class Exception : public std::runtime_error
    {
    public:
        /** \brief This is just a wrapper on std::runtime_error */
        explicit Exception(const std::string &what) : std::runtime_error(what)
        {
        }

        /** \brief This is just a wrapper on std::runtime_error with a
            prefix added */
        Exception(const std::string &prefix, const std::string &what) : std::runtime_error(prefix + ": " + what)
        {
        }

        ~Exception() noexcept override = default;
    };
}

