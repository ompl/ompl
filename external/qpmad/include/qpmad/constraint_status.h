/**
    @file
    @author  Alexander Sherikov

    @copyright 2017 Alexander Sherikov. Licensed under the Apache License,
    Version 2.0. (see LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace qpmad
{
    class ConstraintStatus
    {
    public:
        enum Status
        {
            UNDEFINED = 0,
            INCONSISTENT = 1,
            EQUALITY = 2,
            INACTIVE = 3,
            ACTIVE_LOWER_BOUND = 4,
            ACTIVE_UPPER_BOUND = 5
        };
    };
}  // namespace qpmad
