/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/

#pragma once

#include "util/ShmSharedPtr.h"
#include "util/ShmUniquePtr.h"

namespace ros { namespace shm
{

    template <typename T, typename ManagedMemory>
    inline
    SharedPtr<T> convertToShared(UniquePtr<T> uniquePtr, ManagedMemory& managerMemory)
    {
        using NoConstT = typename std::remove_cv<T>::type;
        static_assert(std::is_same<typename ManagedMemory::template deleter<NoConstT>::type,
                                   typename UniquePtr<NoConstT>::deleter_type>::value,
                      "The deleters type does not coincide!");
        return boost::interprocess::make_managed_shared_ptr(uniquePtr.release().get(), managerMemory);
    }

    template <typename T>
    inline
    UniquePtr<const T> makeConst(UniquePtr<T> ptr)
    {
        return {std::move(ptr)};
    }

    template <typename T>
    inline
    SharedPtr<const T> makeConst(SharedPtr<T> ptr)
    {
        return {std::move(ptr)};
    }
}}

