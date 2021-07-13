/***************************************************************************************************
 * Copyright Five AI 2020.
 * All rights reserved.
 ***************************************************************************************************/

#pragma once

#include <boost/interprocess/smart_ptr/unique_ptr.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

#include <type_traits>

namespace fiveai { namespace platform { namespace shm
{
    namespace __unique_ptr
    {
        // Need to remove the constness to allow UniquePtr<T> -> UniquePtr<const T> implicit conversions
        template <typename T>
        using Deleter = typename boost::interprocess::managed_shared_memory::deleter<typename std::remove_cv<T>::type>::type;
    }

    template <typename T>
    using UniquePtr = boost::movelib::unique_ptr
    <
        T,
        __unique_ptr::Deleter<T>
    >;

    template <typename T, typename ManagedMemory>
    inline
    UniquePtr<T> makeManagedUniquePtr(T* constructedObject, ManagedMemory& managedMemory)
    {
        using NoConstT = typename std::remove_cv<T>::type;
        return {const_cast<NoConstT*>(constructedObject),
                managedMemory.template get_deleter<NoConstT>()};

    }
}}}

