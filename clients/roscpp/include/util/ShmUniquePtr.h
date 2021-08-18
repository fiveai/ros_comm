/***************************************************************************************************
 * Copyright Five AI Limited 2021
 * All rights reserved

 * This file is released under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 * International License. Please refer to the README.md file supplied with this repository.
 ***************************************************************************************************/

#pragma once

#include <boost/interprocess/smart_ptr/unique_ptr.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

#include <type_traits>

namespace ros { namespace shm
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
}}
