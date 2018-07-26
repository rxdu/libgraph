/* 
 * helper_func.hpp
 * 
 * Created on: Nov 21, 2017 22:35
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef HELPER_FUNC_HPP
#define HELPER_FUNC_HPP

#include <cstdint>

namespace librav
{

/*
 * Description: The template `HasIDGenFunc<T>` exports a
 *    boolean constant `value` that is true iff `T` provides
 *    `int64_t GetUniqueID() const`
 *
 * Source: http://clauandr.sdf-eu.org/blog/2014/01/09/c-template-to-check-for-a-functions-existence-with-sfinae/
 *         https://stackoverflow.com/questions/12015195/how-to-call-member-function-only-if-object-happens-to-have-it
 */
template <typename T>
struct HasIDGenFunc
{
    /* SFINAE foo-has-correct-sig */
    template <typename A>
    static std::true_type test(int64_t (A::*)() const)
    {
        return std::true_type();
    }

    /* SFINAE foo-exists  */
    template <typename A>
    static decltype(test(&A::GetUniqueID))
    test(decltype(&A::GetUniqueID), void *)
    {
        /* foo exists. What about sig? */
        typedef decltype(test(&A::GetUniqueID)) return_type;
        return return_type();
    }

    /* SFINAE game over */
    template <typename A>
    static std::false_type test(...)
    {
        return std::false_type();
    }

    /* This will be either `std::true_type` or `std::false_type` */
    typedef decltype(test<T>(0, 0)) type;

    static const bool value = type::value; /* Which is it? */
};
}

#endif /* HELPER_FUNC_HPP */
