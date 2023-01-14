/*
 * default_indexer.hpp
 *
 * Created on: Sep 03, 2018 06:57
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

/*
 * Reference:
 *
 * Member detector:
 * [1] https://en.wikibooks.org/wiki/More_C%2B%2B_Idioms/Member_Detector
 * [2] https://jguegant.github.io/blogs/tech/sfinae-introduction.html
 *
 * Check std::shared_ptr
 * [3]
 * https://stackoverflow.com/questions/10539305/generic-way-to-test-if-a-type-is-a-smart-pointer
 *
 * Access value/pointer types
 * [4]
 * https://stackoverflow.com/questions/14466620/c-template-specialization-calling-methods-on-types-that-could-be-pointers-or/14466705
 *
 * SFINAE:
 * [5] https://cpppatterns.com/patterns/class-template-sfinae.html
 *
 */

#ifndef STATE_INDEXER_HPP
#define STATE_INDEXER_HPP

#include <iostream>
#include <type_traits>
#include <memory>
#include <iostream>
#include <iomanip>

#if __cplusplus <= 201703L
template <class...>
using void_t = void;
#endif

#if __cplusplus < 201402L
namespace std {
template <bool B, class T = void>
using enable_if_t = typename enable_if<B, T>::type;
}
#endif

namespace robosw {
#define GENERATE_HAS_MEM_VAR(member)                   \
                                                       \
  template <class T>                                   \
  class HasMember_##member {                           \
   private:                                            \
    using Yes = char[2];                               \
    using No = char[1];                                \
                                                       \
    struct Fallback {                                  \
      int member;                                      \
    };                                                 \
    struct Derived : T, Fallback {};                   \
                                                       \
    template <class U>                                 \
    static No &test(decltype(U::member) *);            \
    template <typename U>                              \
    static Yes &test(U *);                             \
                                                       \
   public:                                             \
    static constexpr bool RESULT =                     \
        sizeof(test<Derived>(nullptr)) == sizeof(Yes); \
  };                                                   \
                                                       \
  template <class T>                                   \
  struct has_memvar_##member                           \
      : public std::integral_constant<bool, HasMember_##member<T>::RESULT> {};

#define GENERATE_HAS_MEM_FUNC(member)                          \
                                                               \
  template <typename T, typename = void>                       \
  struct has_memfunc_##member : std::false_type {};            \
                                                               \
  template <typename T>                                        \
  struct has_memfunc_##member<                                 \
      T, void_t<decltype(std::declval<T>().member() == true)>> \
      : std::true_type {};

GENERATE_HAS_MEM_VAR(id);
GENERATE_HAS_MEM_VAR(id_);
GENERATE_HAS_MEM_FUNC(GetId);

template <typename T>
struct is_shared_ptr : std::false_type {};

template <typename T>
struct is_shared_ptr<std::shared_ptr<T>> : std::true_type {};

template <typename T>
T *get_ptr(T &obj) {
  return &obj;
}

template <typename T>
T *get_ptr(const T &obj) {
  return &obj;
}

template <typename T>
T *get_ptr(T *obj) {
  return obj;
}

template <typename T>
std::shared_ptr<T> get_ptr(std::shared_ptr<T> obj) {
  return obj;
}

/*---------------------------------------------------------------------------------*/

template <typename T>
using check_GetId =
    has_memfunc_GetId<typename std::remove_pointer<typename std::remove_const<
        typename std::remove_reference<T>::type>::type>::type>;

template <typename T>
using check_id =
    has_memvar_id<typename std::remove_pointer<typename std::remove_const<
        typename std::remove_reference<T>::type>::type>::type>;

template <typename T>
using check_id_ =
    has_memvar_id_<typename std::remove_pointer<typename std::remove_const<
        typename std::remove_reference<T>::type>::type>::type>;

template <typename State>
struct DefaultIndexer {
  // check GetId()
  template <typename T = State,
            std::enable_if_t<check_GetId<T>::value, bool> = true>
  int64_t operator()(T state) const {
    return static_cast<int64_t>(get_ptr(state)->GetId());
  }

  template <typename T = State,
            std::enable_if_t<check_GetId<T>::value, bool> = true>
  int64_t operator()(std::shared_ptr<T> state) const {
    return static_cast<int64_t>(get_ptr(state)->GetId());
  }

  // check id
  template <typename T = State,
            std::enable_if_t<!check_GetId<T>::value && check_id<T>::value,
                             bool> = true>
  int64_t operator()(T state) const {
    return static_cast<int64_t>(get_ptr(state)->id);
  }

  template <typename T = State,
            std::enable_if_t<!check_GetId<T>::value && check_id<T>::value,
                             bool> = true>
  int64_t operator()(std::shared_ptr<T> state) const {
    return static_cast<int64_t>(get_ptr(state)->id);
  }

  // check id_
  template <typename T = State,
            std::enable_if_t<!check_GetId<T>::value && check_id_<T>::value,
                             bool> = true>
  int64_t operator()(T state) const {
    return static_cast<int64_t>(get_ptr(state)->id_);
  }

  template <typename T = State,
            std::enable_if_t<!check_GetId<T>::value && check_id_<T>::value,
                             bool> = true>
  int64_t operator()(std::shared_ptr<T> state) const {
    return static_cast<int64_t>(get_ptr(state)->id_);
  }
};
}  // namespace robosw

#endif /* STATE_INDEXER_HPP */
