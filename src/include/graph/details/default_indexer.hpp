/*
 * default_indexer.hpp
 *
 * Created on: Sep 03, 2018 06:57
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef STATE_INDEXER_HPP
#define STATE_INDEXER_HPP

#include <iostream>
#include <type_traits>
#include <memory>
#include <iostream>
#include <iomanip>

namespace rdu {
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

#define GENERATE_HAS_MEMBER(member)                    \
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
  struct has_member_##member                           \
      : public std::integral_constant<bool, HasMember_##member<T>::RESULT> {};

GENERATE_HAS_MEMBER(id);
GENERATE_HAS_MEMBER(id_);

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

template <typename State>
struct DefaultIndexer {
  // check id
  template <
      typename T = State,
      std::enable_if_t<
          has_member_id<typename std::remove_pointer<typename std::remove_const<
              typename std::remove_reference<T>::type>::type>::type>::value,
          bool> = true>
  int64_t operator()(T state) const {
    return static_cast<int64_t>(get_ptr(state)->id);
  }

  template <
      typename T = State,
      std::enable_if_t<
          has_member_id<typename std::remove_pointer<typename std::remove_const<
              typename std::remove_reference<T>::type>::type>::type>::value,
          bool> = true>
  int64_t operator()(std::shared_ptr<T> state) const {
    return static_cast<int64_t>(get_ptr(state)->id);
  }

  // check id_
  template <
      typename T = State,
      std::enable_if_t<
          has_member_id_<
              typename std::remove_pointer<typename std::remove_const<
                  typename std::remove_reference<T>::type>::type>::type>::value,
          bool> = true>
  int64_t operator()(T state) const {
    return static_cast<int64_t>(get_ptr(state)->id_);
  }

  template <
      typename T = State,
      std::enable_if_t<
          has_member_id_<
              typename std::remove_pointer<typename std::remove_const<
                  typename std::remove_reference<T>::type>::type>::type>::value,
          bool> = true>
  int64_t operator()(std::shared_ptr<T> state) const {
    return static_cast<int64_t>(get_ptr(state)->id_);
  }
};
}  // namespace rdu

#endif /* STATE_INDEXER_HPP */
