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
 * SFINAE:
 * [4] https://cpppatterns.com/patterns/class-template-sfinae.html
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

template <typename State, typename Enable = void>
struct DefaultIndexer;

// when State has a member "id_"
template <typename State>
struct DefaultIndexer<
    State, typename std::enable_if<has_member_id_<typename std::remove_pointer<
               typename std::remove_const<typename std::remove_reference<
                   State>::type>::type>::type>::value>::type> {
  template <
      typename T = State,
      typename std::enable_if<(!std::is_pointer<T>::value &&
                               !is_shared_ptr<T>::value)>::type * = nullptr>
  int64_t operator()(const State &state) const {
    return static_cast<int64_t>(state.id_);
  }

  template <typename T = State, typename std::enable_if<(
                                    std::is_pointer<T>::value ||
                                    is_shared_ptr<T>::value)>::type * = nullptr>
  int64_t operator()(State state) const {
    return static_cast<int64_t>(state->id_);
  }
};

// when State has a member "id"
template <typename State>
struct DefaultIndexer<
    State, typename std::enable_if<has_member_id<typename std::remove_pointer<
               typename std::remove_const<typename std::remove_reference<
                   State>::type>::type>::type>::value>::type> {
  template <
      typename T = State,
      typename std::enable_if<(!std::is_pointer<T>::value &&
                               !is_shared_ptr<T>::value)>::type * = nullptr>
  int64_t operator()(const State &state) const {
    return static_cast<int64_t>(state.id);
  }

  template <typename T = State, typename std::enable_if<(
                                    std::is_pointer<T>::value ||
                                    is_shared_ptr<T>::value)>::type * = nullptr>
  int64_t operator()(State state) const {
    return static_cast<int64_t>(state->id);
  }
};
}  // namespace rdu

#endif /* STATE_INDEXER_HPP */
