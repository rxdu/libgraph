#ifndef STATE_INDEXER_HPP
#define STATE_INDEXER_HPP

#include <memory>
#include <type_traits>

namespace librav
{
// Reference:
// [1] https://stackoverflow.com/questions/10539305/generic-way-to-test-if-a-type-is-a-smart-pointer
template<typename T> struct is_shared_ptr : std::false_type {};
template<typename T> struct is_shared_ptr<std::shared_ptr<T>> : std::true_type {};

template <typename State>
struct DefaultIndexer
{
    template <class T = State, typename std::enable_if<!std::is_pointer<T>::value && !is_shared_ptr<T>::value>::type * = nullptr>
    int64_t operator()(const State &state) const { return static_cast<int64_t>(state.id_); }

    template <class T = State, typename std::enable_if<std::is_pointer<T>::value || is_shared_ptr<T>::value>::type * = nullptr>
    int64_t operator()(State state) const { return static_cast<int64_t>(state->id_); }
};
} // namespace librav

#endif /* STATE_INDEXER_HPP */
