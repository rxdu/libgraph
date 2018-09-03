#ifndef STATE_INDEXER_HPP
#define STATE_INDEXER_HPP

namespace librav
{
template <typename State>
struct DefaultIndexer
{
    template <class T = State, typename std::enable_if<!std::is_pointer<T>::value>::type * = nullptr>
    int64_t operator()(const State &state) const { return state.id_; }

    template <class T = State, typename std::enable_if<std::is_pointer<T>::value>::type * = nullptr>
    int64_t operator()(const State state) const { return state->id_; }
};
} // namespace librav

#endif /* STATE_INDEXER_HPP */
