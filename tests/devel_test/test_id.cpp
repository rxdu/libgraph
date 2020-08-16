#include <iostream>
#include <type_traits> // To use 'std::integral_constant'.
#include <iostream>    // To use 'std::cout'.
#include <iomanip>     // To use 'std::boolalpha'.
#include <memory>

#define GENERATE_HAS_MEMBER(member)                                                   \
                                                                                      \
    template <class T>                                                                \
    class HasMember_##member                                                          \
    {                                                                                 \
      private:                                                                        \
        using Yes = char[2];                                                          \
        using No = char[1];                                                           \
                                                                                      \
        struct Fallback                                                               \
        {                                                                             \
            int member;                                                               \
        };                                                                            \
        struct Derived : T, Fallback                                                  \
        {                                                                             \
        };                                                                            \
                                                                                      \
        template <class U>                                                            \
        static No &test(decltype(U::member) *);                                       \
        template <typename U>                                                         \
        static Yes &test(U *);                                                        \
                                                                                      \
      public:                                                                         \
        static constexpr bool RESULT = sizeof(test<Derived>(nullptr)) == sizeof(Yes); \
    };                                                                                \
                                                                                      \
    template <class T>                                                                \
    struct has_member_##member                                                        \
        : public std::integral_constant<bool, HasMember_##member<T>::RESULT>          \
    {                                                                                 \
    };

GENERATE_HAS_MEMBER(id);
GENERATE_HAS_MEMBER(id_);

struct TestState
{
    int32_t id_;
};

int main(int argc, char **argv)
{
    std::cout << "test id result: " << has_member_id<TestState>::value << std::endl;
    std::cout << "test id_ result: " << has_member_id_<TestState>::value << std::endl;

    std::cout << " ------ " << std::endl;

    std::cout << "test id result: " << has_member_id_<std::remove_pointer<TestState*>::type>::value << std::endl;
    std::cout << "test id result: " << has_member_id_<std::remove_pointer<std::remove_const<const TestState*>::type>::type>::value << std::endl;
    std::cout << "test id result: " << has_member_id<std::remove_pointer<std::remove_const<std::shared_ptr<TestState>>::type>::type>::value << std::endl;

    return 0;
}