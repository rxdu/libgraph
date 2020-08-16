/* 
 * default_comparator.hpp
 * 
 * Created on: Sep 04, 2018 12:02
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef DEFAULT_COMPARATOR_HPP
#define DEFAULT_COMPARATOR_HPP

#include <functional>

namespace librav
{
template <typename Transition>
class DefaultComparator
{
  public:
    bool operator()(Transition lhs, Transition rhs)
    {
        return (lhs->value_ > rhs->value_);
    }

  private:
    // Comparator compare;
};
} // namespace librav

#endif /* DEFAULT_COMPARATOR_HPP */
