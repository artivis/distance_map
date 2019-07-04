/*
 * Copyright 2019 Jeremie Deray
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Author: Jeremie Deray
 */

#ifndef SINGLETON_H_
#define SINGLETON_H_

#include <memory>
#include <assert.h>

namespace distmap {
namespace details {

/**
* \brief A thread-safer? Singleton implementation with
* argument forwarding.
**/
template <class T>
class Singleton
{
  /**
  * \brief Custom deleter to by-pass private destructor issue.
  **/
  struct Deleter;

  using SingletonOPtr = std::unique_ptr<T, Deleter>;

public:

  template <typename... Args>
  static T& get(Args&&... args)
  {
    // c++11 equivalent to `std::make_unique`
    static SingletonOPtr instance_(new T(std::forward<Args>(args)...));

    assert(instance_ != nullptr);

    return *instance_;
  }

  constexpr Singleton(const Singleton&)       = delete;
  //constexpr Singleton(const Singleton&&)      = delete;

  constexpr Singleton& operator=(const Singleton&)  = delete;
  //constexpr Singleton& operator=(const Singleton&&) = delete;

protected:

  Singleton()          = default;
  virtual ~Singleton() = default;
};

template <class T>
struct Singleton<T>::Deleter
{
  void operator()( const T* const /*p*/ )
  {
    //
  }
};

} /* namespace details */
} /* namespace distmap */

#endif /* SINGLETON_H_ */
