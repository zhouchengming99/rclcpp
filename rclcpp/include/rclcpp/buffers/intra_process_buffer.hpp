// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RCLCPP__BUFFERS__INTRA_PROCESS_BUFFER_HPP_
#define RCLCPP__BUFFERS__INTRA_PROCESS_BUFFER_HPP_

#include <memory>
#include <type_traits>
#include <utility>

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/allocator/allocator_deleter.hpp"
#include "rclcpp/buffers/buffer_implementation_base.hpp"

namespace rclcpp
{
namespace intra_process_buffer
{

class IntraProcessBufferBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(IntraProcessBufferBase)

  virtual void clear() = 0;

  virtual bool has_data() const = 0;
  virtual bool use_take_shared_method() const = 0;
};

template<
  typename MessageT,
  typename Alloc = std::allocator<void>>
class IntraProcessBuffer : public IntraProcessBufferBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(IntraProcessBuffer)

  using MessageAllocTraits = allocator::AllocRebind<MessageT, Alloc>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;
  using MessageDeleter = allocator::Deleter<MessageAlloc, MessageT>;
  using MessageUniquePtr = std::unique_ptr<MessageT, MessageDeleter>;
  using MessageSharedPtr = std::shared_ptr<const MessageT>;

  virtual void add_shared(MessageSharedPtr msg) = 0;
  virtual void add_unique(MessageUniquePtr msg) = 0;

  virtual MessageSharedPtr consume_shared() = 0;
  virtual MessageUniquePtr consume_unique() = 0;
};

template<
  typename MessageT,
  typename Alloc = std::allocator<void>,
  typename BufferT = std::unique_ptr<MessageT>>
class TypedIntraProcessBuffer : public IntraProcessBuffer<MessageT, Alloc>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(TypedIntraProcessBuffer)

  using MessageAllocTraits = allocator::AllocRebind<MessageT, Alloc>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;
  using MessageDeleter = allocator::Deleter<MessageAlloc, MessageT>;
  using MessageUniquePtr = std::unique_ptr<MessageT, MessageDeleter>;
  using MessageSharedPtr = std::shared_ptr<const MessageT>;
  static_assert(std::is_same<BufferT, MessageSharedPtr>::value ||
    std::is_same<BufferT, MessageUniquePtr>::value,
    "BufferT is not a valid type");

  TypedIntraProcessBuffer(
    std::unique_ptr<BufferImplementationBase<BufferT>> buffer_impl)
  {
    buffer_ = std::move(buffer_impl);
  }

  void add_shared(MessageSharedPtr msg)
  {
    add_shared_impl<BufferT>(std::move(msg));
  }

  void add_unique(MessageUniquePtr msg)
  {
    add_unique_impl<BufferT>(std::move(msg));
  }

  MessageSharedPtr consume_shared()
  {
    return consume_shared_impl<BufferT>();
  }

  MessageUniquePtr consume_unique()
  {
    return consume_unique_impl<BufferT>();
  }

  bool has_data() const
  {
    return buffer_->has_data();
  }

  void clear()
  {
    buffer_->clear();
  }

  bool use_take_shared_method() const
  {
    return std::is_same<BufferT, MessageSharedPtr>::value;
  }

private:
  std::unique_ptr<BufferImplementationBase<BufferT>> buffer_;

  // MessageSharedPtr to MessageSharedPtr
  template<typename DestinationT>
  typename std::enable_if<
    std::is_same<DestinationT, MessageSharedPtr>::value
  >::type
  add_shared_impl(MessageSharedPtr shared_msg)
  {
    buffer_->enqueue(std::move(shared_msg));
  }

  // MessageSharedPtr to MessageUniquePtr
  template<typename DestinationT>
  typename std::enable_if<
    std::is_same<DestinationT, MessageUniquePtr>::value
  >::type
  add_shared_impl(MessageSharedPtr shared_msg)
  {
    auto unique_msg = std::make_unique<MessageT>(*shared_msg);
    buffer_->enqueue(std::move(unique_msg));
  }

  // MessageUniquePtr to MessageSharedPtr
  template<typename DestinationT>
  typename std::enable_if<
    std::is_same<DestinationT, MessageSharedPtr>::value
  >::type
  add_unique_impl(MessageUniquePtr unique_msg)
  {
    buffer_->enqueue(std::move(unique_msg));
  }

  // MessageUniquePtr to MessageUniquePtr
  template<typename DestinationT>
  typename std::enable_if<
    std::is_same<DestinationT, MessageUniquePtr>::value
  >::type
  add_unique_impl(MessageUniquePtr unique_msg)
  {
    buffer_->enqueue(std::move(unique_msg));
  }

  // MessageSharedPtr to MessageSharedPtr
  template<typename OriginT>
  typename std::enable_if<
    (std::is_same<OriginT, MessageSharedPtr>::value),
    MessageSharedPtr
  >::type
  consume_shared_impl()
  {
    return buffer_->dequeue();
  }

  // MessageUniquePtr to MessageSharedPtr
  template<typename OriginT>
  typename std::enable_if<
    (std::is_same<OriginT, MessageUniquePtr>::value),
    MessageSharedPtr
  >::type
  consume_shared_impl()
  {
    // automatic cast from unique ptr to shared ptr
    return buffer_->dequeue();
  }

  // MessageSharedPtr to MessageUniquePtr
  template<typename OriginT>
  typename std::enable_if<
    (std::is_same<OriginT, MessageSharedPtr>::value),
    MessageUniquePtr
  >::type
  consume_unique_impl()
  {
    MessageSharedPtr buffer_msg = buffer_->dequeue();
    return std::make_unique<MessageT>(*buffer_msg);
  }

  // MessageUniquePtr to MessageUniquePtr
  template<typename OriginT>
  typename std::enable_if<
    (std::is_same<OriginT, MessageUniquePtr>::value),
    MessageUniquePtr
  >::type
  consume_unique_impl()
  {
    return buffer_->dequeue();
  }
};

}  // namespace intra_process_buffer
}  // namespace rclcpp


#endif  // RCLCPP__BUFFERS__INTRA_PROCESS_BUFFER_HPP_
