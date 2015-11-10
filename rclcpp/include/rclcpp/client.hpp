// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__CLIENT_HPP_
#define RCLCPP__CLIENT_HPP_

#include <future>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"

namespace rclcpp
{
namespace client
{

class ClientBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(ClientBase);

  RCLCPP_PUBLIC
  ClientBase(
    std::shared_ptr<rmw_node_t> node_handle,
    rmw_client_t * client_handle,
    const std::string & service_name);

  RCLCPP_PUBLIC
  virtual ~ClientBase();

  RCLCPP_PUBLIC
  const std::string &
  get_service_name() const;

  RCLCPP_PUBLIC
  const rmw_client_t *
  get_client_handle() const;

  virtual std::shared_ptr<void> create_response() = 0;
  virtual std::shared_ptr<void> create_request_header() = 0;
  virtual void handle_response(
    const std::shared_ptr<void> & request_header, const std::shared_ptr<void> & response) = 0;

private:
  RCLCPP_DISABLE_COPY(ClientBase);

  std::shared_ptr<rmw_node_t> node_handle_;

  rmw_client_t * client_handle_;
  std::string service_name_;
};

template<typename ServiceT, typename Alloc = std::allocator<void>>
class Client : public ClientBase
{
public:
  using RequestAllocTraits = allocator::AllocRebind<typename ServiceT::Request, Alloc>;
  using RequestAlloc = typename RequestAllocTraits::allocator_type;

  using ResponseAllocTraits = allocator::AllocRebind<typename ServiceT::Response, Alloc>;
  using ResponseAlloc = typename ResponseAllocTraits::allocator_type;
  using ResponseDeleter = allocator::Deleter<ResponseAlloc, typename ServiceT::Response>;


  using HeaderAllocTraits = allocator::AllocRebind<rmw_request_id_t, Alloc>;
  using HeaderAlloc = typename HeaderAllocTraits::allocator_type;
  using HeaderDeleter = allocator::Deleter<HeaderAlloc, rmw_request_id_t>;

  using Promise = std::promise<typename ServiceT::Response::SharedPtr>;
  using SharedPromise = std::shared_ptr<Promise>;
  using SharedFuture = std::shared_future<typename ServiceT::Response::SharedPtr>;

  using PromiseAllocTraits = allocator::AllocRebind<Promise, Alloc>;
  using PromiseAlloc = typename PromiseAllocTraits::allocator_type;

  using CallbackType = std::function<void(SharedFuture)>;

  using RequestTuple = std::tuple<SharedPromise, CallbackType, SharedFuture>;
  using RequestTupleAllocTraits = allocator::AllocRebind<RequestTuple, Alloc>;
  using RequestTupleAlloc = typename RequestTupleAllocTraits::allocator_type;


  RCLCPP_SMART_PTR_DEFINITIONS(Client);
  Client(
    std::shared_ptr<rmw_node_t> node_handle,
    rmw_client_t * client_handle,
    const std::string & service_name,
    std::shared_ptr<Alloc> allocator = nullptr)
  : ClientBase(node_handle, client_handle, service_name)
  {
    if (!allocator) {
      allocator = std::make_shared<Alloc>();
    }
    tuple_allocator_ = std::make_shared<RequestTupleAlloc>(*allocator.get());
    request_allocator_ = std::make_shared<RequestAlloc>(*allocator.get());
    response_allocator_ = std::make_shared<ResponseAlloc>(*allocator.get());
    header_allocator_ = std::make_shared<HeaderAlloc>(*allocator.get());
    promise_allocator_ = std::make_shared<PromiseAlloc>(*allocator.get());

    response_deleter_ = std::make_shared<ResponseDeleter>();
    header_deleter_ = std::make_shared<HeaderDeleter>();
    rclcpp::allocator::set_allocator_for_deleter(response_deleter_.get(), response_allocator_.get());
    rclcpp::allocator::set_allocator_for_deleter(header_deleter_.get(), header_allocator_.get());
  }

  std::shared_ptr<void> create_response()
  {
    return std::allocate_shared<typename ServiceT::Response>(*response_allocator_.get());
  }

  std::shared_ptr<void> create_request_header()
  {
    // TODO(wjwwood): This should probably use rmw_request_id's allocator.
    //                (since it is a C type)
    return std::allocate_shared<rmw_request_id_t>(*header_allocator_.get());
  }

  void handle_response(const std::shared_ptr<void> & request_header, const std::shared_ptr<void> & response)
  {
    // TODO: static_pointer_cast allocations
    //auto typed_request_header = std::static_pointer_cast<rmw_request_id_t>(request_header);
    auto typed_request_header =
      rclcpp::allocator::allocator_static_pointer_cast<rmw_request_id_t>(
        request_header, *header_allocator_.get());
    int64_t sequence_number = typed_request_header->sequence_number;
    //auto typed_response = std::static_pointer_cast<typename ServiceT::Response>(response);
    auto typed_response =
      rclcpp::allocator::allocator_static_pointer_cast<typename ServiceT::Response>(
        response, *response_allocator_.get());

    // std::map::at will throw an exception if sequence_number is invalid
    RequestTuple tuple = std::move(this->pending_requests_.at(sequence_number));
    auto call_promise = std::move(std::get<0>(tuple));
    call_promise->set_value(typed_response);
    auto callback = std::move(std::get<1>(tuple));
    if (callback) {
      auto future = std::move(std::get<2>(tuple));
      callback(future);
    }
    this->pending_requests_.erase(sequence_number);
  }

  SharedFuture async_send_request(
    typename ServiceT::Request::SharedPtr request)
  {
    return async_send_request(request, nullptr);
  }

  SharedFuture async_send_request(
    typename ServiceT::Request::SharedPtr request,
    CallbackType && cb)
  {
    int64_t sequence_number;
    if (RMW_RET_OK != rmw_send_request(get_client_handle(), request.get(), &sequence_number)) {
      // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
      throw std::runtime_error(
        std::string("failed to send request: ") + rmw_get_error_string_safe());
      // *INDENT-ON*
    }

    SharedPromise call_promise = std::allocate_shared<Promise>(*promise_allocator_.get(),
      std::allocator_arg, *promise_allocator_.get());
    SharedFuture f(call_promise->get_future());

    auto tup = RequestTuple(std::move(call_promise), std::forward<CallbackType>(cb), f);
    pending_requests_.emplace(sequence_number, std::move(tup));

    return f;
  }

private:
  RCLCPP_DISABLE_COPY(Client);

  using RequestMap = std::map<int64_t, RequestTuple, std::less<int64_t>,
    typename allocator::AllocRebind<typename std::pair<int64_t, RequestTuple>, Alloc>::allocator_type>;
  RequestMap pending_requests_;

  std::shared_ptr<RequestTupleAlloc> tuple_allocator_;
  std::shared_ptr<RequestAlloc> request_allocator_;
  std::shared_ptr<ResponseAlloc> response_allocator_;
  std::shared_ptr<ResponseDeleter> response_deleter_;
  std::shared_ptr<HeaderAlloc> header_allocator_;
  std::shared_ptr<HeaderDeleter> header_deleter_;
  std::shared_ptr<PromiseAlloc> promise_allocator_;
};

}  // namespace client
}  // namespace rclcpp

#endif  // RCLCPP__CLIENT_HPP_
