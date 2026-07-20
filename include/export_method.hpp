#ifndef SHIZU_EXPORT_METHOD_HPP
#define SHIZU_EXPORT_METHOD_HPP
#include <concepts>
#include <obj_api.hpp>

template <auto T>
  requires std::invocable<decltype(T), uint32_t, uint32_t, uint32_t, uint32_t>
void wrapper(uint32_t r0, uint32_t r1, uint32_t r2, uint32_t r3) {
  T(r0, r1, r2, r3);
  shizu::obj_api::svci<shizu::obj_api::svc_num::EXIT_METHOD>();
}

template <auto T>
  requires std::invocable<decltype(T), uint32_t, uint32_t, uint32_t, uint32_t>
void export_method(uint32_t num = 0) {
  shizu::obj_api::svci<shizu::obj_api::svc_num::EXPORT_METHOD>(
      num, reinterpret_cast<uint32_t>(wrapper<T>));
}
#endif // SHIZU_EXPORT_METHOD_HPP