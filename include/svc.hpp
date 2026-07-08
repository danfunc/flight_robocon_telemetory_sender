#ifndef SHIZU_SVC_HPP
#define SHIZU_SVC_HPP

#include <cstdint>
#include <type_traits>

template <typename T>
  requires(sizeof(T) <= 4)
struct result_t {
  enum struct state_t : uint32_t { SUCCESS = 0, FAILURE } result;
  uint32_t value;
  T get_value() { return static_cast<T>(value); }
};

// ===========================================================================
//  error_t<ERROR_ENUM_TYPE> — SVC ごとに型付けされた戻りエラー (r0)
// ===========================================================================
//  従来の result_t は r0 を固定の {SUCCESS=0, FAILURE=1} でしか表現できなかった。
//  claim / stream など「失敗理由が複数ある」SVC のために、r0 を呼び出し側が指定した
//  enum (ERROR_ENUM_TYPE) として解釈する薄いラッパを用意する。
//
//  【ABI 規約】enum 値 0 == 成功。これはカーネル SVC の「r0==0 で成功」という既存
//  ワイヤ規約と一致するので、error_t は古い result_t とワイヤ互換 (r0 の 0/非0 の
//  意味が変わらない)。各 SVC は自分専用の enum を定義し、OK=0 を必ず先頭に置く。
//
//  意図的に operator bool を提供しない: `if (err)` が「成功」か「エラーあり」かは
//  読み手で解釈が割れる (footgun) ため、常に is_ok()/is_err() を明示させる。
template <typename E>
  requires std::is_enum_v<E>
struct error_t {
  E code; // r0 を ERROR_ENUM_TYPE として解釈したもの (0 == 成功)
  constexpr bool is_ok() const {
    return static_cast<std::underlying_type_t<E>>(code) == 0;
  }
  constexpr bool is_err() const { return !is_ok(); }
  constexpr E get() const { return code; }
  constexpr uint32_t raw() const {
    return static_cast<uint32_t>(static_cast<std::underlying_type_t<E>>(code));
  }
};

// SVC の戻り「error_t と result の二つ」を1つにまとめた型。
//   .error  = error_t<E>  (r0, 型付きエラー)
//   .result = V           (r1, 値。デフォルト uint32_t)
template <typename E, typename V = uint32_t>
  requires(std::is_enum_v<E> && sizeof(V) <= 4)
struct svc_result_t {
  error_t<E> error;
  V result;
  constexpr bool is_ok() const { return error.is_ok(); }
};

template <uintptr_t sys_call_num>
  requires(sys_call_num <= 255) // SVC instruction immediate must be under 256.
static result_t<uint32_t> svc(uintptr_t arg0, uintptr_t arg1, uintptr_t arg2,
                              uintptr_t arg3, uintptr_t r12 = 0) {
  uint32_t return_code, value;
  asm volatile(
      "mov r0,%[arg0];"
      "mov r1,%[arg1];"
      "mov r2,%[arg2];"
      "mov r3,%[arg3];"
      "mov r12,%[r12];"
      "svc %[sys_call_num];"
      "mov %[return_code],r0;"
      "mov %[value],r1"
      : [return_code] "=r"(return_code), [value] "=r"(value)
      : [arg0] "r"(arg0), [arg1] "r"(arg1), [arg2] "r"(arg2), [arg3] "r"(arg3),
        [r12] "r"(r12), [sys_call_num] "i"(sys_call_num)
      : "r0", "r1", "r2", "r3", "ip", "lr", "memory");
  return {.result = (result_t<uint32_t>::state_t)return_code, .value = value};
};

// 型付きエラー版の SVC 呼び出し。既存 svc<N> を包み、r0 を ERROR_ENUM_TYPE(E) と
// して、r1 を V として解釈して返す。新設の claim / stream 系 SVC はこちらを使う。
//   例: auto r = svc_typed<TRY_SWITCH_THREAD, switch_error>(next_tid,0,0,0);
//       if (r.error.is_err()) { switch (r.error.get()) { ... } }
template <uintptr_t sys_call_num, typename E, typename V = uint32_t>
  requires(sys_call_num <= 255 && std::is_enum_v<E> && sizeof(V) <= 4)
static svc_result_t<E, V> svc_typed(uintptr_t arg0, uintptr_t arg1,
                                    uintptr_t arg2, uintptr_t arg3,
                                    uintptr_t r12 = 0) {
  result_t<uint32_t> r = svc<sys_call_num>(arg0, arg1, arg2, arg3, r12);
  return {.error = error_t<E>{static_cast<E>((uint32_t)r.result)},
          .result = static_cast<V>(r.value)};
}

#endif // SHIZU_SVC_HPP