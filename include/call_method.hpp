#ifndef SHIZU_CALL_METHOD_HPP
#define SHIZU_CALL_METHOD_HPP

#include <obj_api.hpp>
#include <object_id.hpp>
#include <svc.hpp> // error_t (call_error の型付き戻り)

// メソッド呼び出し。戻りの error_t<call_error> で成否が取れる:
//   OK                = 呼び出し成功 (メソッドは実行済み)
//   UNDECLARED_METHOD = 呼び先が未 export (まだ main/export_method が走っていない)
//   BAD_OBJECT        = obj/method 番号が範囲外 or 対象オブジェクト未生成
// affinity でオブジェクトのコアを移すと「スレッド番号昇順の初回実行順」による
// export 順の保証が崩れるため、起動直後の呼び出しは is_err() を見て yield →
// 再試行すること (従来はカーネルが panic 停止していた)。戻り値を無視する既存
// 呼び出し元はそのままコンパイルできる (エラーは黙って捨てられる)。
[[maybe_unused]] static error_t<shizu::call_error>
call_method(object_ids callee_obj_id, uint32_t callee_method_id,
            uint32_t arg0) {
  auto r = shizu::obj_api::svci<shizu::obj_api::svc_num::CALL_METHOD>(
      (uint32_t)callee_obj_id, callee_method_id, arg0);
  return error_t<shizu::call_error>{
      static_cast<shizu::call_error>((uint32_t)r.result)};
}

[[maybe_unused]] static error_t<shizu::call_error>
call_method_via_md(uint32_t md, uint32_t arg0, uint32_t arg1 = 0) {
  auto r = shizu::obj_api::svci<shizu::obj_api::svc_num::CALL_METHOD_VIA_MD>(
      md, arg0, arg1);
  return error_t<shizu::call_error>{
      static_cast<shizu::call_error>((uint32_t)r.result)};
};

#endif // SHIZU_CALL_METHOD_HPP
