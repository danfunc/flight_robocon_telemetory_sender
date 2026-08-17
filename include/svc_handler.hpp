#ifndef SHIZU_SVC_HANDLER_HPP
#define SHIZU_SVC_HANDLER_HPP
#include <concepts>
#include <cstdint>
#include <kernel.hpp>

// ===========================================================================
//  svc ハンドラの ABI シム
// ===========================================================================
//  カーネルはハンドラ起動時、呼び出し情報を **callee-saved レジスタ** に載せる:
//    r4 = svc 番号 / r5 = 発行元 obj / r6 = 発行元 thread /
//    r7 = 今のネスト数 (この活性化が走り出す時点の call_stack 深さ)、引数は r0..r3。
//  C の関数はこれを第 5..8 引数として受け取れないので、naked ラッパで
//  {r4-r7, r12} をスタックへ積んでから呼ぶ (AAPCS の第 5 引数以降はスタック渡し)。
//
//  ★第 5..8 引数はスタック上の [sp+0/4/8/12] から取られる。push は番号の小さい
//    レジスタが低いアドレスへ行くので、届くのは **r4, r5, r6, r7** であって r12 では
//    ない (r12 は [sp+16] にあるが第 9 引数に当たるので誰も読まない)。以前は第 8
//    引数を r12 と名乗っていたが、実体は r7 だった — ネスト数を r7 で渡すようになって
//    実害が出るので名前を実体に合わせた。
//
//  r7 (ネスト数) は METHOD_EXIT の arg3 にそのまま載せ返すこと。カーネルが実際の
//  深さと突き合わせ、食い違えば 1 段も落とさずエラーを返す (両側チェック)。
//  普通に return する場合は、カーネルが lr に載せた exit_method が同じことをやる。
//
//  ★このラッパは「委譲ハンドラの ABI そのもの」。共有ヘッダに置いてあるのは、
//    サブシステムごとに自作させると必ず間違えるため (実際に一度やらかした)。
//  ハンドラは普通に return してよい (カーネルが lr に exit_method を載せている)。
//  ハンドラ活性化は in_handler=1 なので、生のカーネルプリミティブを直接叩ける。
template <auto T>
  requires std::invocable<decltype(T), uint32_t, uint32_t, uint32_t, uint32_t,
                          uint32_t, uint32_t, uint32_t, uint32_t>
__attribute__((naked, aligned(4))) void svc_handler_shim() {
  asm volatile("push {r4-r7,r12, lr}\n"
               "ldr  r4, 1f\n"
               "blx  r4\n"
               "pop  {r4-r7,r12, pc}\n"
               ".align 2\n"
               "1: .word %c0\n"
               :
               : "i"(reinterpret_cast<uint32_t>(T))
               :);
}
#endif // SHIZU_SVC_HANDLER_HPP
