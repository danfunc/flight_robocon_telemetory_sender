.syntax unified
.cpu cortex-m33
#if defined(__ARM_FP)
.fpu fpv5-sp-d16   // .cpu 指定は FPU を含まないため、vstmia/vldmia 用に明示する
#endif
.thumb

// context_t のオフセット。kernel.cpp の static_assert と一致させること。
//   0..28 : r4-r11
//   32    : sp (PSP)
//   36    : exc_return (キャッシュした EXC_RETURN)
//   40    : fp[16] (S16-S31 退避域)
.equ CTX_EXC_RETURN, 36
.equ CTX_FP,         40

// ---------------------------------------------------------------------------
// CTX_SAVE — 現在スレッドの context_t へ {r4-r11, PSP, EXC_RETURN, 遅延 S16-S31}
// を退避する。入口条件: lr = EXC_RETURN (例外入口直後)。出口: r0 = context 先頭。
// FP ブロック以外は Cortex-M0+ 互換のエンコーディングに留め、FP 固有部のみ
// __ARM_FP でガードする (armv8-m / FPv5 でのみ有効)。
// ---------------------------------------------------------------------------
.macro CTX_SAVE
    push {lr}                     // EXC_RETURN を退避 (直後の bl で lr が壊れる)
    bl   get_current_context      // r0 = 現在スレッドの context 先頭

    // 整数 callee-saved と PSP を退避
    stmia r0!,{r4-r7}             // ctx[0..3] = r4-r7 ; r0 += 16
    mov  r1,r8
    mov  r2,r9
    mov  r3,r10
    mov  r4,r11
    mrs  r5,PSP
    stmia r0!,{r1-r5}            // ctx[4..8] = r8,r9,r10,r11,PSP ; r0 = base+36
    add  r0, #-36                 // r0 を context 先頭へ巻き戻す

    pop  {r1}                     // r1 = 退避しておいた EXC_RETURN
    str  r1,[r0, #CTX_EXC_RETURN] // ctx->exc_return = EXC_RETURN

#if defined(__ARM_FP)
    // EXC_RETURN bit4 == 0 なら拡張(FP)フレーム。S16-S31 を退避する。
    // (この vstmia 自体が FP 命令なので、遅延スタッキングされていた呼び出し元
    //  フレームの push を完了させる。VLSTM は不要。)
    tst.w r1, #0x10
    bne  1f                       // bit4=1 → FP 無し。退避スキップ
    add.w r1, r0, #CTX_FP         // r1 = &ctx->fp[0]
    vstmia r1!, {s16-s31}
1:
#endif
.endm

// ---------------------------------------------------------------------------
// CTX_RESTORE — get_current_context を再取得し (cpp ハンドラが別スレッドへ
// 切替えたかもしれない)、context_t から復元して例外復帰する。
// ---------------------------------------------------------------------------
.macro CTX_RESTORE
    bl   get_current_context      // r0 = 復帰対象 context (別スレッドかもしれない)

    // 整数 callee-saved と PSP を復元
    ldmia r0!,{r4-r7}            // r4-r7 ; r0 += 16
    ldmia r0!,{r1-r3}           // r8,r9,r10 ; r0 += 12
    mov  r8,r1
    mov  r9,r2
    mov  r10,r3
    ldmia r0!,{r1-r2}           // r11,PSP ; r0 = base+36
    mov  r11,r1
    msr  PSP,r2

    ldr  r3,[r0]                 // r3 = ctx->exc_return (r0 は base+36)

#if defined(__ARM_FP)
    // 復帰対象が拡張(FP)フレームなら S16-S31 を復元する。
    tst.w r3, #0x10
    bne  2f                       // bit4=1 → FP 無し。復元スキップ
    add  r0, #4                   // r0 = base+40 = &ctx->fp[0]
    vldmia r0!, {s16-s31}
2:
#endif

    mov  lr, r3                   // LR = 復帰対象の EXC_RETURN
    bx   lr                       // 例外復帰
.endm

// ---------------------------------------------------------------------------
// SVC 入口。CTX_SAVE → svc_cpp_handler(context) → CTX_RESTORE。
// ---------------------------------------------------------------------------
.thumb_func
.global svc_asm_handler
.align 8
.type svc_asm_handler, %function
svc_asm_handler:
    CTX_SAVE
    bl   svc_cpp_handler          // svc_cpp_handler(context = r0)
    CTX_RESTORE

// ---------------------------------------------------------------------------
// PendSV 入口 (grant 期限の強制コンテキストスイッチ)。退避/復元は SVC と完全同型
// (遅延 FPU 退避含む)。pendsv_cpp_handler が grant スタックを 1 段 pop して
// current_thread_id を grantor へ戻す (無関係な発火はガードして no-op 復帰)。
// ---------------------------------------------------------------------------
.thumb_func
.global pendsv_asm_handler
.align 2
.type pendsv_asm_handler, %function
pendsv_asm_handler:
    CTX_SAVE
    bl   pendsv_cpp_handler       // pendsv_cpp_handler(context = r0)
    CTX_RESTORE
