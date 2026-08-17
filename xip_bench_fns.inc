// 自動生成 (tools/gen_xip_bench_fns.py)。手で編集しないこと。
// 512 個の関数を 128B 境界へ並べ、.text 上に 64KB の的を作る。
// XIP キャッシュ (16KB) より十分大きいので、ランダム順に呼べば
// ほぼ毎回ラインフィル = 命令フェッチのストールを測れる。

#define XIP_BENCH_NFN 512
#define XIP_BENCH_FN_ALIGN 128

static uint32_t __attribute__((noinline, aligned(128)))
xf_0(uint32_t x) { return (x ^ 0u) * 2654435761u + 1u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_1(uint32_t x) { return (x ^ 2654435761u) * 2654435761u + 2u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_2(uint32_t x) { return (x ^ 1013904226u) * 2654435761u + 3u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_3(uint32_t x) { return (x ^ 3668339987u) * 2654435761u + 4u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_4(uint32_t x) { return (x ^ 2027808452u) * 2654435761u + 5u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_5(uint32_t x) { return (x ^ 387276917u) * 2654435761u + 6u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_6(uint32_t x) { return (x ^ 3041712678u) * 2654435761u + 7u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_7(uint32_t x) { return (x ^ 1401181143u) * 2654435761u + 8u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_8(uint32_t x) { return (x ^ 4055616904u) * 2654435761u + 9u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_9(uint32_t x) { return (x ^ 2415085369u) * 2654435761u + 10u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_10(uint32_t x) { return (x ^ 774553834u) * 2654435761u + 11u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_11(uint32_t x) { return (x ^ 3428989595u) * 2654435761u + 12u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_12(uint32_t x) { return (x ^ 1788458060u) * 2654435761u + 13u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_13(uint32_t x) { return (x ^ 147926525u) * 2654435761u + 14u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_14(uint32_t x) { return (x ^ 2802362286u) * 2654435761u + 15u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_15(uint32_t x) { return (x ^ 1161830751u) * 2654435761u + 16u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_16(uint32_t x) { return (x ^ 3816266512u) * 2654435761u + 17u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_17(uint32_t x) { return (x ^ 2175734977u) * 2654435761u + 18u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_18(uint32_t x) { return (x ^ 535203442u) * 2654435761u + 19u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_19(uint32_t x) { return (x ^ 3189639203u) * 2654435761u + 20u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_20(uint32_t x) { return (x ^ 1549107668u) * 2654435761u + 21u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_21(uint32_t x) { return (x ^ 4203543429u) * 2654435761u + 22u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_22(uint32_t x) { return (x ^ 2563011894u) * 2654435761u + 23u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_23(uint32_t x) { return (x ^ 922480359u) * 2654435761u + 24u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_24(uint32_t x) { return (x ^ 3576916120u) * 2654435761u + 25u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_25(uint32_t x) { return (x ^ 1936384585u) * 2654435761u + 26u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_26(uint32_t x) { return (x ^ 295853050u) * 2654435761u + 27u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_27(uint32_t x) { return (x ^ 2950288811u) * 2654435761u + 28u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_28(uint32_t x) { return (x ^ 1309757276u) * 2654435761u + 29u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_29(uint32_t x) { return (x ^ 3964193037u) * 2654435761u + 30u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_30(uint32_t x) { return (x ^ 2323661502u) * 2654435761u + 31u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_31(uint32_t x) { return (x ^ 683129967u) * 2654435761u + 32u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_32(uint32_t x) { return (x ^ 3337565728u) * 2654435761u + 33u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_33(uint32_t x) { return (x ^ 1697034193u) * 2654435761u + 34u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_34(uint32_t x) { return (x ^ 56502658u) * 2654435761u + 35u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_35(uint32_t x) { return (x ^ 2710938419u) * 2654435761u + 36u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_36(uint32_t x) { return (x ^ 1070406884u) * 2654435761u + 37u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_37(uint32_t x) { return (x ^ 3724842645u) * 2654435761u + 38u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_38(uint32_t x) { return (x ^ 2084311110u) * 2654435761u + 39u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_39(uint32_t x) { return (x ^ 443779575u) * 2654435761u + 40u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_40(uint32_t x) { return (x ^ 3098215336u) * 2654435761u + 41u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_41(uint32_t x) { return (x ^ 1457683801u) * 2654435761u + 42u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_42(uint32_t x) { return (x ^ 4112119562u) * 2654435761u + 43u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_43(uint32_t x) { return (x ^ 2471588027u) * 2654435761u + 44u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_44(uint32_t x) { return (x ^ 831056492u) * 2654435761u + 45u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_45(uint32_t x) { return (x ^ 3485492253u) * 2654435761u + 46u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_46(uint32_t x) { return (x ^ 1844960718u) * 2654435761u + 47u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_47(uint32_t x) { return (x ^ 204429183u) * 2654435761u + 48u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_48(uint32_t x) { return (x ^ 2858864944u) * 2654435761u + 49u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_49(uint32_t x) { return (x ^ 1218333409u) * 2654435761u + 50u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_50(uint32_t x) { return (x ^ 3872769170u) * 2654435761u + 51u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_51(uint32_t x) { return (x ^ 2232237635u) * 2654435761u + 52u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_52(uint32_t x) { return (x ^ 591706100u) * 2654435761u + 53u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_53(uint32_t x) { return (x ^ 3246141861u) * 2654435761u + 54u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_54(uint32_t x) { return (x ^ 1605610326u) * 2654435761u + 55u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_55(uint32_t x) { return (x ^ 4260046087u) * 2654435761u + 56u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_56(uint32_t x) { return (x ^ 2619514552u) * 2654435761u + 57u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_57(uint32_t x) { return (x ^ 978983017u) * 2654435761u + 58u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_58(uint32_t x) { return (x ^ 3633418778u) * 2654435761u + 59u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_59(uint32_t x) { return (x ^ 1992887243u) * 2654435761u + 60u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_60(uint32_t x) { return (x ^ 352355708u) * 2654435761u + 61u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_61(uint32_t x) { return (x ^ 3006791469u) * 2654435761u + 62u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_62(uint32_t x) { return (x ^ 1366259934u) * 2654435761u + 63u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_63(uint32_t x) { return (x ^ 4020695695u) * 2654435761u + 64u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_64(uint32_t x) { return (x ^ 2380164160u) * 2654435761u + 65u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_65(uint32_t x) { return (x ^ 739632625u) * 2654435761u + 66u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_66(uint32_t x) { return (x ^ 3394068386u) * 2654435761u + 67u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_67(uint32_t x) { return (x ^ 1753536851u) * 2654435761u + 68u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_68(uint32_t x) { return (x ^ 113005316u) * 2654435761u + 69u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_69(uint32_t x) { return (x ^ 2767441077u) * 2654435761u + 70u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_70(uint32_t x) { return (x ^ 1126909542u) * 2654435761u + 71u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_71(uint32_t x) { return (x ^ 3781345303u) * 2654435761u + 72u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_72(uint32_t x) { return (x ^ 2140813768u) * 2654435761u + 73u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_73(uint32_t x) { return (x ^ 500282233u) * 2654435761u + 74u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_74(uint32_t x) { return (x ^ 3154717994u) * 2654435761u + 75u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_75(uint32_t x) { return (x ^ 1514186459u) * 2654435761u + 76u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_76(uint32_t x) { return (x ^ 4168622220u) * 2654435761u + 77u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_77(uint32_t x) { return (x ^ 2528090685u) * 2654435761u + 78u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_78(uint32_t x) { return (x ^ 887559150u) * 2654435761u + 79u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_79(uint32_t x) { return (x ^ 3541994911u) * 2654435761u + 80u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_80(uint32_t x) { return (x ^ 1901463376u) * 2654435761u + 81u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_81(uint32_t x) { return (x ^ 260931841u) * 2654435761u + 82u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_82(uint32_t x) { return (x ^ 2915367602u) * 2654435761u + 83u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_83(uint32_t x) { return (x ^ 1274836067u) * 2654435761u + 84u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_84(uint32_t x) { return (x ^ 3929271828u) * 2654435761u + 85u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_85(uint32_t x) { return (x ^ 2288740293u) * 2654435761u + 86u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_86(uint32_t x) { return (x ^ 648208758u) * 2654435761u + 87u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_87(uint32_t x) { return (x ^ 3302644519u) * 2654435761u + 88u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_88(uint32_t x) { return (x ^ 1662112984u) * 2654435761u + 89u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_89(uint32_t x) { return (x ^ 21581449u) * 2654435761u + 90u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_90(uint32_t x) { return (x ^ 2676017210u) * 2654435761u + 91u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_91(uint32_t x) { return (x ^ 1035485675u) * 2654435761u + 92u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_92(uint32_t x) { return (x ^ 3689921436u) * 2654435761u + 93u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_93(uint32_t x) { return (x ^ 2049389901u) * 2654435761u + 94u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_94(uint32_t x) { return (x ^ 408858366u) * 2654435761u + 95u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_95(uint32_t x) { return (x ^ 3063294127u) * 2654435761u + 96u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_96(uint32_t x) { return (x ^ 1422762592u) * 2654435761u + 97u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_97(uint32_t x) { return (x ^ 4077198353u) * 2654435761u + 98u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_98(uint32_t x) { return (x ^ 2436666818u) * 2654435761u + 99u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_99(uint32_t x) { return (x ^ 796135283u) * 2654435761u + 100u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_100(uint32_t x) { return (x ^ 3450571044u) * 2654435761u + 101u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_101(uint32_t x) { return (x ^ 1810039509u) * 2654435761u + 102u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_102(uint32_t x) { return (x ^ 169507974u) * 2654435761u + 103u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_103(uint32_t x) { return (x ^ 2823943735u) * 2654435761u + 104u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_104(uint32_t x) { return (x ^ 1183412200u) * 2654435761u + 105u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_105(uint32_t x) { return (x ^ 3837847961u) * 2654435761u + 106u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_106(uint32_t x) { return (x ^ 2197316426u) * 2654435761u + 107u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_107(uint32_t x) { return (x ^ 556784891u) * 2654435761u + 108u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_108(uint32_t x) { return (x ^ 3211220652u) * 2654435761u + 109u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_109(uint32_t x) { return (x ^ 1570689117u) * 2654435761u + 110u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_110(uint32_t x) { return (x ^ 4225124878u) * 2654435761u + 111u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_111(uint32_t x) { return (x ^ 2584593343u) * 2654435761u + 112u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_112(uint32_t x) { return (x ^ 944061808u) * 2654435761u + 113u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_113(uint32_t x) { return (x ^ 3598497569u) * 2654435761u + 114u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_114(uint32_t x) { return (x ^ 1957966034u) * 2654435761u + 115u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_115(uint32_t x) { return (x ^ 317434499u) * 2654435761u + 116u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_116(uint32_t x) { return (x ^ 2971870260u) * 2654435761u + 117u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_117(uint32_t x) { return (x ^ 1331338725u) * 2654435761u + 118u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_118(uint32_t x) { return (x ^ 3985774486u) * 2654435761u + 119u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_119(uint32_t x) { return (x ^ 2345242951u) * 2654435761u + 120u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_120(uint32_t x) { return (x ^ 704711416u) * 2654435761u + 121u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_121(uint32_t x) { return (x ^ 3359147177u) * 2654435761u + 122u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_122(uint32_t x) { return (x ^ 1718615642u) * 2654435761u + 123u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_123(uint32_t x) { return (x ^ 78084107u) * 2654435761u + 124u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_124(uint32_t x) { return (x ^ 2732519868u) * 2654435761u + 125u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_125(uint32_t x) { return (x ^ 1091988333u) * 2654435761u + 126u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_126(uint32_t x) { return (x ^ 3746424094u) * 2654435761u + 127u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_127(uint32_t x) { return (x ^ 2105892559u) * 2654435761u + 128u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_128(uint32_t x) { return (x ^ 465361024u) * 2654435761u + 129u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_129(uint32_t x) { return (x ^ 3119796785u) * 2654435761u + 130u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_130(uint32_t x) { return (x ^ 1479265250u) * 2654435761u + 131u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_131(uint32_t x) { return (x ^ 4133701011u) * 2654435761u + 132u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_132(uint32_t x) { return (x ^ 2493169476u) * 2654435761u + 133u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_133(uint32_t x) { return (x ^ 852637941u) * 2654435761u + 134u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_134(uint32_t x) { return (x ^ 3507073702u) * 2654435761u + 135u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_135(uint32_t x) { return (x ^ 1866542167u) * 2654435761u + 136u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_136(uint32_t x) { return (x ^ 226010632u) * 2654435761u + 137u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_137(uint32_t x) { return (x ^ 2880446393u) * 2654435761u + 138u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_138(uint32_t x) { return (x ^ 1239914858u) * 2654435761u + 139u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_139(uint32_t x) { return (x ^ 3894350619u) * 2654435761u + 140u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_140(uint32_t x) { return (x ^ 2253819084u) * 2654435761u + 141u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_141(uint32_t x) { return (x ^ 613287549u) * 2654435761u + 142u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_142(uint32_t x) { return (x ^ 3267723310u) * 2654435761u + 143u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_143(uint32_t x) { return (x ^ 1627191775u) * 2654435761u + 144u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_144(uint32_t x) { return (x ^ 4281627536u) * 2654435761u + 145u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_145(uint32_t x) { return (x ^ 2641096001u) * 2654435761u + 146u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_146(uint32_t x) { return (x ^ 1000564466u) * 2654435761u + 147u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_147(uint32_t x) { return (x ^ 3655000227u) * 2654435761u + 148u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_148(uint32_t x) { return (x ^ 2014468692u) * 2654435761u + 149u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_149(uint32_t x) { return (x ^ 373937157u) * 2654435761u + 150u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_150(uint32_t x) { return (x ^ 3028372918u) * 2654435761u + 151u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_151(uint32_t x) { return (x ^ 1387841383u) * 2654435761u + 152u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_152(uint32_t x) { return (x ^ 4042277144u) * 2654435761u + 153u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_153(uint32_t x) { return (x ^ 2401745609u) * 2654435761u + 154u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_154(uint32_t x) { return (x ^ 761214074u) * 2654435761u + 155u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_155(uint32_t x) { return (x ^ 3415649835u) * 2654435761u + 156u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_156(uint32_t x) { return (x ^ 1775118300u) * 2654435761u + 157u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_157(uint32_t x) { return (x ^ 134586765u) * 2654435761u + 158u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_158(uint32_t x) { return (x ^ 2789022526u) * 2654435761u + 159u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_159(uint32_t x) { return (x ^ 1148490991u) * 2654435761u + 160u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_160(uint32_t x) { return (x ^ 3802926752u) * 2654435761u + 161u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_161(uint32_t x) { return (x ^ 2162395217u) * 2654435761u + 162u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_162(uint32_t x) { return (x ^ 521863682u) * 2654435761u + 163u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_163(uint32_t x) { return (x ^ 3176299443u) * 2654435761u + 164u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_164(uint32_t x) { return (x ^ 1535767908u) * 2654435761u + 165u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_165(uint32_t x) { return (x ^ 4190203669u) * 2654435761u + 166u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_166(uint32_t x) { return (x ^ 2549672134u) * 2654435761u + 167u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_167(uint32_t x) { return (x ^ 909140599u) * 2654435761u + 168u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_168(uint32_t x) { return (x ^ 3563576360u) * 2654435761u + 169u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_169(uint32_t x) { return (x ^ 1923044825u) * 2654435761u + 170u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_170(uint32_t x) { return (x ^ 282513290u) * 2654435761u + 171u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_171(uint32_t x) { return (x ^ 2936949051u) * 2654435761u + 172u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_172(uint32_t x) { return (x ^ 1296417516u) * 2654435761u + 173u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_173(uint32_t x) { return (x ^ 3950853277u) * 2654435761u + 174u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_174(uint32_t x) { return (x ^ 2310321742u) * 2654435761u + 175u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_175(uint32_t x) { return (x ^ 669790207u) * 2654435761u + 176u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_176(uint32_t x) { return (x ^ 3324225968u) * 2654435761u + 177u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_177(uint32_t x) { return (x ^ 1683694433u) * 2654435761u + 178u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_178(uint32_t x) { return (x ^ 43162898u) * 2654435761u + 179u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_179(uint32_t x) { return (x ^ 2697598659u) * 2654435761u + 180u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_180(uint32_t x) { return (x ^ 1057067124u) * 2654435761u + 181u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_181(uint32_t x) { return (x ^ 3711502885u) * 2654435761u + 182u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_182(uint32_t x) { return (x ^ 2070971350u) * 2654435761u + 183u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_183(uint32_t x) { return (x ^ 430439815u) * 2654435761u + 184u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_184(uint32_t x) { return (x ^ 3084875576u) * 2654435761u + 185u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_185(uint32_t x) { return (x ^ 1444344041u) * 2654435761u + 186u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_186(uint32_t x) { return (x ^ 4098779802u) * 2654435761u + 187u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_187(uint32_t x) { return (x ^ 2458248267u) * 2654435761u + 188u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_188(uint32_t x) { return (x ^ 817716732u) * 2654435761u + 189u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_189(uint32_t x) { return (x ^ 3472152493u) * 2654435761u + 190u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_190(uint32_t x) { return (x ^ 1831620958u) * 2654435761u + 191u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_191(uint32_t x) { return (x ^ 191089423u) * 2654435761u + 192u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_192(uint32_t x) { return (x ^ 2845525184u) * 2654435761u + 193u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_193(uint32_t x) { return (x ^ 1204993649u) * 2654435761u + 194u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_194(uint32_t x) { return (x ^ 3859429410u) * 2654435761u + 195u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_195(uint32_t x) { return (x ^ 2218897875u) * 2654435761u + 196u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_196(uint32_t x) { return (x ^ 578366340u) * 2654435761u + 197u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_197(uint32_t x) { return (x ^ 3232802101u) * 2654435761u + 198u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_198(uint32_t x) { return (x ^ 1592270566u) * 2654435761u + 199u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_199(uint32_t x) { return (x ^ 4246706327u) * 2654435761u + 200u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_200(uint32_t x) { return (x ^ 2606174792u) * 2654435761u + 201u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_201(uint32_t x) { return (x ^ 965643257u) * 2654435761u + 202u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_202(uint32_t x) { return (x ^ 3620079018u) * 2654435761u + 203u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_203(uint32_t x) { return (x ^ 1979547483u) * 2654435761u + 204u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_204(uint32_t x) { return (x ^ 339015948u) * 2654435761u + 205u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_205(uint32_t x) { return (x ^ 2993451709u) * 2654435761u + 206u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_206(uint32_t x) { return (x ^ 1352920174u) * 2654435761u + 207u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_207(uint32_t x) { return (x ^ 4007355935u) * 2654435761u + 208u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_208(uint32_t x) { return (x ^ 2366824400u) * 2654435761u + 209u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_209(uint32_t x) { return (x ^ 726292865u) * 2654435761u + 210u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_210(uint32_t x) { return (x ^ 3380728626u) * 2654435761u + 211u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_211(uint32_t x) { return (x ^ 1740197091u) * 2654435761u + 212u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_212(uint32_t x) { return (x ^ 99665556u) * 2654435761u + 213u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_213(uint32_t x) { return (x ^ 2754101317u) * 2654435761u + 214u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_214(uint32_t x) { return (x ^ 1113569782u) * 2654435761u + 215u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_215(uint32_t x) { return (x ^ 3768005543u) * 2654435761u + 216u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_216(uint32_t x) { return (x ^ 2127474008u) * 2654435761u + 217u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_217(uint32_t x) { return (x ^ 486942473u) * 2654435761u + 218u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_218(uint32_t x) { return (x ^ 3141378234u) * 2654435761u + 219u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_219(uint32_t x) { return (x ^ 1500846699u) * 2654435761u + 220u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_220(uint32_t x) { return (x ^ 4155282460u) * 2654435761u + 221u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_221(uint32_t x) { return (x ^ 2514750925u) * 2654435761u + 222u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_222(uint32_t x) { return (x ^ 874219390u) * 2654435761u + 223u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_223(uint32_t x) { return (x ^ 3528655151u) * 2654435761u + 224u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_224(uint32_t x) { return (x ^ 1888123616u) * 2654435761u + 225u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_225(uint32_t x) { return (x ^ 247592081u) * 2654435761u + 226u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_226(uint32_t x) { return (x ^ 2902027842u) * 2654435761u + 227u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_227(uint32_t x) { return (x ^ 1261496307u) * 2654435761u + 228u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_228(uint32_t x) { return (x ^ 3915932068u) * 2654435761u + 229u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_229(uint32_t x) { return (x ^ 2275400533u) * 2654435761u + 230u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_230(uint32_t x) { return (x ^ 634868998u) * 2654435761u + 231u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_231(uint32_t x) { return (x ^ 3289304759u) * 2654435761u + 232u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_232(uint32_t x) { return (x ^ 1648773224u) * 2654435761u + 233u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_233(uint32_t x) { return (x ^ 8241689u) * 2654435761u + 234u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_234(uint32_t x) { return (x ^ 2662677450u) * 2654435761u + 235u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_235(uint32_t x) { return (x ^ 1022145915u) * 2654435761u + 236u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_236(uint32_t x) { return (x ^ 3676581676u) * 2654435761u + 237u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_237(uint32_t x) { return (x ^ 2036050141u) * 2654435761u + 238u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_238(uint32_t x) { return (x ^ 395518606u) * 2654435761u + 239u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_239(uint32_t x) { return (x ^ 3049954367u) * 2654435761u + 240u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_240(uint32_t x) { return (x ^ 1409422832u) * 2654435761u + 241u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_241(uint32_t x) { return (x ^ 4063858593u) * 2654435761u + 242u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_242(uint32_t x) { return (x ^ 2423327058u) * 2654435761u + 243u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_243(uint32_t x) { return (x ^ 782795523u) * 2654435761u + 244u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_244(uint32_t x) { return (x ^ 3437231284u) * 2654435761u + 245u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_245(uint32_t x) { return (x ^ 1796699749u) * 2654435761u + 246u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_246(uint32_t x) { return (x ^ 156168214u) * 2654435761u + 247u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_247(uint32_t x) { return (x ^ 2810603975u) * 2654435761u + 248u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_248(uint32_t x) { return (x ^ 1170072440u) * 2654435761u + 249u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_249(uint32_t x) { return (x ^ 3824508201u) * 2654435761u + 250u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_250(uint32_t x) { return (x ^ 2183976666u) * 2654435761u + 251u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_251(uint32_t x) { return (x ^ 543445131u) * 2654435761u + 252u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_252(uint32_t x) { return (x ^ 3197880892u) * 2654435761u + 253u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_253(uint32_t x) { return (x ^ 1557349357u) * 2654435761u + 254u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_254(uint32_t x) { return (x ^ 4211785118u) * 2654435761u + 255u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_255(uint32_t x) { return (x ^ 2571253583u) * 2654435761u + 256u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_256(uint32_t x) { return (x ^ 930722048u) * 2654435761u + 257u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_257(uint32_t x) { return (x ^ 3585157809u) * 2654435761u + 258u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_258(uint32_t x) { return (x ^ 1944626274u) * 2654435761u + 259u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_259(uint32_t x) { return (x ^ 304094739u) * 2654435761u + 260u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_260(uint32_t x) { return (x ^ 2958530500u) * 2654435761u + 261u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_261(uint32_t x) { return (x ^ 1317998965u) * 2654435761u + 262u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_262(uint32_t x) { return (x ^ 3972434726u) * 2654435761u + 263u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_263(uint32_t x) { return (x ^ 2331903191u) * 2654435761u + 264u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_264(uint32_t x) { return (x ^ 691371656u) * 2654435761u + 265u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_265(uint32_t x) { return (x ^ 3345807417u) * 2654435761u + 266u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_266(uint32_t x) { return (x ^ 1705275882u) * 2654435761u + 267u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_267(uint32_t x) { return (x ^ 64744347u) * 2654435761u + 268u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_268(uint32_t x) { return (x ^ 2719180108u) * 2654435761u + 269u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_269(uint32_t x) { return (x ^ 1078648573u) * 2654435761u + 270u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_270(uint32_t x) { return (x ^ 3733084334u) * 2654435761u + 271u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_271(uint32_t x) { return (x ^ 2092552799u) * 2654435761u + 272u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_272(uint32_t x) { return (x ^ 452021264u) * 2654435761u + 273u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_273(uint32_t x) { return (x ^ 3106457025u) * 2654435761u + 274u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_274(uint32_t x) { return (x ^ 1465925490u) * 2654435761u + 275u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_275(uint32_t x) { return (x ^ 4120361251u) * 2654435761u + 276u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_276(uint32_t x) { return (x ^ 2479829716u) * 2654435761u + 277u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_277(uint32_t x) { return (x ^ 839298181u) * 2654435761u + 278u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_278(uint32_t x) { return (x ^ 3493733942u) * 2654435761u + 279u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_279(uint32_t x) { return (x ^ 1853202407u) * 2654435761u + 280u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_280(uint32_t x) { return (x ^ 212670872u) * 2654435761u + 281u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_281(uint32_t x) { return (x ^ 2867106633u) * 2654435761u + 282u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_282(uint32_t x) { return (x ^ 1226575098u) * 2654435761u + 283u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_283(uint32_t x) { return (x ^ 3881010859u) * 2654435761u + 284u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_284(uint32_t x) { return (x ^ 2240479324u) * 2654435761u + 285u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_285(uint32_t x) { return (x ^ 599947789u) * 2654435761u + 286u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_286(uint32_t x) { return (x ^ 3254383550u) * 2654435761u + 287u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_287(uint32_t x) { return (x ^ 1613852015u) * 2654435761u + 288u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_288(uint32_t x) { return (x ^ 4268287776u) * 2654435761u + 289u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_289(uint32_t x) { return (x ^ 2627756241u) * 2654435761u + 290u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_290(uint32_t x) { return (x ^ 987224706u) * 2654435761u + 291u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_291(uint32_t x) { return (x ^ 3641660467u) * 2654435761u + 292u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_292(uint32_t x) { return (x ^ 2001128932u) * 2654435761u + 293u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_293(uint32_t x) { return (x ^ 360597397u) * 2654435761u + 294u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_294(uint32_t x) { return (x ^ 3015033158u) * 2654435761u + 295u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_295(uint32_t x) { return (x ^ 1374501623u) * 2654435761u + 296u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_296(uint32_t x) { return (x ^ 4028937384u) * 2654435761u + 297u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_297(uint32_t x) { return (x ^ 2388405849u) * 2654435761u + 298u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_298(uint32_t x) { return (x ^ 747874314u) * 2654435761u + 299u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_299(uint32_t x) { return (x ^ 3402310075u) * 2654435761u + 300u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_300(uint32_t x) { return (x ^ 1761778540u) * 2654435761u + 301u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_301(uint32_t x) { return (x ^ 121247005u) * 2654435761u + 302u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_302(uint32_t x) { return (x ^ 2775682766u) * 2654435761u + 303u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_303(uint32_t x) { return (x ^ 1135151231u) * 2654435761u + 304u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_304(uint32_t x) { return (x ^ 3789586992u) * 2654435761u + 305u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_305(uint32_t x) { return (x ^ 2149055457u) * 2654435761u + 306u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_306(uint32_t x) { return (x ^ 508523922u) * 2654435761u + 307u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_307(uint32_t x) { return (x ^ 3162959683u) * 2654435761u + 308u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_308(uint32_t x) { return (x ^ 1522428148u) * 2654435761u + 309u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_309(uint32_t x) { return (x ^ 4176863909u) * 2654435761u + 310u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_310(uint32_t x) { return (x ^ 2536332374u) * 2654435761u + 311u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_311(uint32_t x) { return (x ^ 895800839u) * 2654435761u + 312u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_312(uint32_t x) { return (x ^ 3550236600u) * 2654435761u + 313u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_313(uint32_t x) { return (x ^ 1909705065u) * 2654435761u + 314u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_314(uint32_t x) { return (x ^ 269173530u) * 2654435761u + 315u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_315(uint32_t x) { return (x ^ 2923609291u) * 2654435761u + 316u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_316(uint32_t x) { return (x ^ 1283077756u) * 2654435761u + 317u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_317(uint32_t x) { return (x ^ 3937513517u) * 2654435761u + 318u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_318(uint32_t x) { return (x ^ 2296981982u) * 2654435761u + 319u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_319(uint32_t x) { return (x ^ 656450447u) * 2654435761u + 320u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_320(uint32_t x) { return (x ^ 3310886208u) * 2654435761u + 321u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_321(uint32_t x) { return (x ^ 1670354673u) * 2654435761u + 322u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_322(uint32_t x) { return (x ^ 29823138u) * 2654435761u + 323u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_323(uint32_t x) { return (x ^ 2684258899u) * 2654435761u + 324u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_324(uint32_t x) { return (x ^ 1043727364u) * 2654435761u + 325u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_325(uint32_t x) { return (x ^ 3698163125u) * 2654435761u + 326u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_326(uint32_t x) { return (x ^ 2057631590u) * 2654435761u + 327u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_327(uint32_t x) { return (x ^ 417100055u) * 2654435761u + 328u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_328(uint32_t x) { return (x ^ 3071535816u) * 2654435761u + 329u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_329(uint32_t x) { return (x ^ 1431004281u) * 2654435761u + 330u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_330(uint32_t x) { return (x ^ 4085440042u) * 2654435761u + 331u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_331(uint32_t x) { return (x ^ 2444908507u) * 2654435761u + 332u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_332(uint32_t x) { return (x ^ 804376972u) * 2654435761u + 333u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_333(uint32_t x) { return (x ^ 3458812733u) * 2654435761u + 334u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_334(uint32_t x) { return (x ^ 1818281198u) * 2654435761u + 335u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_335(uint32_t x) { return (x ^ 177749663u) * 2654435761u + 336u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_336(uint32_t x) { return (x ^ 2832185424u) * 2654435761u + 337u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_337(uint32_t x) { return (x ^ 1191653889u) * 2654435761u + 338u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_338(uint32_t x) { return (x ^ 3846089650u) * 2654435761u + 339u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_339(uint32_t x) { return (x ^ 2205558115u) * 2654435761u + 340u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_340(uint32_t x) { return (x ^ 565026580u) * 2654435761u + 341u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_341(uint32_t x) { return (x ^ 3219462341u) * 2654435761u + 342u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_342(uint32_t x) { return (x ^ 1578930806u) * 2654435761u + 343u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_343(uint32_t x) { return (x ^ 4233366567u) * 2654435761u + 344u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_344(uint32_t x) { return (x ^ 2592835032u) * 2654435761u + 345u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_345(uint32_t x) { return (x ^ 952303497u) * 2654435761u + 346u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_346(uint32_t x) { return (x ^ 3606739258u) * 2654435761u + 347u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_347(uint32_t x) { return (x ^ 1966207723u) * 2654435761u + 348u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_348(uint32_t x) { return (x ^ 325676188u) * 2654435761u + 349u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_349(uint32_t x) { return (x ^ 2980111949u) * 2654435761u + 350u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_350(uint32_t x) { return (x ^ 1339580414u) * 2654435761u + 351u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_351(uint32_t x) { return (x ^ 3994016175u) * 2654435761u + 352u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_352(uint32_t x) { return (x ^ 2353484640u) * 2654435761u + 353u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_353(uint32_t x) { return (x ^ 712953105u) * 2654435761u + 354u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_354(uint32_t x) { return (x ^ 3367388866u) * 2654435761u + 355u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_355(uint32_t x) { return (x ^ 1726857331u) * 2654435761u + 356u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_356(uint32_t x) { return (x ^ 86325796u) * 2654435761u + 357u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_357(uint32_t x) { return (x ^ 2740761557u) * 2654435761u + 358u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_358(uint32_t x) { return (x ^ 1100230022u) * 2654435761u + 359u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_359(uint32_t x) { return (x ^ 3754665783u) * 2654435761u + 360u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_360(uint32_t x) { return (x ^ 2114134248u) * 2654435761u + 361u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_361(uint32_t x) { return (x ^ 473602713u) * 2654435761u + 362u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_362(uint32_t x) { return (x ^ 3128038474u) * 2654435761u + 363u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_363(uint32_t x) { return (x ^ 1487506939u) * 2654435761u + 364u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_364(uint32_t x) { return (x ^ 4141942700u) * 2654435761u + 365u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_365(uint32_t x) { return (x ^ 2501411165u) * 2654435761u + 366u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_366(uint32_t x) { return (x ^ 860879630u) * 2654435761u + 367u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_367(uint32_t x) { return (x ^ 3515315391u) * 2654435761u + 368u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_368(uint32_t x) { return (x ^ 1874783856u) * 2654435761u + 369u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_369(uint32_t x) { return (x ^ 234252321u) * 2654435761u + 370u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_370(uint32_t x) { return (x ^ 2888688082u) * 2654435761u + 371u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_371(uint32_t x) { return (x ^ 1248156547u) * 2654435761u + 372u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_372(uint32_t x) { return (x ^ 3902592308u) * 2654435761u + 373u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_373(uint32_t x) { return (x ^ 2262060773u) * 2654435761u + 374u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_374(uint32_t x) { return (x ^ 621529238u) * 2654435761u + 375u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_375(uint32_t x) { return (x ^ 3275964999u) * 2654435761u + 376u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_376(uint32_t x) { return (x ^ 1635433464u) * 2654435761u + 377u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_377(uint32_t x) { return (x ^ 4289869225u) * 2654435761u + 378u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_378(uint32_t x) { return (x ^ 2649337690u) * 2654435761u + 379u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_379(uint32_t x) { return (x ^ 1008806155u) * 2654435761u + 380u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_380(uint32_t x) { return (x ^ 3663241916u) * 2654435761u + 381u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_381(uint32_t x) { return (x ^ 2022710381u) * 2654435761u + 382u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_382(uint32_t x) { return (x ^ 382178846u) * 2654435761u + 383u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_383(uint32_t x) { return (x ^ 3036614607u) * 2654435761u + 384u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_384(uint32_t x) { return (x ^ 1396083072u) * 2654435761u + 385u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_385(uint32_t x) { return (x ^ 4050518833u) * 2654435761u + 386u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_386(uint32_t x) { return (x ^ 2409987298u) * 2654435761u + 387u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_387(uint32_t x) { return (x ^ 769455763u) * 2654435761u + 388u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_388(uint32_t x) { return (x ^ 3423891524u) * 2654435761u + 389u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_389(uint32_t x) { return (x ^ 1783359989u) * 2654435761u + 390u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_390(uint32_t x) { return (x ^ 142828454u) * 2654435761u + 391u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_391(uint32_t x) { return (x ^ 2797264215u) * 2654435761u + 392u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_392(uint32_t x) { return (x ^ 1156732680u) * 2654435761u + 393u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_393(uint32_t x) { return (x ^ 3811168441u) * 2654435761u + 394u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_394(uint32_t x) { return (x ^ 2170636906u) * 2654435761u + 395u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_395(uint32_t x) { return (x ^ 530105371u) * 2654435761u + 396u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_396(uint32_t x) { return (x ^ 3184541132u) * 2654435761u + 397u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_397(uint32_t x) { return (x ^ 1544009597u) * 2654435761u + 398u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_398(uint32_t x) { return (x ^ 4198445358u) * 2654435761u + 399u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_399(uint32_t x) { return (x ^ 2557913823u) * 2654435761u + 400u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_400(uint32_t x) { return (x ^ 917382288u) * 2654435761u + 401u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_401(uint32_t x) { return (x ^ 3571818049u) * 2654435761u + 402u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_402(uint32_t x) { return (x ^ 1931286514u) * 2654435761u + 403u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_403(uint32_t x) { return (x ^ 290754979u) * 2654435761u + 404u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_404(uint32_t x) { return (x ^ 2945190740u) * 2654435761u + 405u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_405(uint32_t x) { return (x ^ 1304659205u) * 2654435761u + 406u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_406(uint32_t x) { return (x ^ 3959094966u) * 2654435761u + 407u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_407(uint32_t x) { return (x ^ 2318563431u) * 2654435761u + 408u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_408(uint32_t x) { return (x ^ 678031896u) * 2654435761u + 409u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_409(uint32_t x) { return (x ^ 3332467657u) * 2654435761u + 410u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_410(uint32_t x) { return (x ^ 1691936122u) * 2654435761u + 411u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_411(uint32_t x) { return (x ^ 51404587u) * 2654435761u + 412u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_412(uint32_t x) { return (x ^ 2705840348u) * 2654435761u + 413u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_413(uint32_t x) { return (x ^ 1065308813u) * 2654435761u + 414u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_414(uint32_t x) { return (x ^ 3719744574u) * 2654435761u + 415u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_415(uint32_t x) { return (x ^ 2079213039u) * 2654435761u + 416u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_416(uint32_t x) { return (x ^ 438681504u) * 2654435761u + 417u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_417(uint32_t x) { return (x ^ 3093117265u) * 2654435761u + 418u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_418(uint32_t x) { return (x ^ 1452585730u) * 2654435761u + 419u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_419(uint32_t x) { return (x ^ 4107021491u) * 2654435761u + 420u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_420(uint32_t x) { return (x ^ 2466489956u) * 2654435761u + 421u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_421(uint32_t x) { return (x ^ 825958421u) * 2654435761u + 422u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_422(uint32_t x) { return (x ^ 3480394182u) * 2654435761u + 423u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_423(uint32_t x) { return (x ^ 1839862647u) * 2654435761u + 424u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_424(uint32_t x) { return (x ^ 199331112u) * 2654435761u + 425u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_425(uint32_t x) { return (x ^ 2853766873u) * 2654435761u + 426u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_426(uint32_t x) { return (x ^ 1213235338u) * 2654435761u + 427u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_427(uint32_t x) { return (x ^ 3867671099u) * 2654435761u + 428u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_428(uint32_t x) { return (x ^ 2227139564u) * 2654435761u + 429u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_429(uint32_t x) { return (x ^ 586608029u) * 2654435761u + 430u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_430(uint32_t x) { return (x ^ 3241043790u) * 2654435761u + 431u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_431(uint32_t x) { return (x ^ 1600512255u) * 2654435761u + 432u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_432(uint32_t x) { return (x ^ 4254948016u) * 2654435761u + 433u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_433(uint32_t x) { return (x ^ 2614416481u) * 2654435761u + 434u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_434(uint32_t x) { return (x ^ 973884946u) * 2654435761u + 435u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_435(uint32_t x) { return (x ^ 3628320707u) * 2654435761u + 436u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_436(uint32_t x) { return (x ^ 1987789172u) * 2654435761u + 437u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_437(uint32_t x) { return (x ^ 347257637u) * 2654435761u + 438u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_438(uint32_t x) { return (x ^ 3001693398u) * 2654435761u + 439u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_439(uint32_t x) { return (x ^ 1361161863u) * 2654435761u + 440u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_440(uint32_t x) { return (x ^ 4015597624u) * 2654435761u + 441u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_441(uint32_t x) { return (x ^ 2375066089u) * 2654435761u + 442u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_442(uint32_t x) { return (x ^ 734534554u) * 2654435761u + 443u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_443(uint32_t x) { return (x ^ 3388970315u) * 2654435761u + 444u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_444(uint32_t x) { return (x ^ 1748438780u) * 2654435761u + 445u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_445(uint32_t x) { return (x ^ 107907245u) * 2654435761u + 446u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_446(uint32_t x) { return (x ^ 2762343006u) * 2654435761u + 447u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_447(uint32_t x) { return (x ^ 1121811471u) * 2654435761u + 448u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_448(uint32_t x) { return (x ^ 3776247232u) * 2654435761u + 449u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_449(uint32_t x) { return (x ^ 2135715697u) * 2654435761u + 450u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_450(uint32_t x) { return (x ^ 495184162u) * 2654435761u + 451u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_451(uint32_t x) { return (x ^ 3149619923u) * 2654435761u + 452u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_452(uint32_t x) { return (x ^ 1509088388u) * 2654435761u + 453u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_453(uint32_t x) { return (x ^ 4163524149u) * 2654435761u + 454u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_454(uint32_t x) { return (x ^ 2522992614u) * 2654435761u + 455u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_455(uint32_t x) { return (x ^ 882461079u) * 2654435761u + 456u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_456(uint32_t x) { return (x ^ 3536896840u) * 2654435761u + 457u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_457(uint32_t x) { return (x ^ 1896365305u) * 2654435761u + 458u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_458(uint32_t x) { return (x ^ 255833770u) * 2654435761u + 459u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_459(uint32_t x) { return (x ^ 2910269531u) * 2654435761u + 460u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_460(uint32_t x) { return (x ^ 1269737996u) * 2654435761u + 461u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_461(uint32_t x) { return (x ^ 3924173757u) * 2654435761u + 462u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_462(uint32_t x) { return (x ^ 2283642222u) * 2654435761u + 463u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_463(uint32_t x) { return (x ^ 643110687u) * 2654435761u + 464u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_464(uint32_t x) { return (x ^ 3297546448u) * 2654435761u + 465u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_465(uint32_t x) { return (x ^ 1657014913u) * 2654435761u + 466u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_466(uint32_t x) { return (x ^ 16483378u) * 2654435761u + 467u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_467(uint32_t x) { return (x ^ 2670919139u) * 2654435761u + 468u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_468(uint32_t x) { return (x ^ 1030387604u) * 2654435761u + 469u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_469(uint32_t x) { return (x ^ 3684823365u) * 2654435761u + 470u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_470(uint32_t x) { return (x ^ 2044291830u) * 2654435761u + 471u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_471(uint32_t x) { return (x ^ 403760295u) * 2654435761u + 472u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_472(uint32_t x) { return (x ^ 3058196056u) * 2654435761u + 473u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_473(uint32_t x) { return (x ^ 1417664521u) * 2654435761u + 474u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_474(uint32_t x) { return (x ^ 4072100282u) * 2654435761u + 475u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_475(uint32_t x) { return (x ^ 2431568747u) * 2654435761u + 476u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_476(uint32_t x) { return (x ^ 791037212u) * 2654435761u + 477u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_477(uint32_t x) { return (x ^ 3445472973u) * 2654435761u + 478u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_478(uint32_t x) { return (x ^ 1804941438u) * 2654435761u + 479u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_479(uint32_t x) { return (x ^ 164409903u) * 2654435761u + 480u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_480(uint32_t x) { return (x ^ 2818845664u) * 2654435761u + 481u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_481(uint32_t x) { return (x ^ 1178314129u) * 2654435761u + 482u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_482(uint32_t x) { return (x ^ 3832749890u) * 2654435761u + 483u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_483(uint32_t x) { return (x ^ 2192218355u) * 2654435761u + 484u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_484(uint32_t x) { return (x ^ 551686820u) * 2654435761u + 485u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_485(uint32_t x) { return (x ^ 3206122581u) * 2654435761u + 486u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_486(uint32_t x) { return (x ^ 1565591046u) * 2654435761u + 487u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_487(uint32_t x) { return (x ^ 4220026807u) * 2654435761u + 488u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_488(uint32_t x) { return (x ^ 2579495272u) * 2654435761u + 489u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_489(uint32_t x) { return (x ^ 938963737u) * 2654435761u + 490u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_490(uint32_t x) { return (x ^ 3593399498u) * 2654435761u + 491u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_491(uint32_t x) { return (x ^ 1952867963u) * 2654435761u + 492u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_492(uint32_t x) { return (x ^ 312336428u) * 2654435761u + 493u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_493(uint32_t x) { return (x ^ 2966772189u) * 2654435761u + 494u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_494(uint32_t x) { return (x ^ 1326240654u) * 2654435761u + 495u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_495(uint32_t x) { return (x ^ 3980676415u) * 2654435761u + 496u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_496(uint32_t x) { return (x ^ 2340144880u) * 2654435761u + 497u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_497(uint32_t x) { return (x ^ 699613345u) * 2654435761u + 498u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_498(uint32_t x) { return (x ^ 3354049106u) * 2654435761u + 499u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_499(uint32_t x) { return (x ^ 1713517571u) * 2654435761u + 500u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_500(uint32_t x) { return (x ^ 72986036u) * 2654435761u + 501u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_501(uint32_t x) { return (x ^ 2727421797u) * 2654435761u + 502u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_502(uint32_t x) { return (x ^ 1086890262u) * 2654435761u + 503u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_503(uint32_t x) { return (x ^ 3741326023u) * 2654435761u + 504u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_504(uint32_t x) { return (x ^ 2100794488u) * 2654435761u + 505u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_505(uint32_t x) { return (x ^ 460262953u) * 2654435761u + 506u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_506(uint32_t x) { return (x ^ 3114698714u) * 2654435761u + 507u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_507(uint32_t x) { return (x ^ 1474167179u) * 2654435761u + 508u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_508(uint32_t x) { return (x ^ 4128602940u) * 2654435761u + 509u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_509(uint32_t x) { return (x ^ 2488071405u) * 2654435761u + 510u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_510(uint32_t x) { return (x ^ 847539870u) * 2654435761u + 511u; }
static uint32_t __attribute__((noinline, aligned(128)))
xf_511(uint32_t x) { return (x ^ 3501975631u) * 2654435761u + 512u; }

static xf_t const xf_flash_table[XIP_BENCH_NFN] = {
    xf_0, xf_1, xf_2, xf_3, xf_4, xf_5, xf_6, xf_7,
    xf_8, xf_9, xf_10, xf_11, xf_12, xf_13, xf_14, xf_15,
    xf_16, xf_17, xf_18, xf_19, xf_20, xf_21, xf_22, xf_23,
    xf_24, xf_25, xf_26, xf_27, xf_28, xf_29, xf_30, xf_31,
    xf_32, xf_33, xf_34, xf_35, xf_36, xf_37, xf_38, xf_39,
    xf_40, xf_41, xf_42, xf_43, xf_44, xf_45, xf_46, xf_47,
    xf_48, xf_49, xf_50, xf_51, xf_52, xf_53, xf_54, xf_55,
    xf_56, xf_57, xf_58, xf_59, xf_60, xf_61, xf_62, xf_63,
    xf_64, xf_65, xf_66, xf_67, xf_68, xf_69, xf_70, xf_71,
    xf_72, xf_73, xf_74, xf_75, xf_76, xf_77, xf_78, xf_79,
    xf_80, xf_81, xf_82, xf_83, xf_84, xf_85, xf_86, xf_87,
    xf_88, xf_89, xf_90, xf_91, xf_92, xf_93, xf_94, xf_95,
    xf_96, xf_97, xf_98, xf_99, xf_100, xf_101, xf_102, xf_103,
    xf_104, xf_105, xf_106, xf_107, xf_108, xf_109, xf_110, xf_111,
    xf_112, xf_113, xf_114, xf_115, xf_116, xf_117, xf_118, xf_119,
    xf_120, xf_121, xf_122, xf_123, xf_124, xf_125, xf_126, xf_127,
    xf_128, xf_129, xf_130, xf_131, xf_132, xf_133, xf_134, xf_135,
    xf_136, xf_137, xf_138, xf_139, xf_140, xf_141, xf_142, xf_143,
    xf_144, xf_145, xf_146, xf_147, xf_148, xf_149, xf_150, xf_151,
    xf_152, xf_153, xf_154, xf_155, xf_156, xf_157, xf_158, xf_159,
    xf_160, xf_161, xf_162, xf_163, xf_164, xf_165, xf_166, xf_167,
    xf_168, xf_169, xf_170, xf_171, xf_172, xf_173, xf_174, xf_175,
    xf_176, xf_177, xf_178, xf_179, xf_180, xf_181, xf_182, xf_183,
    xf_184, xf_185, xf_186, xf_187, xf_188, xf_189, xf_190, xf_191,
    xf_192, xf_193, xf_194, xf_195, xf_196, xf_197, xf_198, xf_199,
    xf_200, xf_201, xf_202, xf_203, xf_204, xf_205, xf_206, xf_207,
    xf_208, xf_209, xf_210, xf_211, xf_212, xf_213, xf_214, xf_215,
    xf_216, xf_217, xf_218, xf_219, xf_220, xf_221, xf_222, xf_223,
    xf_224, xf_225, xf_226, xf_227, xf_228, xf_229, xf_230, xf_231,
    xf_232, xf_233, xf_234, xf_235, xf_236, xf_237, xf_238, xf_239,
    xf_240, xf_241, xf_242, xf_243, xf_244, xf_245, xf_246, xf_247,
    xf_248, xf_249, xf_250, xf_251, xf_252, xf_253, xf_254, xf_255,
    xf_256, xf_257, xf_258, xf_259, xf_260, xf_261, xf_262, xf_263,
    xf_264, xf_265, xf_266, xf_267, xf_268, xf_269, xf_270, xf_271,
    xf_272, xf_273, xf_274, xf_275, xf_276, xf_277, xf_278, xf_279,
    xf_280, xf_281, xf_282, xf_283, xf_284, xf_285, xf_286, xf_287,
    xf_288, xf_289, xf_290, xf_291, xf_292, xf_293, xf_294, xf_295,
    xf_296, xf_297, xf_298, xf_299, xf_300, xf_301, xf_302, xf_303,
    xf_304, xf_305, xf_306, xf_307, xf_308, xf_309, xf_310, xf_311,
    xf_312, xf_313, xf_314, xf_315, xf_316, xf_317, xf_318, xf_319,
    xf_320, xf_321, xf_322, xf_323, xf_324, xf_325, xf_326, xf_327,
    xf_328, xf_329, xf_330, xf_331, xf_332, xf_333, xf_334, xf_335,
    xf_336, xf_337, xf_338, xf_339, xf_340, xf_341, xf_342, xf_343,
    xf_344, xf_345, xf_346, xf_347, xf_348, xf_349, xf_350, xf_351,
    xf_352, xf_353, xf_354, xf_355, xf_356, xf_357, xf_358, xf_359,
    xf_360, xf_361, xf_362, xf_363, xf_364, xf_365, xf_366, xf_367,
    xf_368, xf_369, xf_370, xf_371, xf_372, xf_373, xf_374, xf_375,
    xf_376, xf_377, xf_378, xf_379, xf_380, xf_381, xf_382, xf_383,
    xf_384, xf_385, xf_386, xf_387, xf_388, xf_389, xf_390, xf_391,
    xf_392, xf_393, xf_394, xf_395, xf_396, xf_397, xf_398, xf_399,
    xf_400, xf_401, xf_402, xf_403, xf_404, xf_405, xf_406, xf_407,
    xf_408, xf_409, xf_410, xf_411, xf_412, xf_413, xf_414, xf_415,
    xf_416, xf_417, xf_418, xf_419, xf_420, xf_421, xf_422, xf_423,
    xf_424, xf_425, xf_426, xf_427, xf_428, xf_429, xf_430, xf_431,
    xf_432, xf_433, xf_434, xf_435, xf_436, xf_437, xf_438, xf_439,
    xf_440, xf_441, xf_442, xf_443, xf_444, xf_445, xf_446, xf_447,
    xf_448, xf_449, xf_450, xf_451, xf_452, xf_453, xf_454, xf_455,
    xf_456, xf_457, xf_458, xf_459, xf_460, xf_461, xf_462, xf_463,
    xf_464, xf_465, xf_466, xf_467, xf_468, xf_469, xf_470, xf_471,
    xf_472, xf_473, xf_474, xf_475, xf_476, xf_477, xf_478, xf_479,
    xf_480, xf_481, xf_482, xf_483, xf_484, xf_485, xf_486, xf_487,
    xf_488, xf_489, xf_490, xf_491, xf_492, xf_493, xf_494, xf_495,
    xf_496, xf_497, xf_498, xf_499, xf_500, xf_501, xf_502, xf_503,
    xf_504, xf_505, xf_506, xf_507, xf_508, xf_509, xf_510, xf_511,
};
