#!/usr/bin/env python3
"""測定気圧と真の絶対高度から QNH (海面更正気圧) を逆算する。

機体は気圧センサの生値 [Pa] をテレメトリで送ってくる。外部カメラによる位置
推定から、その瞬間の真の絶対高度 [m] が別途分かる。この 2 つから QNH を
逆算して `QNH <hPa>` として機体へ返せば、機体の高度が海抜になる。
地上の気圧センサーでも、その設置標高を高度に入れれば同じ式で出せる。

  h = 44330 * (1 - (p / p_ref)^(1/5.255))   を p_ref について解く

★★**黙って間違った値を出さないこと**が最優先。高度の基準がずれると機体の
  高度保持が地面より下を狙う。範囲外は採用せず、終了コードで失敗を伝える。
★中央値は「各サンプルから出した QNH の中央値」を取る。気圧と高度をそれぞれ
  中央値にしてから 1 回計算すると、同時に観測されたペアでない組み合わせから
  計算してしまう (作成者の判断。妥当と確認した)。
★機体側の受け口は 800-1100 hPa しか受け付けない。ここで弾いておけば、
  範囲外を送って黙って無視されるより早く気づける。

初版は Gemini 3.7 Flash が生成し、こちらで検証して取り込んだ:
  * 独立に検算 (QNH 101991.04 Pa)、往復で高度が 12.5000 m に戻ることを確認
  * CSV の不正行スキップ / 範囲外で rc≠0 / traceback を出さないことを確認
"""
import sys
import argparse
import csv
import json
import statistics

# 定数定義
# 範囲外の数値を渡すと機体の動作が危険になるため、厳格に定義する
P_MIN, P_MAX = 30000.0, 120000.0
H_MIN, H_MAX = -500.0, 10000.0
QNH_MIN, QNH_MAX = 80000.0, 110000.0

def calculate_qnh(pressure_pa, altitude_m):
    """
    国際標準大気式に基づき QNH [Pa] を逆算する。
    式: QNH = p / (1 - h/44330)^5.255
    """
    try:
        qnh = pressure_pa / ((1.0 - (altitude_m / 44330.0)) ** 5.255)
        return qnh
    except ZeroDivisionError:
        return float('inf')
    except ValueError:
        return float('nan')

def validate_and_calculate(p, h):
    """入力値と計算結果の妥当性検査を行う"""
    if not (P_MIN <= p <= P_MAX):
        raise ValueError(f"Pressure {p} Pa out of valid range ({P_MIN}-{P_MAX})")
    if not (H_MIN <= h <= H_MAX):
        raise ValueError(f"Altitude {h} m out of valid range ({H_MIN}-{H_MAX})")
    
    qnh = calculate_qnh(p, h)
    
    if not (QNH_MIN <= qnh <= QNH_MAX):
        # 機体仕様外の値は致命的なミスに繋がるため、単なるエラーではなく警告扱いにする
        raise ArithmeticError(f"Calculated QNH {qnh:.2f} Pa out of safety range ({QNH_MIN}-{QNH_MAX})")
    
    return qnh

def main():
    parser = argparse.ArgumentParser(description="Calculate QNH from pressure and altitude.")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--pressure", type=float, help="Pressure in Pa")
    group.add_argument("--csv", help="CSV file with pressure_pa,altitude_m")
    parser.add_argument("--altitude", type=float, help="Altitude in m")
    parser.add_argument("--json", action="store_true", help="Output in JSON format")
    
    args = parser.parse_args()

    qnh_values = []

    if args.pressure is not None:
        if args.altitude is None:
            print("Error: --altitude is required when --pressure is provided.", file=sys.stderr)
            return 1
        try:
            qnh_values.append(validate_and_calculate(args.pressure, args.altitude))
        except (ValueError, ArithmeticError) as e:
            print(f"Error: {e}", file=sys.stderr)
            return 1
    elif args.csv:
        try:
            with open(args.csv, 'r') as f:
                reader = csv.reader(f)
                first_row = next(reader, None)
                if not first_row:
                    print("Error: CSV file is empty.", file=sys.stderr)
                    return 1
                
                # ヘッダスキップ判定: 各カラムが数値に変換できない場合はヘッダとみなす
                try:
                    float(first_row[0])
                    rows = [first_row] + list(reader)
                except ValueError:
                    rows = list(reader)
                
                for i, row in enumerate(rows, 1):
                    try:
                        p, h = float(row[0]), float(row[1])
                        qnh_values.append(validate_and_calculate(p, h))
                    except (ValueError, ArithmeticError, IndexError) as e:
                        print(f"Skipping row {i}: {e}", file=sys.stderr)
        except Exception as e:
            print(f"Error reading CSV: {e}", file=sys.stderr)
            return 1

    if not qnh_values:
        print("Error: No valid data found.", file=sys.stderr)
        return 1

    # 中央値計算：外れ値に強いという要件を満たすため、個々の計算結果から中央値を取る
    # 個々の計算結果から取る理由: 気圧と高度のペアを維持した状態でQNHを計算してから統計処理を行わないと、
    # 観測時刻の相関が壊れ、誤ったQNHが算出される可能性があるため。
    final_qnh = statistics.median(qnh_values)
    
    hpa = final_qnh / 100.0
    cmd = f"QNH {hpa:.2f}"

    if args.json:
        print(json.dumps({"qnh_pa": int(round(final_qnh)), "qnh_hpa": round(hpa, 2), "command": cmd}))
    else:
        print(f"QNH: {int(round(final_qnh))} Pa")
        print(f"QNH: {hpa:.2f} hPa")
        print(f"Command: {cmd}")

    return 0

if __name__ == "__main__":
    sys.exit(main())
