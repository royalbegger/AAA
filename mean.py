import argparse
import re

def parse_numeric_prefix(parts):
    """
    从字符串列表（按空格分割后的行）中提取开头连续的数值部分。
    返回浮点数列表，一旦遇到无法转换为 float 的项就停止。
    """
    numbers = []
    for part in parts:
        try:
            num = float(part)
            numbers.append(num)
        except ValueError:
            # 遇到非数字，停止解析该行
            break
    return numbers

def calculate_column_averages(file_path):
    """
    从txt文件读取数据并计算每列的平均值。
    每行可以包含数值开头，后面可能有额外文本，会自动忽略文本部分。
    
    参数:
        file_path: 文本文件的路径（如 "data.txt"）
    返回:
        每列的平均值列表
    """
    data = []  # 存储所有有效数据行（每行是数值列表）
    
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            first_row_cols = None  # 记录第一行的数值个数，作为列数基准
            
            for line_num, line in enumerate(file, 1):
                stripped_line = line.strip()
                if not stripped_line:
                    continue  # 跳过空行
                
                parts = stripped_line.split()
                row_data = parse_numeric_prefix(parts)
                
                if not row_data:
                    print(f"警告：第 {line_num} 行开头没有有效数值，已跳过：{stripped_line}")
                    continue
                
                # 确定列数（以第一行有效数值个数为准）
                if first_row_cols is None:
                    first_row_cols = len(row_data)
                    data.append(row_data)
                else:
                    # 检查列数是否一致
                    if len(row_data) == first_row_cols:
                        data.append(row_data)
                    else:
                        print(f"警告：第 {line_num} 行的数值个数（{len(row_data)}）与第一行（{first_row_cols}）不一致，已跳过：{stripped_line}")
        
        if not data:
            print("错误：文件中没有有效数据！")
            return []
        
        rows = len(data)
        cols = first_row_cols
        
        # 计算每列平均值
        column_averages = []
        for col in range(cols):
            total = sum(row[col] for row in data)
            average = round(total / rows, 4)
            column_averages.append(average)
        
        return column_averages
    
    except FileNotFoundError:
        print(f"错误：找不到文件 '{file_path}'，请检查文件路径是否正确！")
        return []
    except Exception as e:
        print(f"读取文件时发生错误：{e}")
        return []

# ------------------- 主程序 -------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='计算文本文件每列的平均值（自动忽略行末尾的文本）')
    parser.add_argument('--file_name', type=str, required=True,
                        help='要处理的文本文件路径')
    args = parser.parse_args()
    
    averages = calculate_column_averages(args.file_name)
    
    if averages:
        # 输出特定列的平均值（索引1~5对应第2~6列，与原始代码保持一致）
        # 注意：若实际列数不足，会触发 IndexError，这里简单处理为输出全部列
        if len(averages) >= 6:
            print(f"Avg Success: {averages[1]}")
            print(f"Avg Collision: {averages[2]}")
            print(f"Avg Timeout: {averages[3]}")
            print(f"Avg Time: {averages[4]}")
            print(f"Avg Score: {averages[5]}")
        else:
            # 如果列数不足6，按实际列数输出，并给出提示
            print("警告：数据列数不足6列，按实际列数输出：")
            for idx, avg in enumerate(averages):
                print(f"Column {idx}: {avg}")