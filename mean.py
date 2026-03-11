import argparse

def calculate_column_averages(file_path):
    """
    从txt文件读取数据并计算每列的平均值
    
    参数:
        file_path: 文本文件的路径（如 "data.txt"）
    返回:
        每列的平均值列表
    """
    # 存储所有有效数据行
    data = []
    
    try:
        # 打开并读取文件
        with open(file_path, 'r', encoding='utf-8') as file:
            for line_num, line in enumerate(file, 1):
                # 去除行首尾的空白字符（换行、空格等）
                stripped_line = line.strip()
                
                # 跳过空行
                if not stripped_line:
                    continue
                
                # 按空格分割每行数据（兼容多个空格分隔的情况）
                parts = stripped_line.split()
                
                try:
                    # 将分割后的字符串转换为浮点数
                    row_data = [float(part) for part in parts]
                    data.append(row_data)
                except ValueError:
                    print(f"警告：第 {line_num} 行数据格式错误，已跳过：{stripped_line}")
        
        # 检查是否读取到有效数据
        if not data:
            print("错误：文件中没有有效数据！")
            return []
        
        # 获取行数和列数
        rows = len(data)
        cols = len(data[0])
        
        # 验证所有行的列数是否一致
        for row_num, row in enumerate(data, 1):
            if len(row) != cols:
                print(f"警告：第 {row_num} 行的列数与第一行不一致，已跳过")
                data.remove(row)
                rows -= 1
        
        # 计算每列平均值
        column_averages = []
        for col in range(cols):
            total = sum(row[col] for row in data)
            average = round(total / rows, 4)  # 保留4位小数
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
    # 创建命令行参数解析器
    parser = argparse.ArgumentParser(description='计算文本文件每列的平均值')
    
    # 添加文件名参数
    parser.add_argument('--file_name', type=str, required=True, 
                       help='要处理的文本文件路径')
    
    # 解析参数
    args = parser.parse_args()
    
    # 计算每列平均值
    averages = calculate_column_averages(args.file_name)
    
    # 输出结果
    if averages:
        for idx, avg in enumerate(averages):
            if idx==1:
                print(f"Avg Success: {avg}")
            elif idx == 2:
                print(f"Avg Collision: {avg}")
            elif idx == 3:
                print(f"Avg Timeout: {avg}")
            elif idx == 4 :
                print(f"Avf Time: {avg}")
            elif idx == 5:
                print(f"Avg Score: {avg}")