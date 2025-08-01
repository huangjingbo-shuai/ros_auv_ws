#!/usr/bin/env python3
# import pandas as pd
# import matplotlib.pyplot as plt
# import numpy as np
# import os
# from matplotlib.gridspec import GridSpec

# # 设置中文字体
# plt.rcParams['font.sans-serif'] = ['SimHei']  # 使用黑体
# plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

# def plot_pid_data(csv_file):
#     # 读取CSV数据
#     print(f"读取文件: {csv_file}")
#     df = pd.read_csv(csv_file)
    
#     # 创建一个大图布局
#     plt.figure(figsize=(14, 10))
#     gs = GridSpec(3, 2)
    
#     # 1. Pitch角度和误差
#     ax1 = plt.subplot(gs[0, :])
#     ax1.plot(df['timestamp'], df['pitch_deg'], 'b-', label='Pitch角度')
#     ax1.plot(df['timestamp'], df['error_deg'], 'r-', label='误差')
#     ax1.set_title('Pitch角度和误差')
#     ax1.set_ylabel('角度 (度)')
#     ax1.grid(True)
#     ax1.legend()
    
#     # 2. PID输出
#     ax2 = plt.subplot(gs[1, :])
#     ax2.plot(df['timestamp'], df['pid_output'], 'g-', label='PID输出')
#     ax2.set_title('PID控制器输出')
#     ax2.set_ylabel('输出值')
#     ax2.grid(True)
#     ax2.legend()
    
#     # 3. PID各项贡献
#     ax3 = plt.subplot(gs[2, 0])
#     ax3.plot(df['timestamp'], df['p_term'], 'r-', label='P项')
#     ax3.plot(df['timestamp'], df['i_term'], 'g-', label='I项')
#     ax3.plot(df['timestamp'], df['d_term'], 'b-', label='D项')
#     ax3.set_title('PID各项贡献')
#     ax3.set_xlabel('时间 (秒)')
#     ax3.set_ylabel('贡献值')
#     ax3.grid(True)
#     ax3.legend()
    
#     # 4. 电机PWM值
#     ax4 = plt.subplot(gs[2, 1])
#     ax4.plot(df['timestamp'], df['motor3_pwm'], 'r-', label='电机3 PWM')
#     ax4.plot(df['timestamp'], df['motor4_pwm'], 'b-', label='电机4 PWM')
#     ax4.set_title('电机PWM值')
#     ax4.set_xlabel('时间 (秒)')
#     ax4.set_ylabel('PWM值')
#     ax4.grid(True)
#     ax4.legend()
    
#     # 计算一些统计信息
#     pitch_mean = df['pitch_deg'].mean()
#     pitch_std = df['pitch_deg'].std()
#     error_mean = df['error_deg'].mean()
#     error_std = df['error_deg'].std()
    
#     # 添加统计信息到图表
#     stats_text = f"Pitch均值: {pitch_mean:.2f}°, 标准差: {pitch_std:.2f}°\n"
#     stats_text += f"误差均值: {error_mean:.2f}°, 标准差: {error_std:.2f}°"
#     plt.figtext(0.5, 0.01, stats_text, ha='center', fontsize=10, 
#                 bbox={'facecolor': 'lightgray', 'alpha': 0.5, 'pad': 5})
    
#     # 调整布局
#     plt.tight_layout()
#     plt.subplots_adjust(bottom=0.08)
    
#     # 保存图表
#     output_file = os.path.splitext(csv_file)[0] + '_analysis.png'
#     plt.savefig(output_file, dpi=300)
#     print(f"图表已保存为: {output_file}")
    
#     # 显示图表
#     plt.show()
    
#     # 创建震荡分析图表
#     plt.figure(figsize=(14, 6))
    
#     # 计算震荡频率和幅度
#     if len(df) > 50:  # 确保有足够的数据点
#         # 去除趋势
#         pitch_detrend = df['pitch_deg'] - df['pitch_deg'].rolling(window=20, min_periods=1).mean()
        
#         # 计算FFT
#         N = len(pitch_detrend)
#         T = np.mean(np.diff(df['timestamp']))  # 平均采样间隔
#         yf = np.abs(np.fft.fft(pitch_detrend.values))
#         xf = np.fft.fftfreq(N, T)[:N//2]
#         yf = yf[:N//2]
        
#         # 绘制频谱
#         plt.subplot(1, 2, 1)
#         plt.plot(xf[1:], yf[1:])  # 跳过零频率
#         plt.title('Pitch震荡频谱分析')
#         plt.xlabel('频率 (Hz)')
#         plt.ylabel('幅度')
#         plt.grid(True)
        
#         # 找出主要震荡频率
#         max_idx = np.argmax(yf[1:]) + 1  # 跳过零频率
#         main_freq = xf[max_idx]
#         main_amp = yf[max_idx]
#         plt.axvline(x=main_freq, color='r', linestyle='--')
#         plt.text(main_freq, main_amp, f'{main_freq:.2f} Hz', 
#                  verticalalignment='bottom', horizontalalignment='right')
        
#         # 绘制去趋势后的Pitch信号
#         plt.subplot(1, 2, 2)
#         plt.plot(df['timestamp'], pitch_detrend)
#         plt.title('去趋势后的Pitch信号')
#         plt.xlabel('时间 (秒)')
#         plt.ylabel('Pitch (度)')
#         plt.grid(True)
        
#         # 添加震荡信息
#         osc_text = f"主要震荡频率: {main_freq:.2f} Hz\n周期: {1/main_freq:.2f} 秒"
#         plt.figtext(0.5, 0.01, osc_text, ha='center', fontsize=10, 
#                     bbox={'facecolor': 'lightgray', 'alpha': 0.5, 'pad': 5})
        
#         # 保存震荡分析图
#         osc_file = os.path.splitext(csv_file)[0] + '_oscillation.png'
#         plt.tight_layout()
#         plt.subplots_adjust(bottom=0.15)
#         plt.savefig(osc_file, dpi=300)
#         print(f"震荡分析图已保存为: {osc_file}")
        
#         plt.show()

# # 主程序
# if __name__ == "__main__":
#     # 手动指定文件路径
#     csv_file = "pid_data_20250801_195747.csv"  # 替换为你的实际文件路径
    
#     if os.path.exists(csv_file):  # 检查文件是否存在
#         plot_pid_data(csv_file)
#     else:
#         print(f"文件不存在: {csv_file}")
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from matplotlib.gridspec import GridSpec

def plot_pid_data(csv_file):
    # Read CSV data
    print(f"Reading file: {csv_file}")
    df = pd.read_csv(csv_file)
    
    # Create a large figure layout
    plt.figure(figsize=(14, 10))
    gs = GridSpec(3, 2)
    
    # 1. Pitch angle and error
    ax1 = plt.subplot(gs[0, :])
    ax1.plot(df['timestamp'], df['pitch_deg'], 'b-', label='Pitch angle')
    ax1.plot(df['timestamp'], df['error_deg'], 'r-', label='Error')
    ax1.set_title('Pitch Angle and Error')
    ax1.set_ylabel('Angle (deg)')
    ax1.grid(True)
    ax1.legend()
    
    # 2. PID output
    ax2 = plt.subplot(gs[1, :])
    ax2.plot(df['timestamp'], df['pid_output'], 'g-', label='PID Output')
    ax2.set_title('PID Controller Output')
    ax2.set_ylabel('Output Value')
    ax2.grid(True)
    ax2.legend()
    
    # 3. PID components
    ax3 = plt.subplot(gs[2, 0])
    ax3.plot(df['timestamp'], df['p_term'], 'r-', label='P term')
    ax3.plot(df['timestamp'], df['i_term'], 'g-', label='I term')
    ax3.plot(df['timestamp'], df['d_term'], 'b-', label='D term')
    ax3.set_title('PID Components Contribution')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Component Value')
    ax3.grid(True)
    ax3.legend()
    
    # 4. Motor PWM values
    ax4 = plt.subplot(gs[2, 1])
    ax4.plot(df['timestamp'], df['motor3_pwm'], 'r-', label='Motor3 PWM')
    ax4.plot(df['timestamp'], df['motor4_pwm'], 'b-', label='Motor4 PWM')
    ax4.set_title('Motor PWM Values')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('PWM Value')
    ax4.grid(True)
    ax4.legend()
    
    # Calculate some statistics
    pitch_mean = df['pitch_deg'].mean()
    pitch_std = df['pitch_deg'].std()
    error_mean = df['error_deg'].mean()
    error_std = df['error_deg'].std()
    
    # Add statistics to the plot
    stats_text = f"Pitch mean: {pitch_mean:.2f}°, std: {pitch_std:.2f}°\n"
    stats_text += f"Error mean: {error_mean:.2f}°, std: {error_std:.2f}°"
    plt.figtext(0.5, 0.01, stats_text, ha='center', fontsize=10, 
                bbox={'facecolor': 'lightgray', 'alpha': 0.5, 'pad': 5})
    
    # Adjust layout
    plt.tight_layout()
    plt.subplots_adjust(bottom=0.08)
    
    # Save the plot
    output_file = os.path.splitext(csv_file)[0] + '_analysis.png'
    plt.savefig(output_file, dpi=300)
    print(f"Plot saved as: {output_file}")
    
    # Show the plot
    plt.show()
    
    # Create oscillation analysis plot
    plt.figure(figsize=(14, 6))
    
    # Calculate oscillation frequency and amplitude
    if len(df) > 50:  # Ensure enough data points
        # Detrend the data
        pitch_detrend = df['pitch_deg'] - df['pitch_deg'].rolling(window=20, min_periods=1).mean()
        
        # Calculate FFT
        N = len(pitch_detrend)
        T = np.mean(np.diff(df['timestamp']))  # Average sampling interval
        yf = np.abs(np.fft.fft(pitch_detrend.values))
        xf = np.fft.fftfreq(N, T)[:N//2]
        yf = yf[:N//2]
        
        # Plot frequency spectrum
        plt.subplot(1, 2, 1)
        plt.plot(xf[1:], yf[1:])  # Skip zero frequency
        plt.title('Pitch Oscillation Frequency Spectrum')
        plt.xlabel('Frequency (Hz)')
        plt.ylabel('Amplitude')
        plt.grid(True)
        
        # Find dominant oscillation frequency
        max_idx = np.argmax(yf[1:]) + 1  # Skip zero frequency
        main_freq = xf[max_idx]
        main_amp = yf[max_idx]
        plt.axvline(x=main_freq, color='r', linestyle='--')
        plt.text(main_freq, main_amp, f'{main_freq:.2f} Hz', 
                 verticalalignment='bottom', horizontalalignment='right')
        
        # Plot detrended pitch signal
        plt.subplot(1, 2, 2)
        plt.plot(df['timestamp'], pitch_detrend)
        plt.title('Detrended Pitch Signal')
        plt.xlabel('Time (s)')
        plt.ylabel('Pitch (deg)')
        plt.grid(True)
        
        # Add oscillation info
        osc_text = f"Dominant frequency: {main_freq:.2f} Hz\nPeriod: {1/main_freq:.2f} s"
        plt.figtext(0.5, 0.01, osc_text, ha='center', fontsize=10, 
                    bbox={'facecolor': 'lightgray', 'alpha': 0.5, 'pad': 5})
        
        # Save oscillation plot
        osc_file = os.path.splitext(csv_file)[0] + '_oscillation.png'
        plt.tight_layout()
        plt.subplots_adjust(bottom=0.15)
        plt.savefig(osc_file, dpi=300)
        print(f"Oscillation plot saved as: {osc_file}")
        
        plt.show()

# Main program
if __name__ == "__main__":
    # Manually specify file path
    csv_file = "pid_data_20250801_170346.csv"  # Replace with your actual file path
    
    if os.path.exists(csv_file):  # Check if file exists
        plot_pid_data(csv_file)
    else:
        print(f"File not found: {csv_file}")