import matplotlib.pyplot as plt
import numpy as np

#! Construct Different Place Descriptor
# 构造不同的位置描述符字典
lcd_names = ['PointNetVLAD', 'PCAN', 'OverlapNet', 'SphereVLAD']
LCD = { lcd_names[0]: [97,34,87,79,31],
        lcd_names[1]: [96,38,92,81,43],
        lcd_names[2]: [93,89,85,71,53],
        lcd_names[3]: [95,92,81,87,95]}

# Each attribute we'll plot in the radar chart.
# 定义要在雷达图上绘制的属性标签
labels = ['Condition', 'Viewpoint', 'Accuracy', 'Generalization', 'Efficiency']

# Number of variables we're plotting.
# 计算标签数量
num_vars = len(labels)

# Split the circle into even parts and save the angles
# so we know where to put each axis.
# 将圆分割为等份并保存角度以确定每个轴的位置
angles = np.linspace(0, 2 * np.pi, num_vars, endpoint=False).tolist()

# The plot is a circle, so we need to "complete the loop"
# and append the start value to the end.
# 为了形成闭环，将起始值添加到末尾
angles += angles[:1]

# 创建极坐标图
# ax = plt.subplot(polar=True)
fig, ax = plt.subplots(figsize=(6, 6), subplot_kw=dict(polar=True))

# Helper function to plot each car on the radar chart.
# 辅助函数，用于绘制雷达图中的每个位置描述符
def add_to_radar(car_model, color):
  # values = dft.loc[car_model].tolist()
  values = LCD[car_model]
  values += values[:1]
  ax.plot(angles, values, color=color, linewidth=1, label=car_model)
  ax.fill(angles, values, color=color, alpha=0.25)

#! Add each car to the chart.
# 将每个位置描述符添加到雷达图中
add_to_radar(lcd_names[0], '#1aaf6c')
add_to_radar(lcd_names[1], '#429bf4')
add_to_radar(lcd_names[2], '#d42cea')
add_to_radar(lcd_names[3], '#f9a9c7')

# Fix axis to go in the right order and start at 12 o'clock.
# 调整极坐标轴的方向和起始位置
ax.set_theta_offset(np.pi / 2)
ax.set_theta_direction(-1)

# Draw axis lines for each angle and label.
# 绘制雷达图的轴线和标签
ax.set_thetagrids(np.degrees(angles), labels)

# Go through labels and adjust alignment based on where
# it is in the circle.
# 根据角度调整轴标签的对齐方式
for label, angle in zip(ax.get_xticklabels(), angles):
  if angle in (0, np.pi):
    label.set_horizontalalignment('center')
  elif 0 < angle < np.pi:
    label.set_horizontalalignment('left')
  else:
    label.set_horizontalalignment('right')

# Ensure radar goes from 0 to 100.
# 设置雷达图的值范围
ax.set_ylim(0, 100)
# You can also set gridlines manually like this:
# 设置雷达图的网格线
ax.set_rgrids([20, 40, 60, 80, 100])

# Set position of y-labels (0-100) to be in the middle
# of the first two axes.
# 设置y轴标签位置
ax.set_rlabel_position(180 / num_vars)

# Add some custom styling.
# 添加自定义样式
# Change the color of the tick labels.
ax.tick_params(colors='#222222')
# Make the y-axis (0-100) labels smaller.
ax.tick_params(axis='y', labelsize=8)
# Change the color of the circular gridlines.
ax.grid(color='#AAAAAA')
# Change the color of the outermost gridline (the spine).
ax.spines['polar'].set_color('#222222')
# Change the background color inside the circle itself.
ax.set_facecolor('#FAFAFA')

# Add title.
# 添加标题（已注释掉）
# ax.set_title('Star-diagram Evaluation', y=1.08)

# Add a legend as well.
# 添加图例
ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1.1))

# 显示雷达图
plt.show()

# 保存雷达图（已注释掉）
# plt.savefig("star.png")