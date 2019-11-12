import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.widgets import Button, TextBox

fig = plt.figure(constrained_layout=True)
fig.suptitle("Pendulum Controller")

gs = GridSpec(4, 3, figure=fig, wspace=0.05, height_ratios=[.3,.3,.3,.1])
ax_position = fig.add_subplot(gs[:1, :])
ax_angle = fig.add_subplot(gs[1:2, :])
ax_command = fig.add_subplot(gs[2:3, :])

ax_button_start = fig.add_subplot(gs[3, :1])
ax_button_stop = fig.add_subplot(gs[3, 1:2])
ax_button_calib = fig.add_subplot(gs[3, 2:])

button_start = Button(ax_button_start, 'Start')
button_stop = Button(ax_button_stop, 'Stop')
button_calib = Button(ax_button_calib, 'Calibrate')



#ax_text_cos_mag1 = fig.add_subplot(gs[1, 1])
#ax_text_cos_mag2 = fig.add_subplot(gs[2, 1])
#ax_text_cos_mag3 = fig.add_subplot(gs[3, 1])
#ax_text_cos_freq1 = fig.add_subplot(gs[1, 2])
#ax_text_cos_freq2 = fig.add_subplot(gs[2, 2])
#ax_text_cos_freq3 = fig.add_subplot(gs[3, 2])
#ax_text_cos_ph1 = fig.add_subplot(gs[1, 3])
#ax_text_cos_ph2 = fig.add_subplot(gs[2, 3])
#ax_text_cos_ph3 = fig.add_subplot(gs[3, 3])
#
#
#text_cos_mag1 = TextBox(ax_text_cos_mag1, 'Cos1', initial='0.0')
#text_cos_mag2 = TextBox(ax_text_cos_mag2, 'Cos2', initial='0.0')
#text_cos_mag3 = TextBox(ax_text_cos_mag3, 'Cos3', initial='0.0')
#
#text_cos_freq1 = TextBox(ax_text_cos_freq1, None, initial='0.0')
#text_cos_freq2 = TextBox(ax_text_cos_freq2, None, initial='0.0')
#text_cos_freq3 = TextBox(ax_text_cos_freq3, None, initial='0.0')
#
#ax_text_step_mag1 = fig.add_subplot(gs[5, 2])
#ax_text_step_mag2 = fig.add_subplot(gs[6, 2])
#ax_text_step_mag3 = fig.add_subplot(gs[7, 2])
#ax_text_step_mag3 = fig.add_subplot(gs[8, 2])
#ax_text_step_ph1 = fig.add_subplot(gs[5, 3])
#ax_text_step_ph2 = fig.add_subplot(gs[6, 3])
#ax_text_step_ph3 = fig.add_subplot(gs[7, 3])
#ax_text_step_ph3 = fig.add_subplot(gs[8, 3])

plt.show()

