import matplotlib.pyplot as plt
# plt.rcParams['font.family'] = 'Apple Color Emoji'
from matplotlib.animation import FuncAnimation
import numpy as np

def animate_trajectory(RM1, RM2, RT1, RT2, T, skip=50, save_path=None):
    
    if save_path:
        max_frames = 360
        skip = max(1, len(RM1) // max_frames)

    RM1 = RM1[::skip]
    RM2 = RM2[::skip]
    RT1 = RT1[::skip]
    RT2 = RT2[::skip]
    T   =   T[::skip]
     
    fig, ax = plt.subplots(figsize=(6,4))

    all_x = np.concatenate([RM1, RT1])
    all_y = np.concatenate([RM2, RT2])

    ax.set_xlim(0.975*np.min(all_x), 1.025*np.max(all_x))
    ax.set_ylim(0.975*np.min(all_y), 1.025*np.max(all_y))
    # ax.set_aspect('equal')

    missile_dot, = ax.plot([], [], 'ro', label="Missile")
    target_dot,  = ax.plot([], [], 'bo', label="Target")

    missile_path, = ax.plot([], [], 'r--', alpha=0.5)
    target_path,  = ax.plot([], [], 'b--', alpha=0.5)
    los_line,     = ax.plot([], [], 'g-',  alpha=0.3)

    ax.legend()
    ax.grid()
    
    # --- ADD THIS BLOCK ---
    text = ax.text(
        0.02, 0.95, '',
        transform=ax.transAxes,
        fontsize=12,
        verticalalignment='top',
        bbox=dict(facecolor='black', alpha=0.6),
        color='lime'
    )
    # ----------------------

    def init():
        text.set_text('')
        return missile_dot, target_dot, missile_path, target_path, los_line, text

    def update(frame):
        real_frame = min(frame, frames - 1)
        
        missile_dot.set_data([RM1[real_frame]], [RM2[real_frame]])
        target_dot.set_data ([RT1[real_frame]], [RT2[real_frame]])

        missile_path.set_data(RM1[:real_frame], RM2[:real_frame])
        target_path.set_data (RT1[:real_frame], RT2[:real_frame])

        los_line.set_data(
            [RM1[real_frame], RT1[real_frame]],
            [RM2[real_frame], RT2[real_frame]]
        )

        R = np.hypot(RM1[real_frame] - RT1[real_frame],
                     RM2[real_frame] - RT2[real_frame])

        text.set_text(
            f'T = {T[real_frame]:.2f} s\n'
            f'R = {R:.2f} m'
        )
        
        ax.set_title("Intercept ***" if R < 50 else "Proportional Navigation")
        
        if not save_path and frame == frames - 1:
            ani.event_source.stop()
            
        return missile_dot, target_dot, missile_path, target_path, los_line, text


    frames       = min(len(RM1), len(RT1))
    pause_frames = 40 if save_path else 0
    frames_total = frames + pause_frames


    ani = FuncAnimation(
        fig,
        update,
        frames      =   frames_total,
        init_func   =   init,
        interval    =   400,
        blit        =   True
    )


    if save_path:
        ani.save(save_path, writer="pillow", fps = 30)
    #else:
    #    plt.show()
    plt.show()

    return ani



if __name__ == "__main__":
    import numpy as np

    data = np.loadtxt("output/csv/run_default_xnt_t.csv",
                      delimiter=",",
                      skiprows=1)[:, 0:5]

    RM1, RM2, RT1, RT2, T = data.T

    animate_trajectory(RM1, RM2, RT1, RT2, T, save_path = "output/gifs/intercept_xnt_t.gif")
