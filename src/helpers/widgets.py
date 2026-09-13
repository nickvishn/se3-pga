import ipywidgets as widgets


def motion_controls():
    vx = widgets.FloatSlider(value=1.0, min=-2.0, max=2.0, step=0.1, description="v_x")
    vy = widgets.FloatSlider(value=0.0, min=-2.0, max=2.0, step=0.1, description="v_y")
    vz = widgets.FloatSlider(value=0.2, min=-2.0, max=2.0, step=0.1, description="v_z")

    wx = widgets.FloatSlider(
        value=0.0, min=-3.0, max=3.0, step=0.1, description="omega_x"
    )
    wy = widgets.FloatSlider(
        value=0.0, min=-3.0, max=3.0, step=0.1, description="omega_y"
    )
    wz = widgets.FloatSlider(
        value=1.0, min=-3.0, max=3.0, step=0.1, description="omega_z"
    )

    return vx, vy, vz, wx, wy, wz


def scene_controls():
    lx = widgets.FloatSlider(value=1.0, min=0.2, max=3.0, step=0.1, description="L_x")
    ly = widgets.FloatSlider(value=0.6, min=0.2, max=3.0, step=0.1, description="L_y")
    lz = widgets.FloatSlider(value=0.4, min=0.2, max=3.0, step=0.1, description="L_z")

    dt = widgets.FloatSlider(value=0.05, min=0.01, max=0.2, step=0.01, description="dt")
    steps = widgets.IntSlider(value=150, min=20, max=400, step=10, description="steps")
    frame_scale = widgets.FloatSlider(
        value=0.4, min=0.1, max=1.5, step=0.1, description="frame"
    )

    return lx, ly, lz, dt, steps, frame_scale


def playback_controls(max_steps=150):
    time_slider = widgets.IntSlider(
        value=0, min=0, max=max_steps, step=1, description="t step"
    )
    play = widgets.Play(
        value=0, min=0, max=max_steps, step=1, interval=80, description="Play"
    )
    widgets.jslink((play, "value"), (time_slider, "value"))

    show_trace = widgets.Checkbox(value=True, description="show trajectory")
    show_frame = widgets.Checkbox(value=True, description="show body frame")
    show_start_body = widgets.Checkbox(value=True, description="show start pose")

    return time_slider, play, show_trace, show_frame, show_start_body


def bind_time_range(steps, time_slider, play):
    def refresh_time_range(*args):
        time_slider.max = steps.value
        play.max = steps.value
        if time_slider.value > steps.value:
            time_slider.value = steps.value

    steps.observe(refresh_time_range, names="value")
    refresh_time_range()
    return refresh_time_range
