**Install python requirements:**

```python3 -m pip install -r requirements.txt```

**Build package:**

Switch to workspace root directory.

```colcon build```

**Run node executable:**

Open a second terminal. Execute '. install/setup_bash.sh'.

```ros2 run riegl_vz riegl_vz S2222222```

**Run node executable with launch configuration:**

Open a second terminal. Execute '. install/setup_bash.sh'.

```ros2 launch riegl_vz riegl_vz S2222222_launch.py```
