**Install python requirements:**

```python3 -m pip install -r requirements.txt```

**Install librdb python wheel:**

```pip3 install librdb/riegl.rdb-2.3.4-cp34.cp35.cp36.cp37.cp38.cp39-none-linux_x86_64.whl```

**Build package:**

Switch to workspace root directory.

```colcon build```

**Run node executable:**

Open a second terminal. Execute '. install/setup_bash.sh'.

```ros2 run riegl_vz riegl_vz H2222222```

**Run node executable with launch configuration:**

Open a second terminal. Execute '. install/setup_bash.sh'.

Edit parameters in package install directory under 'install/riegl_vz/share/riegl_vz/config/params.yaml'.

Launch riegl_vz node:

```ros2 launch riegl_vz riegl_vz std_launch.py```
