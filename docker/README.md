To build the docker image:
~~~
./build.sh
~~~

The desktop base image uses AMD64. On ARM64 hosts Docker needs AMD64 emulation.
MAVROS is built from the pinned source revision in `workspace.repos`, since
the Humble apt repository may provide `mavros_msgs` without the `mavros` runtime.

To launch Gazebo, RViz, all nodes:
~~~
./run.sh
ros2 launch orca_bringup sim_launch.py
~~~

## macOS (Apple Silicon)

There is no NVIDIA GPU on a Mac, so `run.sh` renders in the container on a
virtual screen and serves it over VNC. The amd64 image runs under Rosetta,
where the Gazebo GUI deadlocks, so Gazebo's server runs in the orca4 container
and its GUI runs natively in a small arm64 sidecar container that draws on
the same screen.

Run all host commands from this `docker/` directory, in the terminal where
`docker` works (e.g. your OrbStack Linux machine). Commands marked "inside
the container" go in the container shell (prompt `orca4@<id>`), where
`docker` is not available.

### Download the published images (no build needed)

Pull both images published for the `docker_fixes_mac` branch:

~~~bash
docker pull ghcr.io/illini-robosub/orca4:docker_fixes_mac
docker pull ghcr.io/illini-robosub/orca4-gz-gui:docker_fixes_mac
~~~

If access is denied because the packages are private, first run
`docker login ghcr.io -u YOUR_GITHUB_USERNAME` using a GitHub personal access
token (classic) with `read:packages` and an account that can access the packages.

Give the downloaded images the local names used by the launch scripts:

~~~bash
docker tag ghcr.io/illini-robosub/orca4:docker_fixes_mac orca4:latest
docker tag ghcr.io/illini-robosub/orca4-gz-gui:docker_fixes_mac orca4-gz-gui:latest
~~~

No Dockerfile copy or build is needed. Tags are additional names for the same
image data, not duplicate copies. Pulling again reuses layers already stored
locally. If you already have these exact images, Docker will reuse them.
The tag commands replace what the local `latest` names point to, so they select
the published versions for the launch scripts.

### Build from source (alternative to downloading)

~~~
./build.sh    # orca4 image (amd64, emulated; takes a few hours)
docker build --platform linux/arm64 -f Dockerfile.gz-gui -t orca4-gz-gui:latest .
~~~
Rebuild the GUI image after rebuilding orca4 if models or worlds changed;
it copies them from `orca4:latest`.

### Launch Gazebo, RViz, all nodes
1. Terminal 1 (host): start the container. You end up in its shell.
   ~~~
   ./run.sh
   ~~~
2. Terminal 1 (inside the container): start the simulation without the
   emulated Gazebo GUI. `gzclient:=False` is required; without it the
   emulated GUI starts and hangs.
   ~~~
   ros2 launch orca_bringup sim_launch.py gzclient:=False
   ~~~
3. Terminal 2 (host): attach the native Gazebo GUI.
   ~~~
   ./run-gz-gui.sh
   ~~~
4. On the Mac: Finder > Go > Connect to Server > `vnc://localhost:5900`,
   password `orca4pass`. RViz appears first; the Gazebo window fills in
   within a minute or so. Switch between them from the taskbar at the
   bottom, and drag or resize windows as needed.

### Test the downloaded images

After pulling and tagging, follow the launch steps above using the scripts in
this checkout's `docker/` directory. You still need this directory because
`run.sh` mounts `vnc-entrypoint.sh` from it; you do not need to rebuild an image.
On Apple Silicon, enable AMD64 emulation in your Docker environment.

Confirm that both containers are running from another host terminal:

~~~bash
docker ps --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
~~~

Expect `orca4` and `orca4-gz-gui` to be listed. In the VNC viewer, confirm that
RViz opens and Gazebo displays the simulation world. Container status alone
does not verify that the simulation is working. Stop the test using the steps
below; stopping the containers leaves the downloaded images available for reuse.

### Stop / exit

- **The simulation (`ros2 launch`):** press Ctrl+C in Terminal 1 while the
  launch is running, and wait for the nodes to shut down.
- **An extra container shell** (opened with `docker exec -it orca4 bash`):
  type `exit` or press Ctrl+D. The container keeps running.
- **The orca4 container:** type `exit` or press Ctrl+D in the **first**
  shell, the one `./run.sh` opened (Terminal 1). `run.sh` uses `--rm`, so
  the container is also deleted.
- **The orca4 container, from outside:** on the host, run
  `docker rm -f orca4`.
- **The Gazebo GUI sidecar:** on the host, run `docker rm -f orca4-gz-gui`.
  It has no shell to exit.
- **The VNC viewer:** close the Screen Sharing window on the Mac. The
  simulation keeps running.
- **Everything at once:** on the host, run
  `docker rm -f orca4-gz-gui orca4`.

To leave the container shell **without** stopping the container, press
Ctrl+P then Ctrl+Q (detach). Reattach with `docker attach orca4`, or open a
new shell with `docker exec -it orca4 bash`.

Stopping orca4 (or its display) also stops the GUI sidecar, which removes
itself; if `./run-gz-gui.sh` still reports the name is in use, run
`docker rm -f orca4-gz-gui` first.

Check what's still running with `docker ps`.

### Troubleshooting
- **Gazebo window is black:** the simulation isn't running yet (step 2),
  or is still starting. The GUI connects on its own once the server is up.
- **"container name /orca4-gz-gui is already in use":** the GUI is already
  running. To restart it, run `docker rm -f orca4-gz-gui` first.
- **`rviz2`/`ardusub`/`ros_gz_image` not found:** `ros2 launch` ran on the
  host instead of inside the container. Use Terminal 1, or open another
  container shell with `docker exec -it orca4 bash`.
- **VNC "Connection failed":** check that `orca4` is running (`docker ps`).
  The container stops when you `exit` its first shell.
- **Simulation runs slower than real time:** expected; the Gazebo server is
  emulated and runs at roughly 40-50% of real time.

To execute a mission:
~~~
docker exec -it orca4 /bin/bash
ros2 run orca_bringup mission_runner.py
~~~
