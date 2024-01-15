# Pallet Pick and Place

## Installation Instructions
1. Install autolab_core, pyyaml, rospkg, pyquaternion
    ```bash
    pip install autolab_core pyyaml rospkg pyquaternion
    ```
2. Copy the dummy config to yk-god
    ```bash
    scp config/dummy-1920x1080.conf mfi@yk-god:~/
    ```
3. Add the following to the .bash_aliases file on yk-god
    ```bash
    alias start_dummy_screen='sudo X -config dummy-1920x1080.conf'
    ```
4. Install testbed_lin_actuator and testbed_camera_utils to yk-god.
5. Copy over the yk_god.launch file to yk-god
    ```bash
    scp launch/yk_god.launch mfi@yk-god:~/
    ```

## Running Instructions
1. Enable the dummy screen on yk-god
    ```bash
    ssh mfi@yk-god
    mfi@yk-god$ start_dummy_screen
    ```
2. Start the linear actuator server, cameras, and amr detection on yk-god
    ```bash
    ssh -X mfi@yk-god
    mfi@yk-god$ export DISPLAY=:0
    mfi@yk-god$ roslaunch yk_god.launch
    ```
3.  Start the pallet pick and place server for each robot using the namespace argument
    ```bash
    roslaunch pallet_pick_and_place pallet_pick_and_place_server.launch namespace:=yk_creator
    ```
4. Start the resource server
    ```bash
    roslaunch pallet_pick_and_place yk_robots_resource_server.py
    ```
5. Send a pick and place command
    ```bash
    python
    import requests
    task1 = {'msgType': 'StartTask', 'taskId': 1, 'name': 'unloadKit', 'resources': ['robotArm1', 'amr2'], 'structureType': 'human', 'location': 'Robot-Arm-1'}
    task2 = {'msgType': 'StartTask', 'taskId': 2, 'name': 'assembleStructure', 'resources': ['robotArm1'], 'structureType': 'human', 'location': 'Robot-Arm-1'}
    task3 = {'msgType': 'StartTask', 'taskId': 3, 'name': 'loadStructure', 'resources': ['robotArm1', 'amr2'], 'structureType': 'human', 'location': 'Robot-Arm-1'}
    task4 = {'msgType': 'StartTask', 'taskId': 4, 'name': 'unloadStructure', 'resources': ['robotArm1', 'amr2'], 'structureType': 'human', 'location': 'Robot-Arm-1'}
    task5 = {'msgType': 'StartTask', 'taskId': 5, 'name': 'disassembleStructure', 'resources': ['robotArm1'], 'structureType': 'human', 'location': 'Robot-Arm-1'}
    task6 = {'msgType': 'StartTask', 'taskId': 6, 'name': 'loadKit', 'resources': ['robotArm1', 'amr2'], 'structureType': 'human', 'location': 'Robot-Arm-1'}
    requests.post('http://192.168.1.2:9089/execution', json=task1)
    ```